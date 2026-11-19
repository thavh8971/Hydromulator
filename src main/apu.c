#include "apu.h"
#include <stddef.h>
#include <string.h>

static const uint8_t LENGTH_TABLE[32] = {
    10, 254, 20,  2, 40,  4, 80,  6,
    160,  8, 60, 10, 14, 12, 26, 14,
    12,  16, 24, 18, 48, 20, 96, 22,
    192, 24, 72, 26, 16, 28, 32, 30
};

static const uint8_t PULSE_DUTY[4][8] = {
    { 0, 1, 0, 0, 0, 0, 0, 0 },
    { 0, 1, 1, 0, 0, 0, 0, 0 },
    { 0, 1, 1, 1, 1, 0, 0, 0 },
    { 1, 0, 0, 1, 1, 1, 1, 1 }
};

static const uint8_t TRI_SEQ[32] = {
    15, 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4,  3,  2,  1,  0,
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15
};

static const uint16_t NOISE_PERIOD_NTSC[16] = {
    4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068
};

static const uint16_t DMC_PERIOD_NTSC[16] = {
    428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106, 84, 72, 54
};

static inline void update_irq_lines(RicohApu *apu) {
    bool f_irq = apu->frame_irq && !apu->fc_irq_inhibit;
    if (apu->bus.on_irq != NULL) {
        apu->bus.on_irq(apu->bus.ctx, 0x01u, f_irq);
        apu->bus.on_irq(apu->bus.ctx, 0x02u, apu->dmc_irq);
    }
    apu->irq_asserted = f_irq || apu->dmc_irq;
}

static void clock_envelope(ApuEnvelope *env) {
    if (env->start) {
        env->start = false;
        env->decay = 15u;
        env->divider = env->volume;
    } else if (env->divider == 0u) {
        env->divider = env->volume;
        if (env->decay > 0u) {
            env->decay--;
        } else if (env->loop) {
            env->decay = 15u;
        }
    } else {
        env->divider--;
    }
}

static inline uint8_t envelope_output(const ApuEnvelope *env) {
    return env->constant_vol ? env->volume : env->decay;
}

static uint16_t pulse_sweep_target(const ApuPulse *p, bool is_pulse1) {
    uint16_t change = (uint16_t)(p->period >> p->sweep_shift);
    int32_t target = (int32_t)p->period;
    if (p->sweep_negate) {
        target -= (int32_t)change;
        if (is_pulse1) {
            target -= 1;
        }
    } else {
        target += (int32_t)change;
    }
    return (target < 0) ? 0u : (uint16_t)target;
}

static inline bool pulse_muted(const ApuPulse *p, bool is_pulse1) {
    return (p->period < 8u) || (pulse_sweep_target(p, is_pulse1) > 0x7FFu);
}

static void clock_sweep(ApuPulse *p, bool is_pulse1) {
    uint16_t target = pulse_sweep_target(p, is_pulse1);
    bool muted = (p->period < 8u) || (target > 0x7FFu);
    if (p->sweep_divider == 0u && p->sweep_enabled && p->sweep_shift > 0u && !muted) {
        p->period = target;
    }
    if (p->sweep_divider == 0u || p->sweep_reload) {
        p->sweep_divider = p->sweep_period;
        p->sweep_reload = false;
    } else {
        p->sweep_divider--;
    }
}

static inline void clock_length_counter(ApuLengthCounter *lc) {
    if (lc->count > 0u && !lc->halt) {
        lc->count--;
    }
}

static void clock_linear_counter(ApuTriangle *tri) {
    if (tri->linear_reload_flag) {
        tri->linear = tri->linear_reload_val;
    } else if (tri->linear > 0u) {
        tri->linear--;
    }
    if (!tri->linear_control) {
        tri->linear_reload_flag = false;
    }
}

static void clock_quarter_frame(RicohApu *apu) {
    clock_envelope(&apu->pulse1.env);
    clock_envelope(&apu->pulse2.env);
    clock_linear_counter(&apu->tri);
    clock_envelope(&apu->noise.env);
}

static void clock_half_frame(RicohApu *apu) {
    clock_length_counter(&apu->pulse1.lc);
    clock_length_counter(&apu->pulse2.lc);
    clock_length_counter(&apu->tri.lc);
    clock_length_counter(&apu->noise.lc);
    clock_sweep(&apu->pulse1, true);
    clock_sweep(&apu->pulse2, false);
}

static void clock_dmc_output(RicohApu *apu) {
    ApuDmc *dmc = &apu->dmc;

    if (!dmc->silence) {
        if (dmc->shift_reg & 0x01u) {
            if (dmc->output_level <= 125u) {
                dmc->output_level = (uint8_t)(dmc->output_level + 2u);
            }
        } else {
            if (dmc->output_level >= 2u) {
                dmc->output_level = (uint8_t)(dmc->output_level - 2u);
            }
        }
    }

    dmc->shift_reg >>= 1;
    dmc->bits_remaining--;

    if (dmc->bits_remaining == 0u) {
        dmc->bits_remaining = 8u;
        if (dmc->buffer_empty) {
            dmc->silence = true;
        } else {
            dmc->silence = false;
            dmc->shift_reg = dmc->sample_buffer;
            dmc->buffer_empty = true;
            if (dmc->bytes_remaining > 0u && apu->dmc_load_delay == 0u &&
                apu->dmc_dma_phase == APU_DMC_DMA_IDLE) {
                apu->dmc_dma_pending = true;
            } else if (dmc->bytes_remaining == 0u &&
                       (apu->dmc_abort_delay > 0u || apu->dmc_implicit_window > 0u) &&
                       apu->dmc_dma_phase == APU_DMC_DMA_IDLE) {
                apu->dmc_dma_abort_one = true;
            }
        }
    }
}

static float compute_raw_mixer(const RicohApu *apu) {
    uint8_t p1 = 0u;
    if (apu->pulse1.lc.count > 0u && !pulse_muted(&apu->pulse1, true) &&
        PULSE_DUTY[apu->pulse1.duty_mode][apu->pulse1.duty_step]) {
        p1 = envelope_output(&apu->pulse1.env);
    }

    uint8_t p2 = 0u;
    if (apu->pulse2.lc.count > 0u && !pulse_muted(&apu->pulse2, false) &&
        PULSE_DUTY[apu->pulse2.duty_mode][apu->pulse2.duty_step]) {
        p2 = envelope_output(&apu->pulse2.env);
    }

    uint8_t tri = TRI_SEQ[apu->tri.step];
    uint8_t noise = 0u;
    if (apu->noise.lc.count > 0u && (apu->noise.lfsr & 0x01u) == 0u) {
        noise = envelope_output(&apu->noise.env);
    }
    uint8_t dmc = apu->dmc.output_level;

    float pulse_out = 0.0f;
    if ((p1 + p2) > 0u) {
        pulse_out = 95.88f / ((8128.0f / (float)(p1 + p2)) + 100.0f);
    }

    float tnd_out = 0.0f;
    float tnd_sum = ((float)tri / 8227.0f) + ((float)noise / 12241.0f) + ((float)dmc / 22638.0f);
    if (tnd_sum > 0.0f) {
        tnd_out = 159.79f / ((1.0f / tnd_sum) + 100.0f);
    }

    return pulse_out + tnd_out;
}

/* ==================== Public API ==================== */

void ricoh_apu_power_on(RicohApu *apu, const RicohApuBus *bus) {
    memset(apu, 0, sizeof(*apu));
    if (bus != NULL) {
        apu->bus = *bus;
    }
    apu->noise.lfsr = 1u;
    apu->dmc.bits_remaining = 8u;
    apu->dmc.silence = true;
    apu->dmc.buffer_empty = true;
    apu->dmc.period = DMC_PERIOD_NTSC[0];
    apu->dmc.timer = apu->dmc.period;
    apu->fc_reset_delay = 0x80u;
}

void ricoh_apu_reset(RicohApu *apu) {
    ricoh_apu_write(apu, 0x4015u, 0x00u);
    apu->frame_irq = false;
    apu->frame_irq_clear_pending = false;
    apu->dmc_irq = false;
    apu->fc_counter = 0u;
    apu->fc_reset_delay = 0x80u;
    apu->oam_dma_active = false;
    apu->oam_dma_halt = false;
    apu->dmc_load_delay = 0u;
    apu->dmc_abort_delay = 0u;
    apu->dmc_implicit_window = 0u;
    apu->dmc_just_reloaded = false;
    apu->dmc_dma_pending = false;
    apu->dmc_dma_abort_one = false;
    apu->dmc_dma_phase = APU_DMC_DMA_IDLE;
    apu->tri.step = 0u;
    apu->ext_input = 0.0f;
    ricoh_apu_write(apu, 0x4017u, apu->fc_last_write);
    update_irq_lines(apu);
}

void ricoh_apu_step(RicohApu *apu) {
    apu->put = !apu->put;

    /* Frame IRQ cleared when APU transitions PUT -> GET */
    if (!apu->put && apu->frame_irq_clear_pending) {
        apu->frame_irq = false;
        apu->frame_irq_clear_pending = false;
    }

    bool quarter = false;
    bool half = false;
    bool just_reset = false;

    if ((apu->fc_reset_delay & 0x80u) == 0u) {
        if (apu->fc_reset_delay > 0u) {
            apu->fc_reset_delay--;
        }
        if (apu->fc_reset_delay == 0u) {
            apu->fc_reset_delay = 0x80u;
            apu->fc_counter = 0u;
            apu->fc_five_step = apu->fc_next_five_step;
            just_reset = true;
            if (apu->fc_five_step) {
                quarter = true;
                half = true;
            }
        }
    }

    if (!apu->put) {
        if (!just_reset) {
            apu->fc_counter++;
            if (!apu->fc_five_step) {
                if (apu->fc_counter == 14914u) {
                    apu->frame_irq = true;
                } else if (apu->fc_counter >= 14915u) {
                    apu->fc_counter = 0u;
                    apu->frame_irq = !apu->fc_irq_inhibit;
                }
            } else if (apu->fc_counter >= 18641u) {
                apu->fc_counter = 0u;
            }
        }
    } else {
        if (apu->fc_counter == 3728u) {
            quarter = true;
        } else if (apu->fc_counter == 7456u) {
            quarter = true;
            half = true;
        } else if (apu->fc_counter == 11185u) {
            quarter = true;
        } else if (!apu->fc_five_step && apu->fc_counter == 14914u) {
            quarter = true;
            half = true;
            apu->frame_irq = true;
        } else if (apu->fc_five_step && apu->fc_counter == 18640u) {
            quarter = true;
            half = true;
        }
    }

    if (quarter) {
        clock_quarter_frame(apu);
    }
    if (half) {
        clock_half_frame(apu);
    }

    apu->pulse1.lc.halt = apu->pulse1.lc.next_halt;
    apu->pulse2.lc.halt = apu->pulse2.lc.next_halt;
    apu->tri.lc.halt    = apu->tri.lc.next_halt;
    apu->noise.lc.halt  = apu->noise.lc.next_halt;

    if (apu->dmc_abort_delay > 0u) {
        apu->dmc_abort_delay--;
    }
    if (apu->dmc_implicit_window > 0u) {
        apu->dmc_implicit_window--;
    }

    if (apu->dmc_load_delay > 0u) {
        apu->dmc_load_delay--;
        if (apu->dmc_load_delay == 0u) {
            if (apu->dmc.buffer_empty && apu->dmc.bytes_remaining > 0u &&
                apu->dmc_dma_phase == APU_DMC_DMA_IDLE) {
                apu->dmc_dma_pending = true;
            }
        }
    }

    if (apu->put) {
        if (apu->pulse1.timer == 0u) {
            apu->pulse1.timer = apu->pulse1.period;
            apu->pulse1.duty_step = (uint8_t)((apu->pulse1.duty_step + 1u) & 7u);
        } else {
            apu->pulse1.timer--;
        }

        if (apu->pulse2.timer == 0u) {
            apu->pulse2.timer = apu->pulse2.period;
            apu->pulse2.duty_step = (uint8_t)((apu->pulse2.duty_step + 1u) & 7u);
        } else {
            apu->pulse2.timer--;
        }
    }

    if (apu->tri.timer == 0u) {
        apu->tri.timer = apu->tri.period;
        if (apu->tri.lc.count > 0u && apu->tri.linear > 0u && apu->tri.period >= 2u) {
            apu->tri.step = (uint8_t)((apu->tri.step + 1u) & 31u);
        }
    } else {
        apu->tri.timer--;
    }

    if (apu->noise.timer == 0u) {
        apu->noise.timer = (uint16_t)(NOISE_PERIOD_NTSC[apu->noise.period_idx] - 1u);
        uint16_t tap = apu->noise.mode ? ((apu->noise.lfsr >> 6) & 1u)
                                       : ((apu->noise.lfsr >> 1) & 1u);
        uint16_t fb = (apu->noise.lfsr & 1u) ^ tap;
        apu->noise.lfsr = (uint16_t)((apu->noise.lfsr >> 1) | (fb << 14));
    } else {
        apu->noise.timer--;
    }

    apu->dmc_just_reloaded = false;
    if (!apu->put) {
        if (apu->dmc.timer <= 2u) {
            apu->dmc.timer = apu->dmc.period;
            apu->dmc_just_reloaded = true;
            clock_dmc_output(apu);
        } else {
            apu->dmc.timer -= 2u;
        }
    }

    update_irq_lines(apu);

    float raw = compute_raw_mixer(apu) + apu->ext_input;
    float hp = 0.999684f * (apu->hp_out + raw - apu->hp_prev_in);
    apu->hp_prev_in = raw;
    apu->hp_out = hp;
    apu->lp_out += 0.04684f * (hp - apu->lp_out);
}

uint8_t ricoh_apu_read_status(RicohApu *apu, uint8_t data_bus) {
    uint8_t status = (uint8_t)(data_bus & 0x20u);
    if (apu->pulse1.lc.count > 0u)      status |= 0x01u;
    if (apu->pulse2.lc.count > 0u)      status |= 0x02u;
    if (apu->tri.lc.count > 0u)         status |= 0x04u;
    if (apu->noise.lc.count > 0u)       status |= 0x08u;
    if (apu->dmc.bytes_remaining > 0u)  status |= 0x10u;
    if (apu->frame_irq)                 status |= 0x40u;
    if (apu->dmc_irq)                   status |= 0x80u;

    apu->frame_irq_clear_pending = true;
    return status;
}

static void write_pulse_reg(ApuPulse *p, uint8_t offset, uint8_t val) {
    switch (offset) {
        case 0:
            p->duty_mode        = (uint8_t)(val >> 6);
            p->lc.next_halt     = (val & 0x20u) != 0u;
            p->env.loop         = (val & 0x20u) != 0u;
            p->env.constant_vol = (val & 0x10u) != 0u;
            p->env.volume       = (uint8_t)(val & 0x0Fu);
            break;
        case 1:
            p->sweep_enabled = (val & 0x80u) != 0u;
            p->sweep_period  = (uint8_t)((val >> 4) & 0x07u);
            p->sweep_negate  = (val & 0x08u) != 0u;
            p->sweep_shift   = (uint8_t)(val & 0x07u);
            p->sweep_reload  = true;
            break;
        case 2:
            p->period = (uint16_t)((p->period & 0x0700u) | val);
            break;
        case 3:
            p->period = (uint16_t)((p->period & 0x00FFu) | ((uint16_t)(val & 0x07u) << 8));
            if (p->lc.enabled) {
                p->lc.count = LENGTH_TABLE[val >> 3];
            }
            p->duty_step = 0u;
            p->env.start = true;
            break;
    }
}

void ricoh_apu_write(RicohApu *apu, uint16_t addr, uint8_t val) {
    if (addr >= 0x4000u && addr <= 0x4003u) {
        write_pulse_reg(&apu->pulse1, (uint8_t)(addr - 0x4000u), val);
        return;
    }
    if (addr >= 0x4004u && addr <= 0x4007u) {
        write_pulse_reg(&apu->pulse2, (uint8_t)(addr - 0x4004u), val);
        return;
    }

    switch (addr) {
        case 0x4008u:
            apu->tri.linear_control    = (val & 0x80u) != 0u;
            apu->tri.lc.next_halt      = (val & 0x80u) != 0u;
            apu->tri.linear_reload_val = (uint8_t)(val & 0x7Fu);
            break;
        case 0x400Au:
            apu->tri.period = (uint16_t)((apu->tri.period & 0x0700u) | val);
            break;
        case 0x400Bu:
            apu->tri.period = (uint16_t)((apu->tri.period & 0x00FFu) | ((uint16_t)(val & 0x07u) << 8));
            if (apu->tri.lc.enabled) {
                apu->tri.lc.count = LENGTH_TABLE[val >> 3];
            }
            apu->tri.linear_reload_flag = true;
            break;

        case 0x400Cu:
            apu->noise.lc.next_halt     = (val & 0x20u) != 0u;
            apu->noise.env.loop         = (val & 0x20u) != 0u;
            apu->noise.env.constant_vol = (val & 0x10u) != 0u;
            apu->noise.env.volume       = (uint8_t)(val & 0x0Fu);
            break;
        case 0x400Eu:
            apu->noise.mode       = (val & 0x80u) != 0u;
            apu->noise.period_idx = (uint8_t)(val & 0x0Fu);
            break;
        case 0x400Fu:
            if (apu->noise.lc.enabled) {
                apu->noise.lc.count = LENGTH_TABLE[val >> 3];
            }
            apu->noise.env.start = true;
            break;

        case 0x4010u:
            apu->dmc.irq_enable = (val & 0x80u) != 0u;
            apu->dmc.loop       = (val & 0x40u) != 0u;
            apu->dmc.rate_idx   = (uint8_t)(val & 0x0Fu);
            apu->dmc.period     = DMC_PERIOD_NTSC[apu->dmc.rate_idx];
            if (apu->dmc_just_reloaded) {
                apu->dmc.timer = apu->dmc.period;
            }
            if (!apu->dmc.irq_enable) {
                apu->dmc_irq = false;
                update_irq_lines(apu);
            }
            break;
        case 0x4011u:
            apu->dmc.output_level = (uint8_t)(val & 0x7Fu);
            break;
        case 0x4012u:
            apu->dmc.sample_addr = (uint16_t)(0xC000u | ((uint16_t)val << 6));
            break;
        case 0x4013u:
            apu->dmc.sample_length = (uint16_t)(((uint16_t)val << 4) | 1u);
            break;

        case 0x4015u:
            apu->dmc_irq = false;
            apu->pulse1.lc.enabled = (val & 0x01u) != 0u;
            if (!apu->pulse1.lc.enabled) apu->pulse1.lc.count = 0u;

            apu->pulse2.lc.enabled = (val & 0x02u) != 0u;
            if (!apu->pulse2.lc.enabled) apu->pulse2.lc.count = 0u;

            apu->tri.lc.enabled    = (val & 0x04u) != 0u;
            if (!apu->tri.lc.enabled) apu->tri.lc.count = 0u;

            apu->noise.lc.enabled  = (val & 0x08u) != 0u;
            if (!apu->noise.lc.enabled) apu->noise.lc.count = 0u;

            apu->dmc.enabled = (val & 0x10u) != 0u;
            if (!apu->dmc.enabled) {
                if (!apu->dmc_dma_pending && apu->dmc.bytes_remaining > 0u &&
                    apu->dmc_dma_phase == APU_DMC_DMA_IDLE) {
                    apu->dmc_abort_delay = apu->put ? 2u : 3u;
                }
                apu->dmc.bytes_remaining = 0u;
                apu->dmc_load_delay = 0u;
            } else {
                apu->dmc_abort_delay = 0u;
                apu->dmc_implicit_window = 0u;
                bool was_zero = (apu->dmc.bytes_remaining == 0u);
                if (was_zero) {
                    apu->dmc.current_addr    = apu->dmc.sample_addr;
                    apu->dmc.bytes_remaining = apu->dmc.sample_length;
                }
                if (was_zero && apu->dmc_load_delay == 0u &&
                    !apu->dmc_dma_pending && apu->dmc_dma_phase == APU_DMC_DMA_IDLE) {
                    apu->dmc_load_delay = apu->put ? 3u : 4u;
                }
            }
            update_irq_lines(apu);
            break;

        case 0x4017u:
            apu->fc_last_write     = val;
            apu->fc_next_five_step = (val & 0x80u) != 0u;
            apu->fc_irq_inhibit    = (val & 0x40u) != 0u;
            if (apu->fc_irq_inhibit) {
                apu->frame_irq = false;
                apu->frame_irq_clear_pending = false;
                update_irq_lines(apu);
            }
            apu->fc_reset_delay = apu->put ? 3u : 4u;
            break;

        default:
            break;
    }
}

void ricoh_apu_trigger_oam_dma(RicohApu *apu, uint8_t page) {
    apu->oam_dma_page = page;
    if (!apu->oam_dma_active) {
        apu->oam_dma_active    = true;
        apu->oam_dma_halt      = true;
        apu->oam_dma_put_phase = false;
        apu->oam_dma_index     = 0u;
    }
}

void ricoh_apu_notify_cpu_write(RicohApu *apu) {
    apu->dmc_dma_abort_one = false;
}

bool ricoh_apu_dma_wants_halt(const RicohApu *apu) {
    if (apu->oam_dma_active) {
        return true;
    }
    if (apu->dmc_dma_phase != APU_DMC_DMA_IDLE) {
        return true;
    }
    if (apu->dmc_dma_pending) {
        return true;
    }
    if (apu->dmc_dma_abort_one && !apu->put) {
        return true;
    }
    if (apu->dmc_load_delay == 1u && apu->dmc.buffer_empty && apu->dmc.bytes_remaining > 0u) {
        return true;
    }
    return false;
}

void ricoh_apu_dmc_finish_dma(RicohApu *apu, uint8_t val) {
    ApuDmc *dmc = &apu->dmc;
    dmc->sample_buffer = val;
    dmc->buffer_empty  = false;
    dmc->current_addr  = (uint16_t)(((dmc->current_addr + 1u) & 0x7FFFu) | 0x8000u);

    if (dmc->bytes_remaining > 0u) {
        dmc->bytes_remaining--;
        if (dmc->bytes_remaining == 0u) {
            if (dmc->loop) {
                dmc->current_addr    = dmc->sample_addr;
                dmc->bytes_remaining = dmc->sample_length;
            } else {
                apu->dmc_implicit_window = 3u;
                if (dmc->irq_enable) {
                    apu->dmc_irq = true;
                    update_irq_lines(apu);
                }
            }
        }
    }
}

void ricoh_apu_set_ext_audio(RicohApu *apu, float ext_sample) {
    apu->ext_input = ext_sample;
}

float ricoh_apu_output(const RicohApu *apu) {
    return apu->lp_out;
}