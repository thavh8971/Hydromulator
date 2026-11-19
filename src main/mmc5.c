#include "mmc5.h"
#include <stddef.h>
#include <string.h>

static const uint8_t MMC5_LENGTH_TABLE[32] = {
    10, 254, 20,  2, 40,  4, 80,  6,
    160,  8, 60, 10, 14, 12, 26, 14,
    12,  16, 24, 18, 48, 20, 96, 22,
    192, 24, 72, 26, 16, 28, 32, 30
};

static const uint8_t MMC5_PULSE_DUTY[4][8] = {
    { 0, 1, 0, 0, 0, 0, 0, 0 },
    { 0, 1, 1, 0, 0, 0, 0, 0 },
    { 0, 1, 1, 1, 1, 0, 0, 0 },
    { 1, 0, 0, 1, 1, 1, 1, 1 }
};

/* ==================== IRQ & Audio Helpers ==================== */

static inline void mmc5_update_irq(const Mmc5 *m) {
    if (m->bus.on_irq == NULL) {
        return;
    }
    bool sl_irq  = m->scanline_irq_enable && m->scanline_irq_pending;
    bool pcm_irq = m->pcm_irq_enable && m->pcm_irq_pending;
    bool tmr_irq = m->is_mmc5a && m->timer_irq_pending;

    m->bus.on_irq(m->bus.ctx, MMC5_IRQ_SCANLINE, sl_irq);
    m->bus.on_irq(m->bus.ctx, MMC5_IRQ_PCM,      pcm_irq);
    m->bus.on_irq(m->bus.ctx, MMC5_IRQ_TIMER,    tmr_irq);
}

static void mmc5_clock_envelope(Mmc5Envelope *env) {
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

static inline uint8_t mmc5_envelope_output(const Mmc5Envelope *env) {
    return env->constant_vol ? env->volume : env->decay;
}

static void mmc5_write_pulse_reg(Mmc5Pulse *p, uint8_t offset, uint8_t val) {
    switch (offset) {
        case 0:
            p->duty_mode        = (uint8_t)(val >> 6);
            p->lc_next_halt     = (val & 0x20u) != 0u;
            p->env.loop         = (val & 0x20u) != 0u;
            p->env.constant_vol = (val & 0x10u) != 0u;
            p->env.volume       = (uint8_t)(val & 0x0Fu);
            break;
        case 1:
            /* Sweep unit is not present in MMC5 */
            break;
        case 2:
            p->period = (uint16_t)((p->period & 0x0700u) | val);
            break;
        case 3:
            p->period = (uint16_t)((p->period & 0x00FFu) | ((uint16_t)(val & 0x07u) << 8));
            if (p->lc_enabled) {
                p->lc_count = MMC5_LENGTH_TABLE[val >> 3];
            }
            p->duty_step = 0u;
            p->env.start = true;
            break;
        default:
            break;
    }
}

static void mmc5_pcm_trigger_sample(Mmc5 *m, uint8_t val) {
    if (val == 0x00u) {
        m->pcm_irq_pending = true;
        mmc5_update_irq(m);
    } else {
        m->pcm_dac = val;
    }
}

/* ==================== PRG Banking & PRG-RAM ==================== */

static inline bool mmc5_prg_ram_writable(const Mmc5 *m) {
    return ((m->prg_ram_protect1 & 0x03u) == 0x02u) &&
           ((m->prg_ram_protect2 & 0x03u) == 0x01u);
}

static bool mmc5_map_prg_ram(const Mmc5 *m, uint8_t reg_val, uint16_t addr, uint32_t *out_offset) {
    uint8_t bank = (uint8_t)(reg_val & 0x0Fu);
    uint32_t chip_offset = 0u;

    if (m->prg_ram_is_128k) {
        chip_offset = ((uint32_t)bank << 13) | (addr & 0x1FFFu);
        if (chip_offset >= MMC5_MAX_PRG_RAM_SIZE) {
            return false;
        }
        *out_offset = chip_offset;
        return true;
    }

    bool chip1 = (bank & 0x04u) != 0u;
    uint8_t sub_bank = (uint8_t)(bank & 0x03u);

    if (!chip1) {
        if (m->prg_ram_chip0_size == 0u) {
            return false;
        }
        chip_offset = (((uint32_t)sub_bank << 13) | (addr & 0x1FFFu)) % m->prg_ram_chip0_size;
        *out_offset = chip_offset;
        return true;
    } else {
        if (m->prg_ram_chip1_size == 0u) {
            return false;
        }
        chip_offset = (((uint32_t)sub_bank << 13) | (addr & 0x1FFFu)) % m->prg_ram_chip1_size;
        *out_offset = 0x10000u + chip_offset;
        return true;
    }
}

static bool mmc5_resolve_prg_slot(const Mmc5 *m, uint16_t addr,
                                  bool *is_ram, uint32_t *out_offset) {
    if (addr < 0x8000u) {
        *is_ram = true;
        return mmc5_map_prg_ram(m, m->prg_banks[0], addr, out_offset);
    }

    uint8_t raw_bank = 0u;
    uint32_t rom_mask_offset = addr & 0x1FFFu;
    bool force_rom = false;

    switch (m->prg_mode & 0x03u) {
        case 0: /* 32KB at $8000-$FFFF via $5117 (bits 6..2) */
            raw_bank = (uint8_t)(m->prg_banks[4] & 0x7Cu);
            rom_mask_offset = addr & 0x7FFFu;
            force_rom = true;
            break;

        case 1: /* 16KB at $8000-$BFFF via $5115, 16KB at $C000-$FFFF via $5117 */
            if (addr < 0xC000u) {
                raw_bank = (uint8_t)(m->prg_banks[2] & 0xFEu);
                if ((m->prg_banks[2] & 0x80u) == 0u) {
                    *is_ram = true;
                    uint8_t ram_bank = (uint8_t)((raw_bank & 0x0Eu) | ((addr >> 13) & 0x01u));
                    return mmc5_map_prg_ram(m, ram_bank, addr, out_offset);
                }
                rom_mask_offset = addr & 0x3FFFu;
            } else {
                raw_bank = (uint8_t)(m->prg_banks[4] & 0x7Eu);
                rom_mask_offset = addr & 0x3FFFu;
                force_rom = true;
            }
            break;

        case 2: /* 16KB at $8000-$BFFF via $5115, 8KB at $C000-$DFFF via $5116, 8KB at $E000-$FFFF via $5117 */
            if (addr < 0xC000u) {
                raw_bank = (uint8_t)(m->prg_banks[2] & 0xFEu);
                if ((m->prg_banks[2] & 0x80u) == 0u) {
                    *is_ram = true;
                    uint8_t ram_bank = (uint8_t)((raw_bank & 0x0Eu) | ((addr >> 13) & 0x01u));
                    return mmc5_map_prg_ram(m, ram_bank, addr, out_offset);
                }
                rom_mask_offset = addr & 0x3FFFu;
            } else if (addr < 0xE000u) {
                raw_bank = m->prg_banks[3];
                if ((raw_bank & 0x80u) == 0u) {
                    *is_ram = true;
                    return mmc5_map_prg_ram(m, raw_bank, addr, out_offset);
                }
            } else {
                raw_bank = m->prg_banks[4];
                force_rom = true;
            }
            break;

        case 3: /* 4 x 8KB via $5114, $5115, $5116, $5117 */
            if (addr < 0xA000u) {
                raw_bank = m->prg_banks[1];
                if ((raw_bank & 0x80u) == 0u) {
                    *is_ram = true;
                    return mmc5_map_prg_ram(m, raw_bank, addr, out_offset);
                }
            } else if (addr < 0xC000u) {
                raw_bank = m->prg_banks[2];
                if ((raw_bank & 0x80u) == 0u) {
                    *is_ram = true;
                    return mmc5_map_prg_ram(m, raw_bank, addr, out_offset);
                }
            } else if (addr < 0xE000u) {
                raw_bank = m->prg_banks[3];
                if ((raw_bank & 0x80u) == 0u) {
                    *is_ram = true;
                    return mmc5_map_prg_ram(m, raw_bank, addr, out_offset);
                }
            } else {
                raw_bank = m->prg_banks[4];
                force_rom = true;
            }
            break;
    }

    (void)force_rom;
    *is_ram = false;
    if (m->prg_size == 0u) {
        return false;
    }
    uint32_t bank8k = (uint32_t)(raw_bank & 0x7Fu);
    *out_offset = ((bank8k << 13) | rom_mask_offset) % m->prg_size;
    return true;
}

/* ==================== CHR Banking & PPU Mapping ==================== */

static void mmc5_write_chr_reg(Mmc5 *m, uint8_t idx, uint8_t val) {
    m->chr_last_written_b = (idx >= 8u);
    uint16_t reg10 = 0u;

    switch (m->chr_mode & 0x03u) {
        case 0: /* 8 KB mode */
            reg10 = (uint16_t)(((uint16_t)(val & 0x7Fu) << 3) | (val & 0x01u));
            break;
        case 1: /* 4 KB mode */
            reg10 = (uint16_t)(((uint16_t)val << 2) | (val & 0x01u));
            break;
        case 2: /* 2 KB mode */
            reg10 = (uint16_t)(((uint16_t)(m->chr_hi & 0x01u) << 9) |
                               ((uint16_t)val << 1) |
                               (val & 0x01u));
            break;
        case 3: /* 1 KB mode */
            reg10 = (uint16_t)(((uint16_t)(m->chr_hi & 0x03u) << 8) | val);
            break;
    }
    m->chr_regs[idx] = reg10;
}

static uint32_t mmc5_map_chr_normal(const Mmc5 *m, uint16_t addr, bool use_set_b) {
    uint8_t reg_idx = 0u;
    uint8_t mode = (uint8_t)(m->chr_mode & 0x03u);

    if (!use_set_b) {
        switch (mode) {
            case 0: reg_idx = 7u; break;
            case 1: reg_idx = (addr < 0x1000u) ? 3u : 7u; break;
            case 2: reg_idx = (uint8_t)(((addr >> 11) * 2u) + 1u); break;
            case 3: reg_idx = (uint8_t)(addr >> 10); break;
        }
    } else {
        switch (mode) {
            case 0: reg_idx = 11u; break;
            case 1: reg_idx = 11u; break;
            case 2: reg_idx = (addr & 0x0800u) ? 11u : 9u; break;
            case 3: reg_idx = (uint8_t)(8u + ((addr >> 10) & 0x03u)); break;
        }
    }

    uint16_t reg10 = m->chr_regs[reg_idx];
    uint32_t offset = ((uint32_t)(reg10 & 0x03F8u) << 10) | (addr & 0x03FFu);

    switch (mode) {
        case 0: offset |= (addr & 0x1C00u); break;
        case 1: offset |= ((uint32_t)(reg10 & 0x0004u) << 10) | (addr & 0x0C00u); break;
        case 2: offset |= ((uint32_t)(reg10 & 0x0006u) << 10) | (addr & 0x0400u); break;
        case 3: offset |= ((uint32_t)(reg10 & 0x0007u) << 10); break;
    }
    return offset % m->chr_size;
}

static inline void mmc5_inc_vsplit_y(Mmc5 *m) {
    if (m->vsplit_y == 239u) {
        m->vsplit_y = 0u;
    } else if (m->vsplit_y == 255u) {
        m->vsplit_y = 0u;
    } else {
        m->vsplit_y++;
    }
}

static void mmc5_decode_fetch(const Mmc5 *m, bool *out_is_sprite,
                              uint8_t *out_tile_x, uint8_t *out_phase) {
    uint8_t fc = m->fetch_count;
    if (fc >= 127u && fc <= 158u) {
        *out_is_sprite = true;
        *out_tile_x = 0u;
        *out_phase = 0u;
        return;
    }

    *out_is_sprite = false;
    if (fc <= 126u) {
        uint8_t slot = (uint8_t)(fc + 9u);
        *out_tile_x = (uint8_t)(slot >> 2);
        *out_phase  = (uint8_t)(slot & 0x03u);
    } else if (fc >= 159u && fc <= 166u) {
        uint8_t slot = (uint8_t)(fc - 159u);
        *out_tile_x = (uint8_t)(slot >> 2);
        *out_phase  = (uint8_t)(slot & 0x03u);
    } else {
        *out_tile_x = 2u;
        *out_phase  = 0u;
    }
}

/* ==================== Lifecycle ==================== */

static void mmc5_apply_reset_regs(Mmc5 *m) {
    m->pcm_read_mode  = false;
    m->pcm_irq_enable = false;
    m->pcm_irq_pending = false;

    m->prg_mode         = 3u;
    m->prg_ram_protect1 = 1u;
    m->prg_ram_protect2 = 2u;
    m->exram_mode       = 3u;
    m->exram_written_since_reset = false;
    m->prg_banks[4]     = 0xFFu;
    m->chr_hi           = 0u;

    m->vsplit_ctrl   = 0u;
    m->vsplit_scroll = 0u;
    m->vsplit_active_tile = false;

    m->scanline_irq_enable = false;
    m->abnormal_m2_a0      = false;

    m->ppu_scroll_second = false;
    m->ppu_scroll_fine_y = 0u;
    m->ppu_addr_second   = false;

    m->cl3_sl3_ctrl      = 0xC0u;
    m->cl3_sl3_out       = 0x00u;
    m->timer_counter     = 0u;
    m->timer_running     = false;
    m->timer_irq_pending = false;

    mmc5_update_irq(m);
}

void mmc5_power_on(Mmc5 *m, const Mmc5Config *cfg, const Mmc5Bus *bus) {
    memset(m, 0, sizeof(*m));
    if (bus != NULL) {
        m->bus = *bus;
    }
    if (cfg != NULL) {
        m->prg_rom    = cfg->prg_rom;
        m->prg_size   = cfg->prg_size;
        m->chr_mem    = cfg->chr_mem;
        m->chr_size   = cfg->chr_size;
        m->chr_is_ram = cfg->chr_is_ram;
        m->is_mmc5a   = cfg->is_mmc5a;
        m->sl_mode    = cfg->sl_mode;

        if (!cfg->is_nes2) {
            /* iNES 1.0 cannot distinguish ELROM/EKROM/ETROM/EWROM: use 64 KB compatible superset */
            m->prg_ram_chip0_size = 32768u;
            m->prg_ram_chip1_size = 32768u;
            m->prg_ram_is_128k    = false;
        } else {
            uint32_t total = cfg->prg_ram_size + cfg->prg_nvram_size;
            if (total >= 131072u) {
                m->prg_ram_is_128k    = true;
                m->prg_ram_chip0_size = MMC5_MAX_PRG_RAM_SIZE;
                m->prg_ram_chip1_size = 0u;
            } else if (cfg->prg_nvram_size > 0u && cfg->prg_ram_size > 0u) {
                /* Dual-chip boards such as ETROM (8K+8K) or 40K/64K */
                m->prg_ram_chip0_size = cfg->prg_nvram_size;
                m->prg_ram_chip1_size = cfg->prg_ram_size;
            } else if (total > 32768u) {
                /* 64K split evenly across both 32K chip slots */
                m->prg_ram_chip0_size = 32768u;
                m->prg_ram_chip1_size = total - 32768u;
            } else {
                m->prg_ram_chip0_size = total;
                m->prg_ram_chip1_size = 0u;
            }
        }
    }

    m->mul_a = 0xFFu;
    m->mul_b = 0xFFu;
    mmc5_apply_reset_regs(m);
}

void mmc5_reset(Mmc5 *m) {
    /* Soft reset clears pulse enables, IRQ enables, protection & reset-sensitive registers */
    mmc5_cpu_write(m, 0x5015u, 0x00u);
    m->in_frame = false;
    m->ppu_match_count = 0u;
    m->ppu_idle_count = 3u;
    mmc5_apply_reset_regs(m);
}

/* ==================== CPU Cycle Tick (M2 Rise) ==================== */

void mmc5_cpu_cycle_start(Mmc5 *m) {
    /* 1. PPU /RD idle timeout: 3 M2 rising edges without PPU read clears in-frame */
    if (m->ppu_idle_count < 3u) {
        m->ppu_idle_count++;
        if (m->ppu_idle_count >= 3u) {
            m->in_frame = false;
        }
    }

    /* 2. MMC5A 16-bit M2 countdown timer */
    if (m->is_mmc5a && m->timer_running) {
        if (m->timer_counter > 0u) {
            m->timer_counter--;
            if (m->timer_counter == 0u) {
                m->timer_running = false;
                m->timer_irq_pending = true;
                mmc5_update_irq(m);
            }
        } else {
            m->timer_running = false;
        }
    }

    /* 3. MMC5 expansion audio: pulse timers at M2/2, envelope & length at M2/7424 (~241.079 Hz) */
    m->pulse_half_clk = !m->pulse_half_clk;
    if (m->pulse_half_clk) {
        if (m->pulse1.timer == 0u) {
            m->pulse1.timer = m->pulse1.period;
            m->pulse1.duty_step = (uint8_t)((m->pulse1.duty_step + 1u) & 7u);
        } else {
            m->pulse1.timer--;
        }

        if (m->pulse2.timer == 0u) {
            m->pulse2.timer = m->pulse2.period;
            m->pulse2.duty_step = (uint8_t)((m->pulse2.duty_step + 1u) & 7u);
        } else {
            m->pulse2.timer--;
        }
    }

    m->pulse1.lc_halt = m->pulse1.lc_next_halt;
    m->pulse2.lc_halt = m->pulse2.lc_next_halt;

    m->audio_frame_div++;
    if (m->audio_frame_div >= 7424u) {
        m->audio_frame_div = 0u;
        mmc5_clock_envelope(&m->pulse1.env);
        mmc5_clock_envelope(&m->pulse2.env);
        if (m->pulse1.lc_count > 0u && !m->pulse1.lc_halt) {
            m->pulse1.lc_count--;
        }
        if (m->pulse2.lc_count > 0u && !m->pulse2.lc_halt) {
            m->pulse2.lc_count--;
        }
    }
}

/* ==================== CPU Reads & Writes ==================== */

uint8_t mmc5_cpu_read(Mmc5 *m, uint16_t addr, uint8_t open_bus, bool *driven_out) {
    if (driven_out != NULL) {
        *driven_out = true;
    }

    if (addr >= 0x6000u) {
        /* Snooping NMI vector fetch resets scanline counter, in-frame, and IRQ pending */
        if (addr == 0xFFFAu || addr == 0xFFFBu) {
            m->in_frame = false;
            m->scanline_counter = 0u;
            m->scanline_irq_pending = false;
            mmc5_update_irq(m);
        }

        bool is_ram = false;
        uint32_t offset = 0u;
        if (!mmc5_resolve_prg_slot(m, addr, &is_ram, &offset)) {
            if (driven_out != NULL) {
                *driven_out = false;
            }
            return open_bus;
        }

        uint8_t val = is_ram ? m->prg_ram[offset] : m->prg_rom[offset];

        /* Raw PCM Read Mode snoops reads from $8000-$BFFF */
        if (m->pcm_read_mode && addr >= 0x8000u && addr <= 0xBFFFu) {
            mmc5_pcm_trigger_sample(m, val);
        }
        return val;
    }

    if (addr >= 0x5C00u) {
        if (m->exram_mode >= 2u) {
            return m->exram[addr & 0x03FFu];
        }
        if (driven_out != NULL) {
            *driven_out = false;
        }
        return open_bus;
    }

    if (addr >= 0x5800u) {
        if (m->is_mmc5a) {
            return m->ext_5800[addr & 0x03FFu];
        }
        if (driven_out != NULL) {
            *driven_out = false;
        }
        return open_bus;
    }

    switch (addr) {
        case 0x5010u: {
            uint8_t val = (uint8_t)((open_bus & 0x7Eu) |
                                    ((m->pcm_irq_pending && m->pcm_irq_enable) ? 0x80u : 0x00u) |
                                    (m->is_mmc5a ? 0x01u : 0x00u));
            m->pcm_irq_pending = false;
            mmc5_update_irq(m);
            return val;
        }

        case 0x5015u: {
            uint8_t val = (uint8_t)(open_bus & 0xFCu);
            if (m->pulse1.lc_count > 0u) val |= 0x01u;
            if (m->pulse2.lc_count > 0u) val |= 0x02u;
            return val;
        }

        case 0x5204u: {
            uint8_t val = (uint8_t)((m->scanline_irq_pending ? 0x80u : 0x00u) |
                                    (m->in_frame ? 0x40u : 0x00u) |
                                    (open_bus & 0x3Eu) |
                                    (m->abnormal_m2_a0 ? 0x01u : 0x00u));
            m->scanline_irq_pending = false;
            mmc5_update_irq(m);
            return val;
        }

        case 0x5205u:
            return (uint8_t)(((uint16_t)m->mul_a * (uint16_t)m->mul_b) & 0xFFu);

        case 0x5206u:
            return (uint8_t)(((uint16_t)m->mul_a * (uint16_t)m->mul_b) >> 8);

        case 0x5208u:
            if (m->is_mmc5a) {
                return (uint8_t)((m->cl3_sl3_out & 0xC0u) | (open_bus & 0x3Fu));
            }
            break;

        case 0x5209u:
            if (m->is_mmc5a) {
                uint8_t val = (uint8_t)((m->timer_irq_pending ? 0x80u : 0x00u) | (open_bus & 0x7Fu));
                m->timer_irq_pending = false;
                mmc5_update_irq(m);
                return val;
            }
            break;

        default:
            break;
    }

    if (driven_out != NULL) {
        *driven_out = false;
    }
    return open_bus;
}

void mmc5_cpu_write(Mmc5 *m, uint16_t addr, uint8_t val) {
    if (addr >= 0x6000u) {
        if (!mmc5_prg_ram_writable(m)) {
            return;
        }
        bool is_ram = false;
        uint32_t offset = 0u;
        if (mmc5_resolve_prg_slot(m, addr, &is_ram, &offset) && is_ram) {
            m->prg_ram[offset] = val;
        }
        return;
    }

    if (addr >= 0x5C00u) {
        if (m->exram_mode <= 1u) {
            m->exram_written_since_reset = true;
            m->exram[addr & 0x03FFu] = m->in_frame ? val : 0x00u;
        } else if (m->exram_mode == 2u) {
            m->exram_written_since_reset = true;
            m->exram[addr & 0x03FFu] = val;
        }
        return;
    }

    if (addr >= 0x5800u) {
        if (m->is_mmc5a) {
            m->ext_5800[addr & 0x03FFu] = val;
        }
        return;
    }

    if (addr >= 0x5000u && addr <= 0x5003u) {
        mmc5_write_pulse_reg(&m->pulse1, (uint8_t)(addr - 0x5000u), val);
        return;
    }
    if (addr >= 0x5004u && addr <= 0x5007u) {
        mmc5_write_pulse_reg(&m->pulse2, (uint8_t)(addr - 0x5004u), val);
        return;
    }

    if (addr >= 0x5120u && addr <= 0x512Bu) {
        mmc5_write_chr_reg(m, (uint8_t)(addr - 0x5120u), val);
        return;
    }

    switch (addr) {
        case 0x5010u:
            m->pcm_irq_enable = (val & 0x80u) != 0u;
            m->pcm_read_mode  = (val & 0x01u) != 0u;
            mmc5_update_irq(m);
            break;

        case 0x5011u:
            if (!m->pcm_read_mode) {
                mmc5_pcm_trigger_sample(m, val);
            }
            break;

        case 0x5015u:
            m->pulse1.lc_enabled = (val & 0x01u) != 0u;
            if (!m->pulse1.lc_enabled) m->pulse1.lc_count = 0u;
            m->pulse2.lc_enabled = (val & 0x02u) != 0u;
            if (!m->pulse2.lc_enabled) m->pulse2.lc_count = 0u;
            break;

        case 0x5100u: m->prg_mode = (uint8_t)(val & 0x03u); break;
        case 0x5101u: m->chr_mode = (uint8_t)(val & 0x03u); break;
        case 0x5102u: m->prg_ram_protect1 = (uint8_t)(val & 0x03u); break;
        case 0x5103u: m->prg_ram_protect2 = (uint8_t)(val & 0x03u); break;
        case 0x5104u: m->exram_mode = (uint8_t)(val & 0x03u); break;
        case 0x5105u: m->nt_mapping = val; break;
        case 0x5106u: m->fill_tile = val; break;
        case 0x5107u: m->fill_color = (uint8_t)(val & 0x03u); break;

        case 0x5113u: m->prg_banks[0] = (uint8_t)(val & 0x0Fu); break;
        case 0x5114u: m->prg_banks[1] = val; break;
        case 0x5115u: m->prg_banks[2] = val; break;
        case 0x5116u: m->prg_banks[3] = val; break;
        case 0x5117u: m->prg_banks[4] = (uint8_t)(val | 0x80u); break;

        case 0x5130u: m->chr_hi = (uint8_t)(val & 0x03u); break;

        case 0x5200u: m->vsplit_ctrl = val; break;
        case 0x5201u: m->vsplit_scroll = val; break;
        case 0x5202u: m->vsplit_chr_bank = val; break;

        case 0x5203u: m->irq_scanline = val; break;
        case 0x5204u:
            m->scanline_irq_enable = (val & 0x80u) != 0u;
            mmc5_update_irq(m);
            break;

        case 0x5205u: m->mul_a = val; break;
        case 0x5206u: m->mul_b = val; break;

        case 0x5207u:
            if (m->is_mmc5a) {
                m->cl3_sl3_ctrl = (uint8_t)(val & 0xC3u);
            }
            break;

        case 0x5208u:
            if (m->is_mmc5a) {
                m->cl3_sl3_out = (uint8_t)(val & 0xC0u);
            }
            break;

        case 0x5209u:
            if (m->is_mmc5a) {
                m->timer_counter = (uint16_t)((m->timer_counter & 0xFF00u) | val);
                if (m->timer_counter > 0u) {
                    m->timer_running = true;
                }
            }
            break;

        case 0x520Au:
            if (m->is_mmc5a) {
                m->timer_counter = (uint16_t)((m->timer_counter & 0x00FFu) | ((uint16_t)val << 8));
            }
            break;

        default:
            break;
    }
}

/* ==================== Hardware Snooping ($2000-$2007, $4014) ==================== */

void mmc5_snoop_cpu_read(Mmc5 *m, uint16_t addr, uint8_t val) {
    /* Fully decoded $2002 (no mirrors) */
    if (addr == 0x2002u) {
        m->ppu_scroll_second = false;
        m->ppu_addr_second   = false;
        if (val & 0x80u) {
            m->in_frame = false;
            m->scanline_counter = 0u;
            m->scanline_irq_pending = false;
            mmc5_update_irq(m);
        }
    }
}

void mmc5_snoop_cpu_write(Mmc5 *m, uint16_t addr, uint8_t val) {
    /* Fully decoded addresses (no mirrors) */
    switch (addr) {
        case 0x2000u:
            m->sprite_8x16 = (val & 0x20u) != 0u;
            break;

        case 0x2001u: {
            bool prev_enabled = m->ppu_subst_enabled;
            bool next_enabled = (val & 0x18u) != 0u;
            m->ppu_subst_enabled = next_enabled;
            if (!next_enabled) {
                m->in_frame = false;
            } else if (!prev_enabled && next_enabled) {
                m->in_frame = false;
                m->scanline_counter = 0u;
                m->scanline_irq_pending = false;
                mmc5_update_irq(m);
            }
            break;
        }

        case 0x2005u:
            if (!m->ppu_scroll_second) {
                m->ppu_scroll_second = true;
            } else {
                m->ppu_scroll_fine_y = (uint8_t)(val & 0x07u);
                m->ppu_scroll_second = false;
            }
            break;

        case 0x2006u:
            m->ppu_addr_second = !m->ppu_addr_second;
            break;

        case 0x4014u:
            m->in_frame = false;
            m->scanline_counter = 0u;
            m->scanline_irq_pending = false;
            mmc5_update_irq(m);
            break;

        default:
            break;
    }
}

/* ==================== PPU Bus Reads & Writes ==================== */

uint8_t mmc5_ppu_read(Mmc5 *m, uint16_t addr, bool count_rd_edge, uint8_t *ciram) {
    addr &= 0x3FFFu;

    if (count_rd_edge) {
        m->ppu_idle_count = 0u;

        bool scanline_detected = (m->ppu_match_count == 2u);
        if (addr >= 0x2000u && addr <= 0x2FFFu && addr == m->ppu_last_addr) {
            if (m->ppu_match_count < 2u) {
                m->ppu_match_count++;
            }
        } else {
            m->ppu_match_count = 0u;
        }
        m->ppu_last_addr = addr;

        if (scanline_detected) {
            if (!m->in_frame) {
                m->in_frame = true;
                m->scanline_counter = 0u;
                m->scanline_irq_pending = false;
                m->vsplit_y = m->vsplit_scroll;
            } else {
                m->scanline_counter++;
                if (m->scanline_counter >= 240u) {
                    /* 241st scanline detected: end of frame, clear counter & pending IRQ */
                    m->in_frame = false;
                    m->scanline_counter = 0u;
                    m->scanline_irq_pending = false;
                    m->vsplit_y = m->vsplit_scroll;
                } else if (m->irq_scanline != 0u && m->scanline_counter == m->irq_scanline) {
                    m->scanline_irq_pending = true;
                }
            }
            m->fetch_count = 0u;
            mmc5_update_irq(m);
        } else if (m->in_frame) {
            if (m->fetch_count < 255u) {
                m->fetch_count++;
            }
            if (m->fetch_count == 159u) {
                /* Start of Tile 0 prefetch for the next scanline */
                mmc5_inc_vsplit_y(m);
            }
        }
    }

    bool subst = m->in_frame && m->ppu_subst_enabled;
    bool is_sprite = false;
    uint8_t tile_x = 0u;
    uint8_t phase  = 0u;
    if (m->in_frame) {
        mmc5_decode_fetch(m, &is_sprite, &tile_x, &phase);
    }

    /* Nametable & Attribute fetches ($2000-$3FFF) */
    if (addr >= 0x2000u) {
        addr &= 0x2FFFu;

        if (subst && !is_sprite) {
            bool split_enabled = ((m->vsplit_ctrl & 0x80u) != 0u) && (m->exram_mode <= 1u);
            if (split_enabled && (phase == 0u || phase == 1u)) {
                uint8_t thresh = (uint8_t)(m->vsplit_ctrl & 0x1Fu);
                bool right_side = (m->vsplit_ctrl & 0x40u) != 0u;
                m->vsplit_active_tile = right_side ? (tile_x >= thresh) : (tile_x < thresh);
            } else if (!split_enabled) {
                m->vsplit_active_tile = false;
            }

            if (split_enabled && m->vsplit_active_tile) {
                if (phase == 0u) {
                    uint16_t nt_idx = (uint16_t)((((uint16_t)(m->vsplit_y >> 3) & 0x1Fu) << 5) |
                                                 (tile_x & 0x1Fu));
                    return m->exram[nt_idx];
                } else if (phase == 1u) {
                    uint16_t at_idx = (uint16_t)(0x03C0u |
                                                 (((uint16_t)(m->vsplit_y >> 5) & 0x07u) << 3) |
                                                 ((tile_x >> 2) & 0x07u));
                    uint8_t shift = (uint8_t)(((m->vsplit_y >> 2) & 0x04u) | (tile_x & 0x02u));
                    uint8_t pal2 = (uint8_t)((m->exram[at_idx] >> shift) & 0x03u);
                    return (uint8_t)(pal2 * 0x55u);
                }
            }

            if (m->exram_mode == 1u) {
                if (phase == 0u) {
                    m->exattr_latch = m->exram[addr & 0x03FFu];
                } else if (phase == 1u) {
                    uint8_t pal2 = (uint8_t)((m->exattr_latch >> 6) & 0x03u);
                    return (uint8_t)(pal2 * 0x55u);
                }
            }
        }

        uint8_t nt_idx = (uint8_t)((addr >> 10) & 0x03u);
        uint8_t src = (uint8_t)((m->nt_mapping >> (nt_idx * 2u)) & 0x03u);
        uint16_t off = (uint16_t)(addr & 0x03FFu);

        switch (src) {
            case 0: return ciram[off];
            case 1: return ciram[0x0400u | off];
            case 2: return (m->exram_mode <= 1u) ? m->exram[off] : 0x00u;
            case 3: return (off < 0x03C0u) ? m->fill_tile : (uint8_t)(m->fill_color * 0x55u);
            default: return 0x00u;
        }
    }

    /* Pattern Table / CHR fetches ($0000-$1FFF) */
    if (m->chr_size == 0u) {
        return 0xFFu;
    }

    if (subst && !is_sprite) {
        if (((m->vsplit_ctrl & 0x80u) != 0u) && (m->exram_mode <= 1u) && m->vsplit_active_tile) {
            uint16_t chr_lo = m->sl_mode
                ? (uint16_t)((addr & 0x0FF8u) | (m->vsplit_y & 0x0007u))
                : (uint16_t)(addr & 0x0FFFu);
            uint32_t offset = (((uint32_t)m->vsplit_chr_bank << 12) | chr_lo) % m->chr_size;
            return m->chr_mem[offset];
        }

        if (m->exram_mode == 1u) {
            uint32_t bank4k = ((uint32_t)(m->chr_hi & 0x03u) << 6) | (m->exattr_latch & 0x3Fu);
            uint32_t offset = ((bank4k << 12) | (addr & 0x0FFFu)) % m->chr_size;
            return m->chr_mem[offset];
        }
    }

    bool dual_chr = m->sprite_8x16 && m->ppu_subst_enabled;
    bool use_set_b = false;
    if (dual_chr) {
        if (m->in_frame) {
            use_set_b = !is_sprite;
        } else {
            use_set_b = m->chr_last_written_b;
        }
    }

    return m->chr_mem[mmc5_map_chr_normal(m, addr, use_set_b)];
}

void mmc5_ppu_write(Mmc5 *m, uint16_t addr, uint8_t val, uint8_t *ciram) {
    addr &= 0x3FFFu;

    if (addr < 0x2000u) {
        if (m->chr_is_ram && m->chr_size > 0u) {
            bool dual_chr = m->sprite_8x16 && m->ppu_subst_enabled;
            bool use_set_b = dual_chr ? m->chr_last_written_b : false;
            m->chr_mem[mmc5_map_chr_normal(m, addr, use_set_b)] = val;
        }
        return;
    }

    addr &= 0x2FFFu;
    uint8_t nt_idx = (uint8_t)((addr >> 10) & 0x03u);
    uint8_t src = (uint8_t)((m->nt_mapping >> (nt_idx * 2u)) & 0x03u);
    uint16_t off = (uint16_t)(addr & 0x03FFu);

    switch (src) {
        case 0: ciram[off] = val; break;
        case 1: ciram[0x0400u | off] = val; break;
        case 2:
            if (m->exram_mode <= 1u) {
                m->exram[off] = val;
            }
            break;
        default:
            break;
    }
}

/* ==================== Expansion Audio Output ==================== */

float mmc5_audio_output(const Mmc5 *m) {
    uint8_t p1 = 0u;
    if (m->pulse1.lc_count > 0u &&
        MMC5_PULSE_DUTY[m->pulse1.duty_mode][m->pulse1.duty_step]) {
        p1 = mmc5_envelope_output(&m->pulse1.env);
    }

    uint8_t p2 = 0u;
    if (m->pulse2.lc_count > 0u &&
        MMC5_PULSE_DUTY[m->pulse2.duty_mode][m->pulse2.duty_step]) {
        p2 = mmc5_envelope_output(&m->pulse2.env);
    }

    float pulse_out = 0.0f;
    if ((p1 + p2) > 0u) {
        pulse_out = 95.88f / ((8128.0f / (float)(p1 + p2)) + 100.0f);
    }

    /* PCM 0..127 matches 2A03 $4011 0..127 (~0.57425), 0..255 is 2x louder, polarity inverted */
    float pcm_out = ((float)m->pcm_dac) * (0.57425f / 127.0f);

    return -(pulse_out + pcm_out);
}