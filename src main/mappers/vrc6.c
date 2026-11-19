#include "vrc6.h"
#include <stddef.h>
#include <string.h>

/* ==================== IRQ Helpers ==================== */

static inline void vrc6_update_irq(const Vrc6 *m) {
    if (m->bus.on_irq != NULL) {
        m->bus.on_irq(m->bus.ctx, VRC6_IRQ_MASK, m->irq_pending);
    }
}

static void vrc6_clock_irq_counter(Vrc6 *m) {
    if (m->irq_counter == 0xFFu) {
        m->irq_counter = m->irq_latch;
        m->irq_pending = true;
        vrc6_update_irq(m);
    } else {
        m->irq_counter++;
    }
}

/* ==================== Audio Helpers ==================== */

static inline uint16_t vrc6_effective_period(uint16_t period, uint8_t freq_ctrl) {
    if (freq_ctrl & 0x04u) {
        return (uint16_t)(period >> 8);
    }
    if (freq_ctrl & 0x02u) {
        return (uint16_t)(period >> 4);
    }
    return period;
}

static void vrc6_step_pulse(Vrc6Pulse *p, uint8_t freq_ctrl) {
    if (!p->enabled) {
        p->duty_step = 15u;
        return;
    }
    if (p->timer == 0u) {
        p->timer = vrc6_effective_period(p->period, freq_ctrl);
        p->duty_step = (uint8_t)((p->duty_step - 1u) & 0x0Fu);
    } else {
        p->timer--;
    }
}

static void vrc6_step_saw(Vrc6Saw *s, uint8_t freq_ctrl) {
    if (s->timer == 0u) {
        s->timer = vrc6_effective_period(s->period, freq_ctrl);
        if (s->enabled) {
            s->step++;
            if (s->step >= 14u) {
                s->step = 0u;
                s->accum = 0u;
            } else if ((s->step & 0x01u) == 0u) {
                s->accum = (uint8_t)(s->accum + s->accum_rate);
            }
        }
    } else {
        s->timer--;
    }
}

static inline uint8_t vrc6_pulse_output(const Vrc6Pulse *p) {
    if (!p->enabled) {
        return 0u;
    }
    if (p->mode_ignore || p->duty_step <= p->duty) {
        return p->volume;
    }
    return 0u;
}

/* ==================== PPU CHR / Nametable Mapping ==================== */

static uint8_t vrc6_chr_bank_1k(const Vrc6 *m, uint16_t addr) {
    uint8_t slot = (uint8_t)((addr >> 10) & 0x07u);
    uint8_t a10  = (uint8_t)(slot & 0x01u);
    uint8_t mode = (uint8_t)(m->b003 & 0x03u);
    bool a10_passthru = (m->b003 & 0x20u) != 0u;

    if (mode == 0u) {
        return m->r[slot];
    }
    if (mode == 1u) {
        uint8_t reg = m->r[slot >> 1];
        return a10_passthru ? (uint8_t)((reg & 0xFEu) | a10) : reg;
    }

    /* Mode 2 or 3: $0000-$0FFF uses R0..R3 (1K), $1000-$1FFF uses R4..R5 (2K) */
    if (slot < 4u) {
        return m->r[slot];
    }
    uint8_t reg = m->r[4u + ((slot - 4u) >> 1)];
    return a10_passthru ? (uint8_t)((reg & 0xFEu) | a10) : reg;
}

static uint8_t vrc6_nt_bank_1k(const Vrc6 *m, uint16_t addr) {
    uint8_t nt = (uint8_t)((addr >> 10) & 0x03u);
    uint8_t reg = 0u;

    switch (m->b003 & 0x07u) {
        case 0:
        case 6:
        case 7:
            reg = (nt < 2u) ? m->r[6] : m->r[7];
            break;
        case 1:
        case 5:
            reg = m->r[4u + nt];
            break;
        case 2:
        case 3:
        case 4:
            reg = (nt & 0x01u) ? m->r[7] : m->r[6];
            break;
    }

    uint8_t mode = (uint8_t)(m->b003 & 0x03u);
    if ((m->b003 & 0x20u) != 0u && (mode == 0u || mode == 3u)) {
        uint8_t mir = (uint8_t)((m->b003 >> 2) & 0x03u);
        uint8_t lsb = 0u;
        if (mode == 0u) {
            switch (mir) {
                case 0: lsb = (uint8_t)(nt & 0x01u); break;        /* Vertical: PPU A10 */
                case 1: lsb = (uint8_t)((nt >> 1) & 0x01u); break; /* Horizontal: PPU A11 */
                case 2: lsb = 0u; break;                           /* 1-screen A */
                case 3: lsb = 1u; break;                           /* 1-screen B */
            }
        } else {
            switch (mir) {
                case 0: lsb = (uint8_t)((nt >> 1) & 0x01u); break; /* PPU A11 */
                case 1: lsb = (uint8_t)(nt & 0x01u); break;        /* PPU A10 */
                case 2: lsb = 1u; break;                           /* Odd only */
                case 3: lsb = 0u; break;                           /* Even only */
            }
        }
        reg = (uint8_t)((reg & 0xFEu) | lsb);
    }

    return reg;
}

/* ==================== Lifecycle ==================== */

void vrc6_power_on(Vrc6 *m, const Vrc6Config *cfg, const Vrc6Bus *bus) {
    memset(m, 0, sizeof(*m));
    if (bus != NULL) {
        m->bus = *bus;
    }
    if (cfg != NULL) {
        m->prg_rom         = cfg->prg_rom;
        m->prg_size        = cfg->prg_size;
        m->chr_mem         = cfg->chr_mem;
        m->chr_size        = cfg->chr_size;
        m->chr_is_ram      = cfg->chr_is_ram;
        m->prg_ram         = cfg->prg_ram;
        m->prg_ram_present = cfg->prg_ram_present;
        m->is_vrc6b        = cfg->is_vrc6b;
    }
    vrc6_reset(m);
}

void vrc6_reset(Vrc6 *m) {
    m->prg_16k        = 0u;
    m->prg_8k         = 0u;
    m->b003           = 0u;
    memset(m->r, 0, sizeof(m->r));

    m->irq_latch      = 0u;
    m->irq_counter    = 0u;
    m->irq_prescaler  = 341;
    m->irq_mode_cycle = false;
    m->irq_enable     = false;
    m->irq_ack_enable = false;
    m->irq_pending    = false;

    m->freq_ctrl      = 0u;
    memset(&m->pulse1, 0, sizeof(m->pulse1));
    memset(&m->pulse2, 0, sizeof(m->pulse2));
    memset(&m->saw, 0, sizeof(m->saw));
    m->pulse1.duty_step = 15u;
    m->pulse2.duty_step = 15u;

    vrc6_update_irq(m);
}

/* ==================== CPU Cycle Tick ==================== */

void vrc6_cpu_cycle(Vrc6 *m) {
    /* 1. VRC IRQ */
    if (m->irq_enable) {
        if (m->irq_mode_cycle) {
            vrc6_clock_irq_counter(m);
        } else {
            m->irq_prescaler -= 3;
            if (m->irq_prescaler <= 0) {
                m->irq_prescaler += 341;
                vrc6_clock_irq_counter(m);
            }
        }
    }

    /* 2. VRC6 Audio */
    if ((m->freq_ctrl & 0x01u) == 0u) {
        vrc6_step_pulse(&m->pulse1, m->freq_ctrl);
        vrc6_step_pulse(&m->pulse2, m->freq_ctrl);
        vrc6_step_saw(&m->saw, m->freq_ctrl);
    }
}

/* ==================== CPU Reads & Writes ==================== */

uint8_t vrc6_cpu_read(const Vrc6 *m, uint16_t addr, uint8_t open_bus, bool *driven_out) {
    if (addr >= 0x6000u && addr < 0x8000u) {
        if (!m->prg_ram_present || m->prg_ram == NULL || (m->b003 & 0x80u) == 0u) {
            if (driven_out) *driven_out = false;
            return open_bus;
        }
        if (driven_out) *driven_out = true;
        return m->prg_ram[addr - 0x6000u];
    }

    if (addr < 0x8000u) {
        if (driven_out) *driven_out = false;
        return open_bus;
    }

    if (driven_out) *driven_out = true;
    uint32_t banks_8k = m->prg_size / 8192u;
    if (banks_8k == 0u) {
        return m->prg_rom[(addr - 0x8000u) % m->prg_size];
    }

    uint32_t bank8k = 0u;
    if (addr < 0xC000u) {
        uint32_t banks_16k = m->prg_size / 16384u;
        if (banks_16k == 0u) {
            return m->prg_rom[(addr - 0x8000u) % m->prg_size];
        }
        uint32_t bank16k = (uint32_t)(m->prg_16k & 0x0Fu) % banks_16k;
        return m->prg_rom[(bank16k * 16384u) | (addr & 0x3FFFu)];
    } else if (addr < 0xE000u) {
        bank8k = (uint32_t)(m->prg_8k & 0x1Fu) % banks_8k;
    } else {
        bank8k = banks_8k - 1u;
    }

    return m->prg_rom[(bank8k * 8192u) | (addr & 0x1FFFu)];
}

static void vrc6_write_pulse_reg(Vrc6Pulse *p, uint8_t offset, uint8_t val) {
    switch (offset) {
        case 0:
            p->volume      = (uint8_t)(val & 0x0Fu);
            p->duty        = (uint8_t)((val >> 4) & 0x07u);
            p->mode_ignore = (val & 0x80u) != 0u;
            break;
        case 1:
            p->period = (uint16_t)((p->period & 0x0F00u) | val);
            break;
        case 2:
            p->period  = (uint16_t)((p->period & 0x00FFu) | ((uint16_t)(val & 0x0Fu) << 8));
            p->enabled = (val & 0x80u) != 0u;
            if (!p->enabled) {
                p->duty_step = 15u;
            }
            break;
        default:
            break;
    }
}

void vrc6_cpu_write(Vrc6 *m, uint16_t addr, uint8_t val) {
    if (addr >= 0x6000u && addr < 0x8000u) {
        if (m->prg_ram_present && m->prg_ram != NULL && (m->b003 & 0x80u) != 0u) {
            m->prg_ram[addr - 0x6000u] = val;
        }
        return;
    }

    if (addr < 0x8000u) {
        return;
    }

    uint16_t reg = (uint16_t)(addr & 0xF003u);
    if (m->is_vrc6b) {
        /* Swap A0 and A1 */
        reg = (uint16_t)((reg & 0xF000u) | ((reg & 0x01u) << 1) | ((reg >> 1) & 0x01u));
    }

    switch (reg) {
        case 0x8000u: case 0x8001u: case 0x8002u: case 0x8003u:
            m->prg_16k = (uint8_t)(val & 0x0Fu);
            break;

        case 0x9000u: case 0x9001u: case 0x9002u:
            vrc6_write_pulse_reg(&m->pulse1, (uint8_t)(reg & 0x03u), val);
            break;

        case 0x9003u:
            m->freq_ctrl = val;
            break;

        case 0xA000u: case 0xA001u: case 0xA002u:
            vrc6_write_pulse_reg(&m->pulse2, (uint8_t)(reg & 0x03u), val);
            break;

        case 0xB000u:
            m->saw.accum_rate = (uint8_t)(val & 0x3Fu);
            break;

        case 0xB001u:
            m->saw.period = (uint16_t)((m->saw.period & 0x0F00u) | val);
            break;

        case 0xB002u:
            m->saw.period  = (uint16_t)((m->saw.period & 0x00FFu) | ((uint16_t)(val & 0x0Fu) << 8));
            m->saw.enabled = (val & 0x80u) != 0u;
            if (!m->saw.enabled) {
                m->saw.accum = 0u;
                m->saw.step  = 0u;
            }
            break;

        case 0xB003u:
            m->b003 = val;
            break;

        case 0xC000u: case 0xC001u: case 0xC002u: case 0xC003u:
            m->prg_8k = (uint8_t)(val & 0x1Fu);
            break;

        case 0xD000u: case 0xD001u: case 0xD002u: case 0xD003u:
            m->r[reg & 0x03u] = val;
            break;

        case 0xE000u: case 0xE001u: case 0xE002u: case 0xE003u:
            m->r[4u + (reg & 0x03u)] = val;
            break;

        case 0xF000u:
            m->irq_latch = val;
            break;

        case 0xF001u:
            m->irq_pending    = false;
            m->irq_prescaler  = 341;
            m->irq_ack_enable = (val & 0x01u) != 0u;
            m->irq_enable     = (val & 0x02u) != 0u;
            m->irq_mode_cycle = (val & 0x04u) != 0u;
            if (m->irq_enable) {
                m->irq_counter = m->irq_latch;
            }
            vrc6_update_irq(m);
            break;

        case 0xF002u:
            m->irq_pending = false;
            m->irq_enable  = m->irq_ack_enable;
            vrc6_update_irq(m);
            break;

        default:
            break;
    }
}

/* ==================== PPU Reads & Writes ==================== */

uint8_t vrc6_ppu_read(const Vrc6 *m, uint16_t addr, const uint8_t *ciram) {
    addr &= 0x3FFFu;

    if (addr < 0x2000u) {
        if (m->chr_size == 0u) return 0xFFu;
        uint32_t bank1k = vrc6_chr_bank_1k(m, addr);
        uint32_t offset = ((bank1k * 1024u) | (addr & 0x03FFu)) % m->chr_size;
        return m->chr_mem[offset];
    }

    addr &= 0x2FFFu;
    uint8_t bank1k = vrc6_nt_bank_1k(m, addr);
    if ((m->b003 & 0x10u) != 0u) {
        if (m->chr_size == 0u) return 0xFFu;
        uint32_t offset = (((uint32_t)bank1k * 1024u) | (addr & 0x03FFu)) % m->chr_size;
        return m->chr_mem[offset];
    }

    return ciram[((uint16_t)(bank1k & 0x01u) << 10) | (addr & 0x03FFu)];
}

void vrc6_ppu_write(Vrc6 *m, uint16_t addr, uint8_t val, uint8_t *ciram) {
    addr &= 0x3FFFu;

    if (addr < 0x2000u) {
        if (m->chr_is_ram && m->chr_size > 0u) {
            uint32_t bank1k = vrc6_chr_bank_1k(m, addr);
            uint32_t offset = ((bank1k * 1024u) | (addr & 0x03FFu)) % m->chr_size;
            m->chr_mem[offset] = val;
        }
        return;
    }

    addr &= 0x2FFFu;
    uint8_t bank1k = vrc6_nt_bank_1k(m, addr);
    if ((m->b003 & 0x10u) != 0u) {
        if (m->chr_is_ram && m->chr_size > 0u) {
            uint32_t offset = (((uint32_t)bank1k * 1024u) | (addr & 0x03FFu)) % m->chr_size;
            m->chr_mem[offset] = val;
        }
        return;
    }

    ciram[((uint16_t)(bank1k & 0x01u) << 10) | (addr & 0x03FFu)] = val;
}

/* ==================== Expansion Audio Output ==================== */

float vrc6_audio_output(const Vrc6 *m) {
    uint8_t p1  = vrc6_pulse_output(&m->pulse1);
    uint8_t p2  = vrc6_pulse_output(&m->pulse2);
    uint8_t saw = m->saw.enabled ? (uint8_t)(m->saw.accum >> 3) : 0u;

    uint8_t sum = (uint8_t)(p1 + p2 + saw); /* 0..61 */
    /* Linear 6-bit DAC, inverted relative to 2A03, 15 == full 2A03 pulse (~0.149377) */
    return -((float)sum) * (0.1493768f / 15.0f);
}