#include "mmc3.h"
#include <stddef.h>
#include <string.h>

static void mmc3_clock_irq_counter(Mmc3 *m) {
    if (m->irq_reload) {
        m->irq_counter = m->irq_latch;
        m->irq_reload  = false;
        if (m->irq_counter == 0u && m->irq_enable && m->bus.on_irq != NULL) {
            m->bus.on_irq(m->bus.ctx, MMC3_IRQ_MASK, true);
        }
    } else if (m->irq_counter == 0u) {
        m->irq_counter = m->irq_latch;
        if (!m->rev_a && m->irq_counter == 0u && m->irq_enable && m->bus.on_irq != NULL) {
            m->bus.on_irq(m->bus.ctx, MMC3_IRQ_MASK, true);
        }
    } else {
        m->irq_counter--;
        if (m->irq_counter == 0u && m->irq_enable && m->bus.on_irq != NULL) {
            m->bus.on_irq(m->bus.ctx, MMC3_IRQ_MASK, true);
        }
    }
}

void mmc3_power_on(Mmc3 *m, const Mmc3Config *cfg, const Mmc3Bus *bus) {
    memset(m, 0, sizeof(*m));
    if (bus != NULL) {
        m->bus = *bus;
    }
    if (cfg != NULL) {
        m->prg_rom           = cfg->prg_rom;
        m->prg_size          = cfg->prg_size;
        m->chr_mem           = cfg->chr_mem;
        m->chr_size          = cfg->chr_size;
        m->chr_is_ram        = cfg->chr_is_ram;
        m->prg_ram           = cfg->prg_ram;
        m->prg_ram_present   = cfg->prg_ram_present;
        m->initial_mirroring = cfg->initial_mirroring;
        m->rev_a             = cfg->rev_a;
        m->mc_acc            = cfg->mc_acc;
        m->is_mmc6           = cfg->is_mmc6;
    }
    mmc3_reset(m);
}

void mmc3_reset(Mmc3 *m) {
    m->mirroring      = m->initial_mirroring;
    m->select         = 0u;
    memset(m->regs, 0, sizeof(m->regs));
    m->irq_latch      = 0u;
    m->irq_counter    = 0u;
    m->irq_reload     = false;
    m->irq_enable     = false;
    m->prev_a12       = false;
    m->a12_current    = false;
    m->a12_low_m2     = 3u;
    m->mc_acc_div     = 0u;
    m->prg_ram_enable = m->prg_ram_present;
    m->prg_ram_ro     = false;
    m->mmc6_ram_ctrl  = 0u;
    if (m->bus.on_irq != NULL) {
        m->bus.on_irq(m->bus.ctx, MMC3_IRQ_MASK, false);
    }
}

void mmc3_sync_a12(Mmc3 *m, uint16_t ppu_vbus) {
    bool a12 = (ppu_vbus & 0x1000u) != 0u;
    m->a12_current = a12;
    m->prev_a12    = a12;
    m->a12_low_m2  = a12 ? 0u : 3u;
}

void mmc3_on_m2_falling(Mmc3 *m) {
    if (m->mc_acc) {
        return;
    }
    if (!m->a12_current) {
        if (m->a12_low_m2 < 3u) {
            m->a12_low_m2++;
        }
    } else {
        m->a12_low_m2 = 0u;
    }
}

void mmc3_watch_a12(Mmc3 *m, uint16_t ppu_vbus) {
    bool a12 = (ppu_vbus & 0x1000u) != 0u;
    if (m->mc_acc) {
        if (a12 && !m->prev_a12) {
            if (m->mc_acc_div == 0u) {
                mmc3_clock_irq_counter(m);
            }
            m->mc_acc_div = (uint8_t)((m->mc_acc_div + 1u) & 0x07u);
        }
    } else {
        if (a12) {
            if (!m->a12_current && m->a12_low_m2 >= 3u) {
                mmc3_clock_irq_counter(m);
            }
            m->a12_low_m2 = 0u;
        }
        m->a12_current = a12;
    }
    m->prev_a12 = a12;
}

uint32_t mmc3_map_chr(const Mmc3 *m, uint16_t addr) {
    uint32_t banks_1k = m->chr_size / 1024u;
    if (banks_1k == 0u) {
        return addr % m->chr_size;
    }

    uint8_t slot = (uint8_t)((addr >> 10) & 0x07u);
    if (m->select & 0x80u) {
        slot ^= 0x04u;
    }

    uint32_t bank = 0u;
    switch (slot) {
        case 0: bank = (uint32_t)(m->regs[0] & 0xFEu); break;
        case 1: bank = (uint32_t)(m->regs[0] | 0x01u); break;
        case 2: bank = (uint32_t)(m->regs[1] & 0xFEu); break;
        case 3: bank = (uint32_t)(m->regs[1] | 0x01u); break;
        case 4: bank = m->regs[2]; break;
        case 5: bank = m->regs[3]; break;
        case 6: bank = m->regs[4]; break;
        case 7: bank = m->regs[5]; break;
    }
    return ((bank % banks_1k) * 1024u) | (addr & 0x03FFu);
}

uint8_t mmc3_cpu_read(const Mmc3 *m, uint16_t addr, uint8_t open_bus, bool *driven_out) {
    if (addr >= 0x6000u && addr < 0x8000u) {
        if (!m->prg_ram_present || m->prg_ram == NULL) {
            if (driven_out) *driven_out = false;
            return open_bus;
        }
        if (m->is_mmc6) {
            if ((m->select & 0x20u) == 0u || addr < 0x7000u) {
                if (driven_out) *driven_out = false;
                return open_bus;
            }
            bool low_r  = (m->mmc6_ram_ctrl & 0x20u) != 0u;
            bool high_r = (m->mmc6_ram_ctrl & 0x80u) != 0u;
            if (!low_r && !high_r) {
                if (driven_out) *driven_out = false;
                return open_bus;
            }
            bool in_high = (addr & 0x0200u) != 0u;
            if (driven_out) *driven_out = true;
            if ((!in_high && !low_r) || (in_high && !high_r)) {
                return 0x00u;
            }
            return m->prg_ram[addr & 0x03FFu];
        }
        if (!m->prg_ram_enable) {
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
    if (banks_8k < 2u) {
        return m->prg_rom[(addr - 0x8000u) % m->prg_size];
    }

    uint8_t slot = (uint8_t)((addr - 0x8000u) >> 13);
    bool prg_mode = (m->select & 0x40u) != 0u;
    uint32_t bank = 0u;

    if (slot == 0u)      bank = prg_mode ? (banks_8k - 2u) : m->regs[6];
    else if (slot == 1u) bank = m->regs[7];
    else if (slot == 2u) bank = prg_mode ? m->regs[6] : (banks_8k - 2u);
    else                 bank = banks_8k - 1u;

    return m->prg_rom[((bank % banks_8k) * 8192u) | (addr & 0x1FFFu)];
}

void mmc3_cpu_write(Mmc3 *m, uint16_t addr, uint8_t val) {
    if (addr >= 0x6000u && addr < 0x8000u) {
        if (!m->prg_ram_present || m->prg_ram == NULL) {
            return;
        }
        if (m->is_mmc6) {
            if ((m->select & 0x20u) == 0u || addr < 0x7000u) {
                return;
            }
            bool in_high = (addr & 0x0200u) != 0u;
            bool can_w = in_high ? ((m->mmc6_ram_ctrl & 0xC0u) == 0xC0u)
                                 : ((m->mmc6_ram_ctrl & 0x30u) == 0x30u);
            if (can_w) {
                m->prg_ram[addr & 0x03FFu] = val;
            }
            return;
        }
        if (!m->prg_ram_enable || m->prg_ram_ro) {
            return;
        }
        m->prg_ram[addr - 0x6000u] = val;
        return;
    }

    if (addr < 0x8000u) {
        return;
    }

    bool even = (addr & 1u) == 0u;
    if (addr < 0xA000u) {
        if (even) {
            m->select = val;
            if (m->is_mmc6 && (val & 0x20u) == 0u) {
                m->mmc6_ram_ctrl = 0u;
            }
        } else {
            uint8_t reg = (uint8_t)(m->select & 0x07u);
            if (reg == 6u || reg == 7u) {
                val &= 0x3Fu;
            }
            m->regs[reg] = val;
        }
    } else if (addr < 0xC000u) {
        if (even) {
            if (m->mirroring != 4u) {
                m->mirroring = (val & 0x01u) ? 0u : 1u; /* 0=H, 1=V */
            }
        } else {
            if (m->is_mmc6) {
                if (m->select & 0x20u) {
                    m->mmc6_ram_ctrl = (uint8_t)(val & 0xF0u);
                }
            } else {
                m->prg_ram_enable = (val & 0x80u) != 0u;
                m->prg_ram_ro     = (val & 0x40u) != 0u;
            }
        }
    } else if (addr < 0xE000u) {
        if (even) {
            m->irq_latch = val;
        } else {
            m->irq_counter = 0u;
            m->irq_reload  = true;
            if (m->mc_acc) {
                m->mc_acc_div = 0u;
            }
        }
    } else {
        if (even) {
            m->irq_enable = false;
            if (m->bus.on_irq != NULL) {
                m->bus.on_irq(m->bus.ctx, MMC3_IRQ_MASK, false);
            }
        } else {
            m->irq_enable = true;
        }
    }
}