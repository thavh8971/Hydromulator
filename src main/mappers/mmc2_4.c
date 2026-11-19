#include "mmc2_4.h"
#include <stddef.h>
#include <string.h>

static uint32_t mmc24_map_chr(const Mmc24 *m, uint16_t addr) {
    uint32_t banks_4k = m->chr_size / 4096u;
    if (banks_4k == 0u) {
        return addr % m->chr_size;
    }
    uint32_t bank4k = (addr < 0x1000u)
        ? ((m->latch0 == 0xFDu) ? m->chr_fd0 : m->chr_fe0)
        : ((m->latch1 == 0xFDu) ? m->chr_fd1 : m->chr_fe1);
    return ((bank4k % banks_4k) * 4096u) | (addr & 0x0FFFu);
}

void mmc24_power_on(Mmc24 *m, const Mmc24Config *cfg) {
    memset(m, 0, sizeof(*m));
    if (cfg != NULL) {
        m->prg_rom           = cfg->prg_rom;
        m->prg_size          = cfg->prg_size;
        m->chr_mem           = cfg->chr_mem;
        m->chr_size          = cfg->chr_size;
        m->chr_is_ram        = cfg->chr_is_ram;
        m->prg_ram           = cfg->prg_ram;
        m->prg_ram_present   = cfg->prg_ram_present;
        m->is_mmc4           = cfg->is_mmc4;
        m->initial_mirroring = cfg->initial_mirroring;
    }
    mmc24_reset(m);
}

void mmc24_reset(Mmc24 *m) {
    m->mirroring = m->initial_mirroring;
    m->prg_bank  = 0u;
    m->chr_fd0   = 0u;
    m->chr_fe0   = 0u;
    m->chr_fd1   = 0u;
    m->chr_fe1   = 0u;
    m->latch0    = 0xFEu;
    m->latch1    = 0xFEu;
}

uint8_t mmc24_cpu_read(const Mmc24 *m, uint16_t addr, uint8_t open_bus, bool *driven_out) {
    if (addr >= 0x6000u && addr < 0x8000u) {
        if (!m->prg_ram_present || m->prg_ram == NULL) {
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

    if (m->is_mmc4) {
        /* Mapper 10 (MMC4): 16 KiB switchable at $8000-$BFFF, 16 KiB fixed last at $C000-$FFFF */
        uint32_t banks_16k = m->prg_size / 16384u;
        if (banks_16k == 0u) {
            return m->prg_rom[(addr - 0x8000u) % m->prg_size];
        }
        uint32_t bank = (addr < 0xC000u) ? ((uint32_t)m->prg_bank % banks_16k)
                                         : (banks_16k - 1u);
        return m->prg_rom[(bank * 16384u) | (addr & 0x3FFFu)];
    }

    /* Mapper 9 (MMC2): 8 KiB switchable at $8000-$9FFF, 24 KiB fixed (-3,-2,-1) at $A000-$FFFF */
    uint32_t banks_8k = m->prg_size / 8192u;
    if (banks_8k == 0u) {
        return m->prg_rom[(addr - 0x8000u) % m->prg_size];
    }
    uint8_t slot = (uint8_t)((addr - 0x8000u) >> 13);
    uint32_t bank = 0u;
    if (slot == 0u)      bank = (uint32_t)m->prg_bank % banks_8k;
    else if (slot == 1u) bank = (banks_8k >= 3u) ? (banks_8k - 3u) : (1u % banks_8k);
    else if (slot == 2u) bank = (banks_8k >= 2u) ? (banks_8k - 2u) : (2u % banks_8k);
    else                 bank = banks_8k - 1u;
    return m->prg_rom[(bank * 8192u) | (addr & 0x1FFFu)];
}

void mmc24_cpu_write(Mmc24 *m, uint16_t addr, uint8_t val) {
    if (addr >= 0x6000u && addr < 0x8000u) {
        if (m->prg_ram_present && m->prg_ram != NULL) {
            m->prg_ram[addr - 0x6000u] = val;
        }
        return;
    }
    if (addr < 0x8000u) {
        return;
    }

    switch (addr & 0xF000u) {
        case 0xA000u: m->prg_bank  = (uint8_t)(val & 0x0Fu); break;
        case 0xB000u: m->chr_fd0   = (uint8_t)(val & 0x1Fu); break;
        case 0xC000u: m->chr_fe0   = (uint8_t)(val & 0x1Fu); break;
        case 0xD000u: m->chr_fd1   = (uint8_t)(val & 0x1Fu); break;
        case 0xE000u: m->chr_fe1   = (uint8_t)(val & 0x1Fu); break;
        case 0xF000u: m->mirroring = (val & 0x01u) ? 0u : 1u; break; /* 0=H, 1=V */
        default: break;
    }
}

uint8_t mmc24_ppu_chr_read(Mmc24 *m, uint16_t addr, bool latch_edge) {
    addr &= 0x1FFFu;
    uint8_t val = m->chr_mem[mmc24_map_chr(m, addr)];

    if (latch_edge) {
        if (m->is_mmc4) {
            /* MMC4 triggers across full 8-byte ranges on both pattern tables */
            if (addr >= 0x0FD8u && addr <= 0x0FDFu) {
                m->latch0 = 0xFDu;
            } else if (addr >= 0x0FE8u && addr <= 0x0FEFu) {
                m->latch0 = 0xFEu;
            } else if (addr >= 0x1FD8u && addr <= 0x1FDFu) {
                m->latch1 = 0xFDu;
            } else if (addr >= 0x1FE8u && addr <= 0x1FEFu) {
                m->latch1 = 0xFEu;
            }
        } else {
            /* MMC2 triggers only on exact $0FD8 / $0FE8 for latch 0 */
            if (addr == 0x0FD8u) {
                m->latch0 = 0xFDu;
            } else if (addr == 0x0FE8u) {
                m->latch0 = 0xFEu;
            } else if (addr >= 0x1FD8u && addr <= 0x1FDFu) {
                m->latch1 = 0xFDu;
            } else if (addr >= 0x1FE8u && addr <= 0x1FEFu) {
                m->latch1 = 0xFEu;
            }
        }
    }

    return val;
}

void mmc24_ppu_chr_write(Mmc24 *m, uint16_t addr, uint8_t val) {
    if (m->chr_is_ram) {
        m->chr_mem[mmc24_map_chr(m, addr & 0x1FFFu)] = val;
    }
}