#include "mmc1.h"
#include <stddef.h>
#include <string.h>

void mmc1_power_on(Mmc1 *m, const Mmc1Config *cfg) {
    memset(m, 0, sizeof(*m));
    if (cfg != NULL) {
        m->prg_rom           = cfg->prg_rom;
        m->prg_size          = cfg->prg_size;
        m->chr_mem           = cfg->chr_mem;
        m->chr_size          = cfg->chr_size;
        m->chr_is_ram        = cfg->chr_is_ram;
        m->prg_ram           = cfg->prg_ram;
        m->prg_ram_present   = cfg->prg_ram_present;
        m->initial_mirroring = cfg->initial_mirroring;
    }
    mmc1_reset(m);
}

void mmc1_reset(Mmc1 *m) {
    m->mirroring        = m->initial_mirroring;
    m->shift            = 0x10u;
    m->ctrl             = 0x0Cu;
    m->chr0             = 0u;
    m->chr1             = 0u;
    m->prg              = 0u;
    m->last_write_cycle = 0u;
}

uint32_t mmc1_map_chr(const Mmc1 *m, uint16_t addr) {
    uint32_t banks_4k = m->chr_size / 4096u;
    if (banks_4k == 0u) {
        return addr % m->chr_size;
    }
    if ((m->ctrl & 0x10u) == 0u) {
        uint32_t bank4k = (uint32_t)(m->chr0 & 0x1Eu) + ((addr >= 0x1000u) ? 1u : 0u);
        return ((bank4k % banks_4k) * 4096u) | (addr & 0x0FFFu);
    }
    uint32_t bank4k = (addr < 0x1000u) ? m->chr0 : m->chr1;
    return ((bank4k % banks_4k) * 4096u) | (addr & 0x0FFFu);
}

uint8_t mmc1_cpu_read(const Mmc1 *m, uint16_t addr, uint8_t open_bus, bool *driven_out) {
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
    uint32_t banks_16k = m->prg_size / 16384u;
    if (banks_16k == 0u) {
        return 0xFFu;
    }

    uint8_t mode = (uint8_t)((m->ctrl >> 2) & 0x03u);
    uint32_t bank = 0u;
    if (mode <= 1u) {
        bank = (uint32_t)(m->prg & 0x0Eu) + ((addr >= 0xC000u) ? 1u : 0u);
    } else if (mode == 2u) {
        bank = (addr < 0xC000u) ? 0u : (uint32_t)m->prg;
    } else {
        bank = (addr < 0xC000u) ? (uint32_t)m->prg : (banks_16k - 1u);
    }
    return m->prg_rom[((bank % banks_16k) * 16384u) | (addr & 0x3FFFu)];
}

void mmc1_cpu_write(Mmc1 *m, uint16_t addr, uint8_t val, uint64_t cpu_cycles) {
    if (addr >= 0x6000u && addr < 0x8000u) {
        if (m->prg_ram_present && m->prg_ram != NULL) {
            m->prg_ram[addr - 0x6000u] = val;
        }
        return;
    }
    if (addr < 0x8000u) {
        return;
    }

    if ((cpu_cycles - m->last_write_cycle) <= 1u) {
        m->last_write_cycle = cpu_cycles;
        return;
    }
    m->last_write_cycle = cpu_cycles;

    if (val & 0x80u) {
        m->shift = 0x10u;
        m->ctrl |= 0x0Cu;
        return;
    }

    bool complete = (m->shift & 0x01u) != 0u;
    m->shift = (uint8_t)((m->shift >> 1) | ((val & 0x01u) << 4));
    if (complete) {
        uint8_t reg = (uint8_t)((addr >> 13) & 0x03u);
        switch (reg) {
            case 0:
                m->ctrl = m->shift;
                switch (m->ctrl & 0x03u) {
                    case 0: m->mirroring = 2u; break; /* Single Lo */
                    case 1: m->mirroring = 3u; break; /* Single Hi */
                    case 2: m->mirroring = 1u; break; /* Vertical */
                    case 3: m->mirroring = 0u; break; /* Horizontal */
                }
                break;
            case 1: m->chr0 = m->shift; break;
            case 2: m->chr1 = m->shift; break;
            case 3: m->prg  = (uint8_t)(m->shift & 0x0Fu); break;
        }
        m->shift = 0x10u;
    }
}