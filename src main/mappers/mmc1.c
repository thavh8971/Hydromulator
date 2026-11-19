#include "mmc1.h"
#include <stddef.h>
#include <string.h>

static bool mmc1_wram_offset(const Mmc1 *m, uint16_t addr, uint32_t *out_off) {
    if (!m->prg_ram_present || m->prg_ram == NULL || m->prg_ram_size == 0u) {
        return false;
    }

    /* MMC1B: $E000 bit 4 disables PRG-RAM */
    if (!m->is_mmc1a && (m->prg & 0x10u) != 0u) {
        return false;
    }

    /* SNROM: CHR A16 ($A000 bit 4) acts as an additional WRAM /CE disable */
    if (m->is_snrom && (m->chr0 & 0x10u) != 0u) {
        return false;
    }

    uint8_t ram_bank = 0u;
    if (m->prg_ram_size >= 32768u) {
        /* SXROM: bit 2 = A13, bit 3 = A14 */
        ram_bank = (uint8_t)((m->chr0 >> 2) & 0x03u);
    } else if (m->prg_ram_size >= 16384u) {
        /* SZROM uses bit 4; SOROM uses bit 3 */
        ram_bank = (m->chr_size > 8192u)
            ? (uint8_t)((m->chr0 >> 4) & 0x01u)
            : (uint8_t)((m->chr0 >> 3) & 0x01u);
    }

    *out_off = (((uint32_t)ram_bank << 13) | (addr & 0x1FFFu)) % m->prg_ram_size;
    return true;
}

void mmc1_power_on(Mmc1 *m, const Mmc1Config *cfg) {
    memset(m, 0, sizeof(*m));
    if (cfg != NULL) {
        m->prg_rom           = cfg->prg_rom;
        m->prg_size          = cfg->prg_size;
        m->chr_mem           = cfg->chr_mem;
        m->chr_size          = cfg->chr_size;
        m->chr_is_ram        = cfg->chr_is_ram;
        m->prg_ram           = cfg->prg_ram;
        m->prg_ram_size      = cfg->prg_ram_size;
        m->prg_ram_present   = cfg->prg_ram_present;
        m->initial_mirroring = cfg->initial_mirroring;
        m->submapper         = cfg->submapper;
        m->is_mmc1a          = cfg->is_mmc1a;
        m->is_snrom          = !cfg->is_mmc1a &&
                               cfg->prg_size <= 262144u &&
                               cfg->chr_size <= 8192u &&
                               cfg->prg_ram_size == 8192u;
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
        uint32_t off = 0u;
        if (!mmc1_wram_offset(m, addr, &off)) {
            if (driven_out) *driven_out = false;
            return open_bus;
        }
        if (driven_out) *driven_out = true;
        return m->prg_ram[off];
    }
    if (addr < 0x8000u) {
        if (driven_out) *driven_out = false;
        return open_bus;
    }

    if (driven_out) *driven_out = true;
    if (m->submapper == 5u && m->prg_size <= 32768u) {
        return m->prg_rom[(addr - 0x8000u) % m->prg_size];
    }

    uint32_t banks_16k = m->prg_size / 16384u;
    if (banks_16k == 0u) {
        return 0xFFu;
    }

    /* SUROM / SXROM (512 KiB): $A000 bit 4 supplies PRG A18 across the entire $8000-$FFFF range */
    uint32_t outer_16k  = (m->prg_size > 262144u) ? (uint32_t)(m->chr0 & 0x10u) : 0u;
    uint32_t fixed_last = (banks_16k > 16u) ? 15u : (banks_16k - 1u);

    uint8_t mode = (uint8_t)((m->ctrl >> 2) & 0x03u);
    uint32_t bank = 0u;
    if (mode <= 1u) {
        bank = outer_16k | ((uint32_t)(m->prg & 0x0Eu) + ((addr >= 0xC000u) ? 1u : 0u));
    } else if (mode == 2u) {
        bank = outer_16k | ((addr < 0xC000u) ? 0u : (uint32_t)(m->prg & 0x0Fu));
    } else {
        bank = outer_16k | ((addr < 0xC000u) ? (uint32_t)(m->prg & 0x0Fu) : fixed_last);
    }

    if (m->is_mmc1a && (m->prg & 0x10u) != 0u) {
        bank = (bank & ~0x08u) | (uint32_t)(m->prg & 0x08u);
    }

    return m->prg_rom[((bank % banks_16k) * 16384u) | (addr & 0x3FFFu)];
}

void mmc1_cpu_write(Mmc1 *m, uint16_t addr, uint8_t val, uint64_t cpu_cycles) {
    if (addr >= 0x6000u && addr < 0x8000u) {
        uint32_t off = 0u;
        if (mmc1_wram_offset(m, addr, &off)) {
            m->prg_ram[off] = val;
        }
        return;
    }
    if (addr < 0x8000u) {
        return;
    }

    if (val & 0x80u) {
        m->shift = 0x10u;
        m->ctrl |= 0x0Cu;
        return;
    }

    if ((cpu_cycles - m->last_write_cycle) <= 1u) {
        m->last_write_cycle = cpu_cycles;
        return;
    }
    m->last_write_cycle = cpu_cycles;

    bool complete = (m->shift & 0x01u) != 0u;
    m->shift = (uint8_t)((m->shift >> 1) | ((val & 0x01u) << 4));
    if (complete) {
        uint8_t reg = (uint8_t)((addr >> 13) & 0x03u);
        switch (reg) {
            case 0:
                m->ctrl = m->shift;
                if (m->submapper != 7u) {
                    switch (m->ctrl & 0x03u) {
                        case 0: m->mirroring = 2u; break; /* Single Lo */
                        case 1: m->mirroring = 3u; break; /* Single Hi */
                        case 2: m->mirroring = 1u; break; /* Vertical */
                        case 3: m->mirroring = 0u; break; /* Horizontal */
                    }
                }
                break;
            case 1: m->chr0 = m->shift; break;
            case 2: m->chr1 = m->shift; break;
            case 3: m->prg  = (uint8_t)(m->shift & 0x1Fu); break;
        }
        m->shift = 0x10u;
    }
}