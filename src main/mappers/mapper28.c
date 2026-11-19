#include "mapper28.h"
#include <stddef.h>
#include <string.h>

void mapper28_power_on(Mapper28 *m, const Mapper28Config *cfg) {
    memset(m, 0, sizeof(*m));
    if (cfg != NULL) {
        m->prg_rom         = cfg->prg_rom;
        m->prg_size        = cfg->prg_size;
        m->chr_mem         = cfg->chr_mem;
        m->chr_size        = cfg->chr_size;
        m->chr_is_ram      = cfg->chr_is_ram;
        m->prg_ram         = cfg->prg_ram;
        m->prg_ram_present = cfg->prg_ram_present;
        m->mode            = cfg->initial_vertical_mirror ? 0x02u : 0x03u;
    } else {
        m->mode            = 0x03u;
    }

    /* Power-on maps the last 16 KiB into $C000-$FFFF */
    m->prg_outer = 0xFFu;
}

void mapper28_reset(Mapper28 *m) {
    /* Action 53 mapper state is preserved across soft reset */
    (void)m;
}

static uint32_t mapper28_map_prg_16k(const Mapper28 *m, uint16_t addr) {
    uint8_t outer_size = (uint8_t)((m->mode >> 4) & 0x03u);
    uint8_t prg_mode   = (uint8_t)((m->mode >> 2) & 0x03u);
    uint32_t cpu_a14   = (addr >> 14) & 0x01u;
    uint32_t outer_16k = (uint32_t)m->prg_outer << 1;

    /* Fixed 16 KiB bank in modes 2 ($8000 fixed) and 3 ($C000 fixed) */
    if (prg_mode == 2u && cpu_a14 == 0u) {
        return outer_16k;
    }
    if (prg_mode == 3u && cpu_a14 == 1u) {
        return outer_16k | 1u;
    }

    uint32_t in_mask = (1u << (outer_size + 1u)) - 1u;
    uint32_t inner_val;
    if ((prg_mode & 0x02u) == 0u) {
        /* 32 KiB mode (modes 0, 1) */
        inner_val = (((uint32_t)(m->prg_inner & 0x07u)) << 1) | cpu_a14;
    } else {
        /* 16 KiB switchable half (modes 2, 3) */
        inner_val = (uint32_t)(m->prg_inner & 0x0Fu);
    }

    return (outer_16k & ~in_mask) | (inner_val & in_mask);
}

uint32_t mapper28_map_chr(const Mapper28 *m, uint16_t addr) {
    uint32_t banks_8k = m->chr_size / 8192u;
    if (banks_8k == 0u) {
        return addr % m->chr_size;
    }
    uint32_t bank = (uint32_t)(m->chr_bank & 0x03u) % banks_8k;
    return (bank * 8192u) | (addr & 0x1FFFu);
}

uint8_t mapper28_mirror_mode(const Mapper28 *m) {
    /* Convert Action 53 (0=1Lo, 1=1Hi, 2=V, 3=H) -> MirrorMode (0=H, 1=V, 2=1Lo, 3=1Hi) */
    switch (m->mode & 0x03u) {
        case 0: return 2u; /* MIRROR_SINGLE_LO */
        case 1: return 3u; /* MIRROR_SINGLE_HI */
        case 2: return 1u; /* MIRROR_VERTICAL */
        case 3: return 0u; /* MIRROR_HORIZONTAL */
    }
    return 0u;
}

uint8_t mapper28_cpu_read(const Mapper28 *m, uint16_t addr, uint8_t open_bus, bool *driven_out) {
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
        return m->prg_rom[(addr - 0x8000u) % m->prg_size];
    }

    uint32_t bank = mapper28_map_prg_16k(m, addr) % banks_16k;
    return m->prg_rom[(bank * 16384u) | (addr & 0x3FFFu)];
}

void mapper28_cpu_write(Mapper28 *m, uint16_t addr, uint8_t val) {
    if (addr >= 0x5000u && addr < 0x6000u) {
        m->reg_sel = (uint8_t)(((val >> 6) & 0x02u) | (val & 0x01u));
        return;
    }

    if (addr >= 0x6000u && addr < 0x8000u) {
        if (m->prg_ram_present && m->prg_ram != NULL) {
            m->prg_ram[addr - 0x6000u] = val;
        }
        return;
    }

    if (addr < 0x8000u) {
        return;
    }

    switch (m->reg_sel & 0x03u) {
        case 0: /* $00: CHR bank */
            m->chr_bank = (uint8_t)(val & 0x03u);
            if ((m->mode & 0x02u) == 0u) {
                m->mode = (uint8_t)((m->mode & 0xFEu) | ((val >> 4) & 0x01u));
            }
            break;

        case 1: /* $01: PRG inner bank */
            m->prg_inner = (uint8_t)(val & 0x0Fu);
            if ((m->mode & 0x02u) == 0u) {
                m->mode = (uint8_t)((m->mode & 0xFEu) | ((val >> 4) & 0x01u));
            }
            break;

        case 2: /* $80: Mode */
            m->mode = (uint8_t)(val & 0x3Fu);
            break;

        case 3: /* $81: PRG outer bank */
            m->prg_outer = val;
            break;
    }
}