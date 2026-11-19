#include "mapper6.h"
#include <stddef.h>
#include <string.h>

/* ==================== IRQ Helpers ==================== */

static inline void mapper6_update_irq(const Mapper6 *m) {
    if (m->bus.on_irq == NULL) {
        return;
    }
    m->bus.on_irq(m->bus.ctx, MAPPER6_IRQ_SMC, m->irq_pending);
    m->bus.on_irq(m->bus.ctx, MAPPER6_IRQ_FDS, m->fds_irq_pending);
}

static void mapper6_clock_smc_counter(Mapper6 *m) {
    if (!m->irq_enable) {
        return;
    }
    if (m->irq_counter == 0xFFFFu) {
        m->irq_counter = 0x0000u;
        m->irq_pending = true;
        mapper6_update_irq(m);
    } else {
        m->irq_counter++;
    }
}

/* ==================== Address Mapping ==================== */

static uint32_t mapper6_map_prg(const Mapper6 *m, uint16_t addr) {
    if (m->prg_size == 0u) {
        return 0u;
    }

    uint32_t prg8_count  = m->prg_size / 8192u;
    uint32_t prg16_count = m->prg_size / 16384u;
    uint32_t prg32_count = m->prg_size / 32768u;
    if (prg8_count  == 0u) prg8_count  = 1u;
    if (prg16_count == 0u) prg16_count = 1u;
    if (prg32_count == 0u) prg32_count = 1u;

    if (m->prg_mode_2m4m == MAPPER6_PRG_4M) {
        uint8_t slot = (uint8_t)((addr >> 13) & 0x03u);
        uint32_t bank = (uint32_t)(m->prg_4m[slot] & 0x3Fu) % prg8_count;
        return ((bank << 13) | (addr & 0x1FFFu)) % m->prg_size;
    }

    if (m->prg_mode_2m4m == MAPPER6_PRG_2M) {
        uint8_t slot = (uint8_t)((addr >> 13) & 0x03u);
        uint32_t bank = (uint32_t)(m->prg_2m[slot] & 0x3Fu) % prg8_count;
        return ((bank << 13) | (addr & 0x1FFFu)) % m->prg_size;
    }

    /* Latch-based 1M modes (0..7) */
    uint8_t l = m->latch;
    bool hi_half = (addr >= 0xC000u);

    switch (m->latch_mode & 0x07u) {
        case 0: { /* UNROM: 16K $8000-#0..7, fixed #7 at $C000 */
            uint32_t bank = hi_half ? (7u % prg16_count) : ((uint32_t)(l & 0x07u) % prg16_count);
            return ((bank << 14) | (addr & 0x3FFFu)) % m->prg_size;
        }
        case 1: { /* UN1ROM+CHRSW: 16K $8000 via bits 5..2, fixed #7 at $C000 */
            uint32_t bank = hi_half ? (7u % prg16_count) : ((uint32_t)((l >> 2) & 0x0Fu) % prg16_count);
            return ((bank << 14) | (addr & 0x3FFFu)) % m->prg_size;
        }
        case 2: { /* UOROM: 16K $8000-#0..15, fixed #15 at $C000 */
            uint32_t bank = hi_half ? (15u % prg16_count) : ((uint32_t)(l & 0x0Fu) % prg16_count);
            return ((bank << 14) | (addr & 0x3FFFu)) % m->prg_size;
        }
        case 3: { /* Reverse UOROM+CHRSW: fixed #15 at $8000, switchable #0..15 at $C000 */
            uint32_t bank = hi_half ? ((uint32_t)(l & 0x0Fu) % prg16_count) : (15u % prg16_count);
            return ((bank << 14) | (addr & 0x3FFFu)) % m->prg_size;
        }
        case 4: { /* GNROM: switchable 32K #0..3 via bits 5..4 */
            uint32_t bank = (uint32_t)((l >> 4) & 0x03u) % prg32_count;
            return ((bank << 15) | (addr & 0x7FFFu)) % m->prg_size;
        }
        case 5:   /* CNROM-256 */
        case 6:   /* CNROM-128 */
        case 7:   /* NROM-256 */
        default: {
            uint32_t bank = 3u % prg32_count;
            return ((bank << 15) | (addr & 0x7FFFu)) % m->prg_size;
        }
    }
}

static uint32_t mapper6_map_chr(const Mapper6 *m, uint16_t addr) {
    addr &= 0x1FFFu;

    if (m->chr_1k_mode) {
        if (m->mmc4_enabled) {
            uint8_t half_idx = (addr < 0x1000u)
                ? ((m->mmc4_latch0 == 0xFDu) ? 0u : 2u)
                : ((m->mmc4_latch1 == 0xFDu) ? 4u : 6u);
            uint8_t slot = (uint8_t)(half_idx | ((addr >> 10) & 0x01u));
            uint32_t bank4k = (uint32_t)(m->chr_1k[slot] >> 2);
            return ((bank4k << 12) | (addr & 0x0FFFu)) & (MAPPER6_CHR_RAM_SIZE - 1u);
        }
        uint8_t slot = (uint8_t)(addr >> 10);
        return (((uint32_t)m->chr_1k[slot] << 10) | (addr & 0x03FFu)) & (MAPPER6_CHR_RAM_SIZE - 1u);
    }

    uint8_t bank8k = 0u;
    if (m->prg_mode_2m4m != MAPPER6_PRG_LATCH) {
        bank8k = (uint8_t)(m->chr_8k_reg & 0x03u);
    } else {
        switch (m->latch_mode & 0x07u) {
            case 1:
            case 4:
            case 5:
                bank8k = (uint8_t)(m->latch & 0x03u);
                break;
            case 3:
                bank8k = (uint8_t)((m->latch >> 4) & 0x03u);
                break;
            case 6:
                bank8k = (uint8_t)(m->latch & 0x01u);
                break;
            case 0:
            case 2:
            case 7:
            default:
                bank8k = 0u;
                break;
        }
    }
    return (((uint32_t)bank8k << 13) | addr) & (MAPPER6_CHR_RAM_SIZE - 1u);
}

static inline bool mapper6_chr_writable(const Mapper6 *m) {
    /* Modes 0..3 allow CHR-RAM writes; modes 4..7 write-protect CHR memory */
    return (m->latch_mode & 0x04u) == 0u;
}

static uint16_t mapper6_mirror_ciram_addr(Mapper6Mirror mode, uint16_t addr) {
    uint16_t off   = (uint16_t)(addr & 0x03FFu);
    uint16_t table = (uint16_t)((addr >> 10) & 0x03u);
    switch (mode) {
        case MAPPER6_MIRROR_SINGLE_LO:  return off;
        case MAPPER6_MIRROR_SINGLE_HI:  return (uint16_t)(0x0400u | off);
        case MAPPER6_MIRROR_VERTICAL:   return (uint16_t)(((table & 0x01u) << 10) | off);
        case MAPPER6_MIRROR_HORIZONTAL: return (uint16_t)(((table >> 1) << 10) | off);
    }
    return off;
}

/* ==================== Register Write Helpers ==================== */

static void mapper6_write_42fx(Mapper6 *m, uint16_t addr, uint8_t val) {
    m->latch_mode        = (uint8_t)((val >> 5) & 0x07u);
    m->prg_write_protect = (addr & 0x02u) != 0u;

    /* A0 selects 1-screen (0) vs 2-screen (1); D4 selects page 0/Vert (0) vs page 1/Horiz (1) */
    uint8_t mir = (uint8_t)(((addr & 0x01u) << 1) | ((val >> 4) & 0x01u));
    m->mirroring = (Mapper6Mirror)mir;

    m->raw_42fx = (uint8_t)((addr & 0x03u) | ((val >> 2) & 0x04u) | ((val >> 2) & 0x38u));
}

static void mapper6_write_43fx(Mapper6 *m, uint16_t addr, uint8_t val) {
    m->raw_43fx = (uint8_t)(addr & 0x03u);
    if (addr & 0x01u) {
        m->prg_mode_2m4m = MAPPER6_PRG_LATCH;
    } else if (addr & 0x02u) {
        m->prg_mode_2m4m = MAPPER6_PRG_2M;
    } else {
        m->prg_mode_2m4m = MAPPER6_PRG_4M;
    }
    m->chr_8k_reg = (uint8_t)(val & 0x03u);
    m->latch = (uint8_t)((m->latch & 0xFCu) | (val & 0x03u));
}

static void mapper6_write_4500(Mapper6 *m, uint8_t val) {
    m->smc_mode        = val;
    m->chr_1k_mode     = (val & 0x01u) != 0u;
    m->ciram_nt_mode   = (val & 0x02u) != 0u;
    m->mmc4_enabled    = (val & 0x04u) == 0u;
    m->irq_source_pa12 = (val & 0x08u) != 0u;
    m->wram_bank       = (uint8_t)((val >> 4) & 0x03u);
}

/* ==================== Lifecycle ==================== */

static void mapper6_init_registers(Mapper6 *m, uint8_t submapper, bool vertical_mirroring) {
    m->latch      = 0u;
    m->chr_8k_reg = 0u;

    uint32_t banks8k = m->prg_size / 8192u;
    for (uint8_t i = 0u; i < 4u; i++) {
        uint8_t b = (banks8k >= 4u) ? (uint8_t)((banks8k - 4u + i) & 0x3Fu) : i;
        m->prg_2m[i] = b;
        m->prg_4m[i] = b;
    }

    for (uint8_t i = 0u; i < 8u; i++) {
        m->chr_1k[i] = i;
    }
    for (uint8_t i = 0u; i < 4u; i++) {
        m->chr_nt[i] = i;
    }
    m->mmc4_latch0 = 0xFDu;
    m->mmc4_latch1 = 0xFDu;

    /* Hardware initial values per NESdev INES Mapper 006 spec */
    mapper6_write_4500(m, 0x42u);
    mapper6_write_42fx(m, 0x42FFu, (uint8_t)(((submapper & 0x07u) << 5) |
                                             (vertical_mirroring ? 0x00u : 0x10u)));
    mapper6_write_43fx(m, 0x43FFu, 0x00u);

    m->irq_counter     = 0u;
    m->irq_enable      = false;
    m->irq_pending     = false;
    m->fds_ctrl        = 0u;
    m->fds_mclk_accum  = 0u;
    m->fds_irq_pending = false;

    mapper6_update_irq(m);
}

void mapper6_power_on(Mapper6 *m, const Mapper6Config *cfg, const Mapper6Bus *bus) {
    memset(m, 0, sizeof(*m));
    if (bus != NULL) {
        m->bus = *bus;
    }

    uint8_t submapper = 1u;
    bool vertical = false;

    if (cfg != NULL) {
        submapper = (uint8_t)(cfg->submapper & 0x07u);
        vertical  = cfg->vertical_mirroring;

        if (cfg->prg_rom != NULL && cfg->prg_size > 0u) {
            uint32_t copy_sz = (cfg->prg_size <= MAPPER6_MAX_PRG_SIZE) ? cfg->prg_size : MAPPER6_MAX_PRG_SIZE;
            memcpy(m->prg_dram, cfg->prg_rom, copy_sz);
            m->prg_size = copy_sz;
        }
        if (cfg->chr_rom != NULL && cfg->chr_size > 0u && !cfg->chr_is_ram) {
            uint32_t copy_sz = (cfg->chr_size <= MAPPER6_CHR_RAM_SIZE) ? cfg->chr_size : MAPPER6_CHR_RAM_SIZE;
            memcpy(m->chr_ram, cfg->chr_rom, copy_sz);
        }
        if (cfg->has_trainer && cfg->trainer != NULL) {
            /* 512-byte trainer loaded into WRAM bank 0 at CPU $7000-$71FF */
            memcpy(&m->wram[0x1000u], cfg->trainer, 512u);
            m->boot_stage = 1u;
        }
    }

    mapper6_init_registers(m, submapper, vertical);
}

void mapper6_reset(Mapper6 *m) {
    /* Soft reset preserves WRAM & PRG-DRAM (per Magic Card trainer spec) and disables boot stub */
    m->boot_stage      = 0u;
    m->irq_enable      = false;
    m->irq_pending     = false;
    m->fds_ctrl        = 0u;
    m->fds_mclk_accum  = 0u;
    m->fds_irq_pending = false;
    mapper6_update_irq(m);
}

/* ==================== Clocks ==================== */

void mapper6_cpu_cycle(Mapper6 *m) {
    if (!m->irq_source_pa12) {
        mapper6_clock_smc_counter(m);
    }

    if (m->fds_ctrl & 0x80u) {
        /* 12 master clocks per NTSC CPU cycle; FDS disk data IRQ triggers every 1792 master clocks */
        m->fds_mclk_accum = (uint16_t)(m->fds_mclk_accum + 12u);
        if (m->fds_mclk_accum >= 1792u) {
            m->fds_mclk_accum = (uint16_t)(m->fds_mclk_accum - 1792u);
            m->fds_irq_pending = true;
            mapper6_update_irq(m);
        }
    }
}

void mapper6_watch_a12(Mapper6 *m, uint16_t ppu_addr) {
    bool a12 = (ppu_addr & 0x1000u) != 0u;
    if (m->irq_source_pa12 && a12 && !m->prev_a12) {
        mapper6_clock_smc_counter(m);
    }
    m->prev_a12 = a12;
}

/* ==================== CPU Reads & Writes ==================== */

uint8_t mapper6_cpu_read(Mapper6 *m, uint16_t addr, uint8_t open_bus, bool *driven_out) {
    if (driven_out != NULL) {
        *driven_out = true;
    }

    /* Hard-reset trainer stub: $FFFC/$FFFD -> $41ED: JSR $7003 ; JMP ($FFFC) */
    if (m->boot_stage == 1u) {
        if (addr == 0xFFFCu) {
            return 0xEDu;
        }
        if (addr == 0xFFFDu) {
            m->boot_stage = 2u;
            return 0x41u;
        }
    } else if (m->boot_stage == 2u && addr >= 0x41EDu && addr <= 0x41F2u) {
        switch (addr) {
            case 0x41EDu: return 0x20u; /* JSR $7003 */
            case 0x41EEu: return 0x03u;
            case 0x41EFu: return 0x70u;
            case 0x41F0u: return 0x6Cu; /* JMP ($FFFC) */
            case 0x41F1u: return 0xFCu;
            case 0x41F2u:
                m->boot_stage = 0u;
                return 0xFFu;
        }
    }

    if (addr >= 0x8000u) {
        if (m->prg_size == 0u) {
            if (driven_out != NULL) *driven_out = false;
            return open_bus;
        }
        return m->prg_dram[mapper6_map_prg(m, addr)];
    }

    if (addr >= 0x6000u) {
        uint32_t off = ((uint32_t)m->wram_bank << 13) | (addr & 0x1FFFu);
        return m->wram[off];
    }

    if (addr >= 0x5000u) {
        return m->scratch_ram[addr & 0x0FFFu];
    }

    switch (addr) {
        case 0x4030u: {
            uint8_t val = (uint8_t)(m->fds_irq_pending ? 0x02u : 0x00u);
            m->fds_irq_pending = false;
            mapper6_update_irq(m);
            return val;
        }

        case 0x4500u:
            /* Bit 6 = 1 (button not pressed), bit 7 = 0 (parallel port idle) */
            return (uint8_t)(0x40u | (m->raw_42fx & 0x3Fu));

        case 0x4501u:
            return (uint8_t)((m->latch & 0xFCu) | (m->raw_43fx & 0x03u));

        default:
            if (addr >= 0x4508u && addr <= 0x450Fu) {
                return 0x00u;
            }
            break;
    }

    if (driven_out != NULL) {
        *driven_out = false;
    }
    return open_bus;
}

void mapper6_cpu_write(Mapper6 *m, uint16_t addr, uint8_t val) {
    if (addr >= 0x8000u) {
        if (!m->prg_write_protect) {
            if (m->prg_size > 0u) {
                m->prg_dram[mapper6_map_prg(m, addr)] = val;
            }
            return;
        }

        /* Bus-conflict-free latch + 2M banking update */
        m->latch = val;
        m->chr_8k_reg = (uint8_t)(val & 0x03u);
        m->prg_2m[(addr >> 13) & 0x03u] = (uint8_t)((val >> 2) & 0x3Fu);
        return;
    }

    if (addr >= 0x6000u) {
        uint32_t off = ((uint32_t)m->wram_bank << 13) | (addr & 0x1FFFu);
        m->wram[off] = val;
        return;
    }

    if (addr >= 0x5000u) {
        m->scratch_ram[addr & 0x0FFFu] = val;
        return;
    }

    if (addr >= 0x4500u && addr <= 0x451Fu) {
        m->scratch_ram[0x0500u | (addr & 0x001Fu)] = val;
    }

    if (addr >= 0x42FCu && addr <= 0x42FFu) {
        mapper6_write_42fx(m, addr, val);
        return;
    }

    if (addr >= 0x43FCu && addr <= 0x43FFu) {
        mapper6_write_43fx(m, addr, val);
        return;
    }

    if (addr >= 0x4504u && addr <= 0x4507u) {
        uint8_t slot = (uint8_t)(addr & 0x03u);
        uint8_t bank = (uint8_t)(val & 0x3Fu);
        m->prg_4m[slot] = bank;
        if (m->prg_mode_2m4m != MAPPER6_PRG_4M) {
            m->prg_2m[slot] = bank;
        }
        return;
    }

    if (addr >= 0x4510u && addr <= 0x4517u) {
        m->chr_1k[addr & 0x07u] = val;
        return;
    }

    if (addr >= 0x4518u && addr <= 0x451Bu) {
        m->chr_nt[addr & 0x03u] = val;
        return;
    }

    switch (addr) {
        case 0x4024u:
            m->fds_irq_pending = false;
            mapper6_update_irq(m);
            break;

        case 0x4025u:
            m->fds_ctrl = val;
            if ((val & 0x80u) == 0u) {
                m->fds_irq_pending = false;
                m->fds_mclk_accum  = 0u;
                mapper6_update_irq(m);
            }
            break;

        case 0x4500u:
            mapper6_write_4500(m, val);
            break;

        case 0x4501u:
            m->irq_enable  = false;
            m->irq_pending = false;
            mapper6_update_irq(m);
            break;

        case 0x4502u:
            m->irq_counter = (uint16_t)((m->irq_counter & 0xFF00u) | val);
            m->irq_pending = false;
            mapper6_update_irq(m);
            break;

        case 0x4503u:
            m->irq_counter = (uint16_t)((m->irq_counter & 0x00FFu) | ((uint16_t)val << 8));
            m->irq_enable  = true;
            m->irq_pending = false;
            mapper6_update_irq(m);
            break;

        default:
            break;
    }
}

void mapper6_snoop_cpu_write(Mapper6 *m, uint16_t addr, uint8_t val) {
    if (addr >= 0x2000u && addr < 0x4000u) {
        m->scratch_ram[0x0800u | (addr & 0x0007u)] = val;
    } else if (addr >= 0x4000u && addr <= 0x4015u) {
        m->scratch_ram[addr & 0x001Fu] = val;
    }
}

/* ==================== PPU Reads & Writes ==================== */

uint8_t mapper6_ppu_read(Mapper6 *m, uint16_t addr, bool count_rd_edge, uint8_t *ciram) {
    addr &= 0x3FFFu;

    if (addr >= 0x2000u) {
        addr &= 0x2FFFu;
        if (m->ciram_nt_mode) {
            return ciram[mapper6_mirror_ciram_addr(m->mirroring, addr)];
        }
        uint8_t bank1k = m->chr_nt[(addr >> 10) & 0x03u];
        uint32_t offset = (((uint32_t)bank1k << 10) | (addr & 0x03FFu)) & (MAPPER6_CHR_RAM_SIZE - 1u);
        return m->chr_ram[offset];
    }

    uint8_t val = m->chr_ram[mapper6_map_chr(m, addr)];

    if (count_rd_edge && m->chr_1k_mode && m->mmc4_enabled) {
        if (addr >= 0x0FD8u && addr <= 0x0FDFu) {
            m->mmc4_latch0 = 0xFDu;
        } else if (addr >= 0x0FE8u && addr <= 0x0FEFu) {
            m->mmc4_latch0 = 0xFEu;
        } else if (addr >= 0x1FD8u && addr <= 0x1FDFu) {
            m->mmc4_latch1 = 0xFDu;
        } else if (addr >= 0x1FE8u && addr <= 0x1FEFu) {
            m->mmc4_latch1 = 0xFEu;
        }
    }

    return val;
}

void mapper6_ppu_write(Mapper6 *m, uint16_t addr, uint8_t val, uint8_t *ciram) {
    addr &= 0x3FFFu;

    if (addr >= 0x2000u) {
        addr &= 0x2FFFu;
        if (m->ciram_nt_mode) {
            ciram[mapper6_mirror_ciram_addr(m->mirroring, addr)] = val;
        } else if (mapper6_chr_writable(m)) {
            uint8_t bank1k = m->chr_nt[(addr >> 10) & 0x03u];
            uint32_t offset = (((uint32_t)bank1k << 10) | (addr & 0x03FFu)) & (MAPPER6_CHR_RAM_SIZE - 1u);
            m->chr_ram[offset] = val;
        }
        return;
    }

    if (mapper6_chr_writable(m)) {
        m->chr_ram[mapper6_map_chr(m, addr)] = val;
    }
}