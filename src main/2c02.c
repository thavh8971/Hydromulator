#include "2c02.h"
#include <stddef.h>
#include <string.h>

enum {
    COMMIT_NT = 0x01u,
    COMMIT_AT = 0x02u,
    COMMIT_LO = 0x04u,
    COMMIT_HI = 0x08u
};

/* ==================== 1. NTSC 2C02 Palette LUT ==================== */

static inline bool in_color_phase(int color, int phase) {
    return ((color + phase) % 12) < 6;
}

static void build_palette_lut(uint32_t lut[512]) {
    static const float levels[16] = {
        0.228f, 0.312f, 0.552f, 0.880f,
        0.616f, 0.840f, 1.100f, 1.100f,
        0.192f, 0.256f, 0.448f, 0.712f,
        0.500f, 0.676f, 0.896f, 0.896f
    };
    static const float sin_tab[12] = {
         0.9659258f,  0.9659258f,  0.7071068f,  0.2588190f,
        -0.2588190f, -0.7071068f, -0.9659258f, -0.9659258f,
        -0.7071068f, -0.2588190f,  0.2588190f,  0.7071068f
    };
    static const float cos_tab[12] = {
         0.2588190f, -0.2588190f, -0.7071068f, -0.9659258f,
        -0.9659258f, -0.7071068f, -0.2588190f,  0.2588190f,
         0.7071068f,  0.9659258f,  0.9659258f,  0.7071068f
    };

    for (int idx = 0; idx < 512; idx++) {
        int color = idx & 0x0F;
        int level = (idx >> 4) & 0x03;
        int emphasis = (idx >> 6) & 0x07;
        if (color > 13) level = 1;

        float y = 0.0f, u = 0.0f, v = 0.0f;
        for (int p = 0; p < 12; p++) {
            int atten = 0;
            if (color < 0x0E) {
                if (((emphasis & 1) && in_color_phase(0x0C, p)) ||
                    ((emphasis & 2) && in_color_phase(0x04, p)) ||
                    ((emphasis & 4) && in_color_phase(0x08, p))) {
                    atten = 8;
                }
            }
            float low  = levels[level + atten];
            float high = levels[4 + level + atten];
            if (color == 0) low = high;
            if (color > 12) high = low;

            float sig = in_color_phase(color, p) ? high : low;
            float sample = ((sig - 0.312f) / (1.100f - 0.312f)) / 12.0f;
            y += sample;
            u += sample * sin_tab[p] * 2.0f;
            v += sample * cos_tab[p] * 2.0f;
        }

        float rf = 255.0f * (y + 1.139883f * v);
        float gf = 255.0f * (y - 0.394642f * u - 0.580622f * v);
        float bf = 255.0f * (y + 2.032062f * u);

        int ri = (int)(rf + (rf >= 0.0f ? 0.5f : -0.5f));
        int gi = (int)(gf + (gf >= 0.0f ? 0.5f : -0.5f));
        int bi = (int)(bf + (bf >= 0.0f ? 0.5f : -0.5f));
        if (ri < 0) ri = 0; else if (ri > 255) ri = 255;
        if (gi < 0) gi = 0; else if (gi > 255) gi = 255;
        if (bi < 0) bi = 0; else if (bi > 255) bi = 255;

        lut[idx] = 0xFF000000u | ((uint32_t)ri << 16) | ((uint32_t)gi << 8) | (uint32_t)bi;
    }
}

/* ==================== 2. Helpers, Open Bus & NMI ==================== */

static inline uint8_t palette_index(uint16_t addr) {
    uint8_t idx = (uint8_t)(addr & 0x1Fu);
    if ((idx & 0x13u) == 0x10u) {
        idx &= (uint8_t)(~0x10u);
    }
    return idx;
}

static inline uint8_t reverse_bits(uint8_t b) {
    b = (uint8_t)(((b & 0xF0u) >> 4) | ((b & 0x0Fu) << 4));
    b = (uint8_t)(((b & 0xCCu) >> 2) | ((b & 0x33u) << 2));
    b = (uint8_t)(((b & 0xAAu) >> 1) | ((b & 0x55u) << 1));
    return b;
}

static inline bool rendering_now(const Ricoh2C02 *ppu) {
    return ppu->show_bg || ppu->show_spr;
}

static inline bool eval_rendering(const Ricoh2C02 *ppu) {
    return ppu->eval_bg || ppu->eval_spr;
}

static inline uint8_t ppu_bus_read(Ricoh2C02 *ppu, uint16_t addr) {
    uint16_t masked = (uint16_t)(addr & 0x3FFFu);
    return (ppu->bus.read != NULL) ? ppu->bus.read(ppu->bus.ctx, masked) : 0xFFu;
}

static inline void ppu_bus_write(Ricoh2C02 *ppu, uint16_t addr, uint8_t val) {
    uint16_t masked = (uint16_t)(addr & 0x3FFFu);
    if (ppu->bus.write != NULL) {
        ppu->bus.write(ppu->bus.ctx, masked, val);
    }
}

static inline void refresh_io_latch(Ricoh2C02 *ppu, uint8_t val, uint8_t bitmask) {
    ppu->io_latch = (uint8_t)((ppu->io_latch & (uint8_t)(~bitmask)) | (val & bitmask));
    for (uint8_t b = 0u; b < 8u; b++) {
        uint8_t m = (uint8_t)(1u << b);
        if (bitmask & m) {
            ppu->io_decay[b] = (val & m) ? 9400u : 0u;
        }
    }
}

static inline void update_nmi(Ricoh2C02 *ppu) {
    bool active = ((ppu->ctrl & RICOH_2C02_CTRL_NMI) != 0u) &&
                  ((ppu->status & RICOH_2C02_STATUS_VBL) != 0u);
    if (active != ppu->nmi_output) {
        ppu->nmi_output = active;
        if (ppu->bus.on_nmi != NULL) {
            ppu->bus.on_nmi(ppu->bus.ctx, active);
        }
    }
}

/* ==================== 3. Loopy Scroll & OAM Corruption ==================== */

static inline void inc_coarse_x(Ricoh2C02 *ppu) {
    if ((ppu->v & 0x001Fu) == 31u) {
        ppu->v &= (uint16_t)(~0x001Fu);
        ppu->v ^= 0x0400u;
    } else {
        ppu->v++;
    }
}

static inline void inc_vert_y(Ricoh2C02 *ppu) {
    if ((ppu->v & 0x7000u) != 0x7000u) {
        ppu->v += 0x1000u;
    } else {
        ppu->v &= (uint16_t)(~0x7000u);
        uint16_t coarse_y = (uint16_t)((ppu->v & 0x03E0u) >> 5);
        if (coarse_y == 29u) {
            coarse_y = 0u;
            ppu->v ^= 0x0800u;
        } else if (coarse_y == 31u) {
            coarse_y = 0u;
        } else {
            coarse_y++;
        }
        ppu->v = (uint16_t)((ppu->v & (uint16_t)(~0x03E0u)) | (coarse_y << 5));
    }
}

static inline void copy_hori_v_from_t(Ricoh2C02 *ppu) {
    ppu->v = (uint16_t)((ppu->v & (uint16_t)(~0x041Fu)) | (ppu->t & 0x041Fu));
}

static inline void copy_vert_v_from_t(Ricoh2C02 *ppu) {
    ppu->v = (uint16_t)((ppu->v & (uint16_t)(~0x7BE0u)) | (ppu->t & 0x7BE0u));
}

static inline void oamc_record(Ricoh2C02 *ppu, bool set_index) {
    ppu->oamc_disabled     = false;
    ppu->oamc_disabled_now = false;
    ppu->oamc_pending      = true;
    if (set_index) {
        ppu->oamc_index = ppu->oam2_addr;
    }
}

static void oamc_corrupt(Ricoh2C02 *ppu) {
    uint8_t row = (uint8_t)(ppu->oamc_index & 0x1Fu);
    if (row > 0u) {
        memcpy(&ppu->oam[row * 8u], &ppu->oam[0], 8u);
    }
    ppu->secondary_oam[row] = ppu->secondary_oam[0];
}

/* ==================== 4. $2007 Data State Machine & VRAM Bus ==================== */

static uint8_t vram_fetch(Ricoh2C02 *ppu) {
    uint16_t addr;
    if (ppu->vbus & 0x2000u) {
        addr = (uint16_t)((ppu->vbus & 0x3F00u) | ppu->octal_latch);
    } else {
        addr = (uint16_t)(((ppu->vbus & 0x3F00u) | ppu->octal_latch) & 0x1FFFu);
    }

    uint8_t val = ppu->rd ? ppu_bus_read(ppu, addr) : (uint8_t)ppu->vbus;
    if (ppu->wr) {
        ppu_bus_write(ppu, addr, (uint8_t)ppu->vbus);
    }
    ppu->vbus = (uint16_t)((ppu->vbus & 0xFF00u) | val);
    return val;
}

static void vram_store(Ricoh2C02 *ppu, uint8_t val) {
    if ((ppu->vbus & 0x3FFFu) >= 0x3F00u) {
        ppu->palette_ram[palette_index(ppu->vbus)] = (uint8_t)(val & 0x3Fu);
    } else if (ppu->wr) {
        uint16_t addr = (uint16_t)((ppu->vbus & 0x3F00u) | ppu->octal_latch);
        ppu_bus_write(ppu, addr, val);
    }
}

static void data_sm_dot(Ricoh2C02 *ppu, bool blnk) {
    bool h0 = ((ppu->dot - 1u) & 1u) != 0u;
    if (ppu->sm_rest) {
        ppu->rd = !blnk && h0;
        ppu->wr = false;
        ppu->ale = !blnk && !h0;
        ppu->blnk_latch = blnk;
        return;
    }

    ppu->rl[0] = ppu->rd_sr;
    ppu->rl[2] = !ppu->rl[1];
    ppu->rl[4] = !ppu->rl[3];
    ppu->pd_rb  = ppu->rl[4] && !ppu->rl[2];
    ppu->rd_ale = !ppu->rl[4] && ppu->rl[2];

    ppu->wl[0] = ppu->wr_sr;
    ppu->wl[2] = !ppu->wl[1];
    ppu->wl[4] = !ppu->wl[3];
    ppu->wr_ale = !ppu->wl[4] && ppu->wl[2];
    ppu->tstep_latch = ppu->db_par;
    ppu->db_par = false;

    ppu->rd = ppu->pd_rb || (!blnk && h0);
    ppu->wr = false;
    ppu->ale = ppu->rd_ale || ppu->wr_ale || (!blnk && !h0);
    ppu->blnk_latch = blnk;

    if ((ppu->rd_ale || ppu->wr_ale) && !ppu->rd) {
        ppu->vbus = ppu->v;
        ppu->octal_latch = (uint8_t)ppu->vbus;
    }
}

static void data_sm_dot_end(Ricoh2C02 *ppu) {
    if (ppu->pd_rb) {
        ppu->read_buffer = vram_fetch(ppu);
        if (ppu->ale) {
            ppu->octal_latch = (uint8_t)ppu->vbus;
        }
    }
}

static void data_sm_half(Ricoh2C02 *ppu) {
    if (ppu->sm_rest) {
        return;
    }

    ppu->ale = ppu->rd_ale || ppu->wr_ale;

    if (ppu->tstep_latch || ppu->pd_rb) {
        if (!ppu->blnk_latch) {
            inc_coarse_x(ppu);
            inc_vert_y(ppu);
        } else {
            ppu->v = (uint16_t)((ppu->v + ((ppu->ctrl & RICOH_2C02_CTRL_INC32) ? 32u : 1u)) & 0x7FFFu);
        }
    }

    ppu->pal_enable = ((ppu->vbus & 0x3F00u) == 0x3F00u) && ppu->blnk_latch;

    ppu->rl[1] = !ppu->rl[0];
    ppu->rl[3] = !ppu->rl[2];
    if (!ppu->rl[3]) {
        ppu->rd_sr = false;
    }

    ppu->wl[1] = !ppu->wl[0];
    ppu->wl[3] = !ppu->wl[2];
    if (!ppu->wl[3]) {
        ppu->wr_sr = false;
    }

    ppu->db_par = ppu->wl[1] && !ppu->wl[3];
    ppu->wr = !ppu->pal_enable && ppu->db_par;

    if (ppu->db_par) {
        vram_store(ppu, ppu->write_data);
    }
    if (ppu->pd_rb) {
        ppu->read_buffer = vram_fetch(ppu);
        if (ppu->ale) {
            ppu->octal_latch = (uint8_t)ppu->vbus;
        }
    }

    ppu->sm_rest = ppu->rl[1] && ppu->rl[3] && ppu->wl[1] && ppu->wl[3] &&
                   !ppu->rd_sr && !ppu->wr_sr && !ppu->pd_rb && !ppu->tstep_latch;
}

/* ==================== 5. Background Fetch & Shifters ==================== */

static inline void load_bg_shifters(Ricoh2C02 *ppu) {
    ppu->bg_shift_pat_lo  = (uint16_t)((ppu->bg_shift_pat_lo & 0xFF00u) | ppu->bg_next_tile_lo);
    ppu->bg_shift_pat_hi  = (uint16_t)((ppu->bg_shift_pat_hi & 0xFF00u) | ppu->bg_next_tile_hi);
    ppu->bg_loaded_attr   = ppu->bg_next_tile_attr;
    ppu->bg_shift_attr_lo = (uint16_t)((ppu->bg_shift_attr_lo & 0xFF00u) |
                                       ((ppu->bg_loaded_attr & 0x01u) ? 0xFFu : 0x00u));
    ppu->bg_shift_attr_hi = (uint16_t)((ppu->bg_shift_attr_hi & 0xFF00u) |
                                       ((ppu->bg_loaded_attr & 0x02u) ? 0xFFu : 0x00u));
}

static inline void shift_bg_shifters(Ricoh2C02 *ppu) {
    ppu->bg_shift_pat_lo  = (uint16_t)(ppu->bg_shift_pat_lo << 1);
    ppu->bg_shift_pat_hi  = (uint16_t)((ppu->bg_shift_pat_hi << 1) | 1u);
    ppu->bg_shift_attr_lo = (uint16_t)((ppu->bg_shift_attr_lo << 1) | (ppu->bg_loaded_attr & 0x01u));
    ppu->bg_shift_attr_hi = (uint16_t)((ppu->bg_shift_attr_hi << 1) | ((ppu->bg_loaded_attr >> 1) & 0x01u));
}

static inline void par_context(Ricoh2C02 *ppu) {
    if (ppu->dot < 257u || ppu->dot > 320u) {
        uint16_t table = (ppu->ctrl & RICOH_2C02_CTRL_BG_TBL) ? 0x1000u : 0x0000u;
        uint16_t fine_y = (uint16_t)((ppu->v & 0x7000u) >> 12);
        ppu->par_chr = (uint16_t)((ppu->par_chr & 0x0FF8u) | table | fine_y);
    } else {
        bool flip = (ppu->spr_fetch_attr & 0x80u) != 0u;
        uint16_t row = (uint16_t)(ppu->spr_fetch_row & 7u);
        if (flip) {
            row = (uint16_t)(7u - row);
        }
        if ((ppu->ctrl & RICOH_2C02_CTRL_SPR_8X16) == 0u) {
            uint16_t table = (ppu->ctrl & RICOH_2C02_CTRL_SPR_TBL) ? 0x1000u : 0x0000u;
            ppu->par_chr = (uint16_t)((ppu->par_chr & 0x0FF8u) | table | row);
        } else {
            uint16_t table = (ppu->spr_fetch_tile & 1u) ? 0x1000u : 0x0000u;
            uint16_t hi_row = (uint16_t)(((ppu->spr_fetch_row & 8u) ^ (flip ? 8u : 0u)) << 1);
            ppu->par_chr = (uint16_t)((ppu->par_chr & 0x0FE8u) | table | row | hi_row);
        }
    }
}

static void step_bg_fetch_step(Ricoh2C02 *ppu, uint8_t step) {
    if (ppu->rd) {
        ppu->octal_latch = (uint8_t)ppu->vbus;
    }

    switch (step) {
        case 0:
            ppu->vbus = (uint16_t)(0x2000u | (ppu->v & 0x0FFFu));
            break;
        case 1:
            ppu->vbus = (uint16_t)(0x2000u | (ppu->v & 0x0F00u) | ppu->octal_latch);
            (void)vram_fetch(ppu);
            ppu->commit |= COMMIT_NT;
            break;
        case 2:
            ppu->vbus = (uint16_t)(0x23C0u |
                                   (ppu->v & 0x0C00u) |
                                   ((ppu->v >> 4) & 0x0038u) |
                                   ((ppu->v >> 2) & 0x0007u));
            break;
        case 3:
            ppu->vbus = (uint16_t)(0x2300u | (ppu->v & 0x0C00u) | ppu->octal_latch);
            ppu->fetch_data = vram_fetch(ppu);
            ppu->commit |= COMMIT_AT;
            break;
        case 4:
            par_context(ppu);
            ppu->par_chr &= 0x1FF7u;
            ppu->vbus = ppu->par_chr;
            break;
        case 5:
            ppu->vbus = (uint16_t)((ppu->par_chr & 0xFF00u) | ppu->octal_latch);
            ppu->fetch_data = vram_fetch(ppu);
            ppu->commit |= COMMIT_LO;
            break;
        case 6:
            par_context(ppu);
            ppu->par_chr |= 8u;
            ppu->vbus = ppu->par_chr;
            break;
        case 7:
            ppu->vbus = (uint16_t)((ppu->par_chr & 0xFF00u) | ppu->octal_latch);
            ppu->fetch_data = vram_fetch(ppu);
            ppu->commit |= COMMIT_HI;
            break;
    }

    if (ppu->ale && !ppu->rd) {
        ppu->octal_latch = (uint8_t)ppu->vbus;
    }
}

static void step_bg_fetch_tail(Ricoh2C02 *ppu, uint16_t dot) {
    if (ppu->rd) {
        ppu->octal_latch = (uint8_t)ppu->vbus;
    }

    if (dot >= 337u && dot <= 340u) {
        switch (dot - 337u) {
            case 0:
            case 2:
                ppu->vbus = (uint16_t)(0x2000u | (ppu->v & 0x0FFFu));
                break;
            case 1:
                ppu->vbus = (uint16_t)(0x2000u | (ppu->v & 0x0F00u) | ppu->octal_latch);
                (void)vram_fetch(ppu);
                ppu->commit |= COMMIT_NT;
                break;
            case 3:
                (void)vram_fetch(ppu);
                break;
        }
    } else if (dot == 0u) {
        par_context(ppu);
        ppu->par_chr &= 0x1FF7u;
        if (ppu->scanline != 261u) {
            ppu->vbus = ppu->par_chr;
        }
    }

    if (ppu->ale && !ppu->rd) {
        ppu->octal_latch = (uint8_t)ppu->vbus;
    }
}

static void bg_commit(Ricoh2C02 *ppu, uint16_t dot) {
    if (ppu->commit & COMMIT_NT) {
        uint8_t tile = (dot >= 257u && dot <= 320u) ? ppu->spr_fetch_tile : (uint8_t)ppu->vbus;
        ppu->par_chr = (uint16_t)((ppu->par_chr & 0x100Fu) | ((uint16_t)tile << 4));
    }
    if (ppu->commit & COMMIT_AT) {
        uint8_t shift = (uint8_t)(((ppu->v >> 4) & 4u) | (ppu->v & 2u));
        ppu->bg_next_tile_attr = (uint8_t)((ppu->fetch_data >> shift) & 0x03u);
    }
    if (ppu->commit & COMMIT_LO) {
        ppu->bg_next_tile_lo = ppu->fetch_data;
    }
    if (ppu->commit & COMMIT_HI) {
        ppu->bg_next_tile_hi = ppu->fetch_data;
        load_bg_shifters(ppu);
        inc_coarse_x(ppu);
    }
    ppu->commit = 0u;
}

/* ==================== 6. Sprite Evaluation & Sprite Fetch ==================== */

static inline void oam2_increment(Ricoh2C02 *ppu) {
    if (!ppu->oam2_overflow) {
        ppu->oam2_addr = (uint8_t)((ppu->oam2_addr + 1u) & 0x1Fu);
        if (ppu->oam2_addr == 0u) {
            ppu->oam2_overflow = true;
        }
    }
}

static void eval_clear(Ricoh2C02 *ppu, uint16_t dot, bool prerender) {
    bool eval = eval_rendering(ppu);

    if (dot & 1u) {
        if (!eval) {
            return;
        }
        ppu->oam_buffer_in = prerender ? ppu->secondary_oam[ppu->oam2_addr] : 0xFFu;
        if (dot == 1u) {
            ppu->oam2_addr = 0u;
            ppu->eval_tick = 0u;
            ppu->eval_done = false;
        }
        if (ppu->oamc_disabled || ppu->oamc_disabled_now) {
            oamc_record(ppu, true);
        }
    } else if (dot == 0u) {
        if (eval) {
            oam2_increment(ppu);
        }
    } else if (eval) {
        if (!prerender) {
            ppu->secondary_oam[ppu->oam2_addr] = ppu->oam_bus_latch;
        }
        if (ppu->oamc_disabled) {
            oamc_record(ppu, true);
        }
        oam2_increment(ppu);
        if (ppu->oamc_disabled_now && dot == 64u) {
            oamc_record(ppu, false);
        }
    } else if (ppu->oamc_disabled || ppu->oamc_disabled_now) {
        oamc_record(ppu, true);
    }
}

static void eval_objects(Ricoh2C02 *ppu, uint16_t dot, bool prerender) {
    uint8_t height = (ppu->ctrl & RICOH_2C02_CTRL_SPR_8X16) ? 16u : 8u;

    if (dot == 65u && eval_rendering(ppu)) {
        ppu->eval_nine = false;
        ppu->eval_odd_corrupt = false;
        ppu->eval_done = false;
    }

    if (!(ppu->instant_bg || ppu->instant_spr || ppu->oamc_disabled_now)) {
        return;
    }

    if (dot & 1u) {
        ppu->oam_buffer_in = ppu->oam[ppu->oam_addr];
        if (ppu->oamc_disabled_now) {
            ppu->oamc_disabled = false;
            if (!prerender) {
                ppu->oam_addr++;
            }
            ppu->eval_odd_corrupt = true;
        }
        return;
    }

    bool step = !ppu->eval_odd_corrupt && !prerender;

    if (!ppu->eval_done) {
        uint8_t before = ppu->oam_addr;
        if (!ppu->oam2_overflow && !prerender) {
            ppu->secondary_oam[ppu->oam2_addr] = ppu->oam_bus_latch;
        }
        uint8_t oam2_val = ppu->secondary_oam[ppu->oam2_addr];

        if (ppu->eval_tick == 0u) {
            ppu->spr_fetch_row = (uint16_t)((ppu->scanline & 0xFFu) - ppu->oam_bus_latch);
            if (!ppu->eval_nine && !prerender && ppu->spr_fetch_row < height) {
                if (!ppu->oam2_overflow) {
                    if (!ppu->eval_odd_corrupt) {
                        ppu->oam_addr++;
                        oam2_increment(ppu);
                    }
                    if (dot == 66u) {
                        ppu->sprite0_next = true;
                    }
                } else {
                    ppu->eval_nine = true;
                    ppu->oam_addr++;
                    ppu->status |= RICOH_2C02_STATUS_OVERFLOW;
                }
                ppu->eval_tick++;
            } else {
                if (dot == 66u) {
                    ppu->sprite0_next = false;
                }
                if (step) {
                    if (ppu->oam2_overflow && !ppu->eval_nine) {
                        ppu->oam_addr = (uint8_t)(ppu->oam_addr + (((ppu->oam_addr & 0x03u) == 0x03u) ? 1u : 5u));
                    } else {
                        ppu->oam_addr = (uint8_t)((ppu->oam_addr + 4u) & 0xFCu);
                    }
                }
            }
        } else {
            if (ppu->eval_tick == 3u) {
                int row = (int)ppu->scanline - (int)ppu->oam_bus_latch;
                if (row >= 0 && row < (int)height) {
                    if (step) {
                        ppu->oam_addr = (uint8_t)(ppu->oam_addr + (ppu->oam2_overflow ? 4u : 1u));
                    }
                } else if (!ppu->oam2_overflow) {
                    if (step) {
                        ppu->oam_addr = (uint8_t)((ppu->oam_addr + 1u) & 0xFCu);
                    }
                } else {
                    ppu->oam_addr = (uint8_t)((ppu->oam_addr + 1u) & 0xFCu);
                }
            } else if (step) {
                ppu->oam_addr++;
            }
            ppu->eval_tick = (uint8_t)((ppu->eval_tick + 1u) & 3u);
            if (!ppu->oam2_overflow && !prerender) {
                oam2_increment(ppu);
            }
        }

        ppu->eval_odd_corrupt = false;
        if (ppu->oam_addr < before && ppu->oam_addr < 4u) {
            ppu->eval_done = true;
        }
        ppu->oam_buffer_in = oam2_val;
    } else {
        if (step) {
            ppu->oam_addr = (uint8_t)((ppu->oam_addr + 4u) & 0xFCu);
        }
        ppu->eval_odd_corrupt = false;
        ppu->oam_buffer_in = ppu->secondary_oam[ppu->oam2_addr];
    }

    if (ppu->oamc_disabled_now) {
        ppu->oamc_disabled = false;
        ppu->oamc_pending = true;
        if ((ppu->oam2_addr & 0x03u) != 0u && !ppu->eval_done && !prerender) {
            ppu->oam2_addr = (uint8_t)((ppu->oam2_addr + 4u) & 0x1Cu);
            if (ppu->oam2_addr == 0u) {
                ppu->oam2_overflow = true;
            }
        }
        ppu->oamc_index = (dot == 256u)
            ? (ppu->eval_odd_corrupt ? 0u : 1u)
            : ppu->oam2_addr;
    }
    ppu->oamc_disabled_now = false;
}

static void eval_load(Ricoh2C02 *ppu, uint16_t dot) {
    bool eval = eval_rendering(ppu);

    if (dot == 257u) {
        ppu->sprite0_on_line = ppu->sprite0_next;
    }

    if (ppu->oamc_disabled) {
        oamc_record(ppu, true);
    }

    if (eval) {
        ppu->oam_addr = 0u;
    }

    if (ppu->rd) {
        ppu->octal_latch = (uint8_t)ppu->vbus;
    }

    uint8_t idx = (uint8_t)((dot - 257u) >> 3);
    uint8_t sub = (uint8_t)((dot - 257u) & 7u);

    ppu->oam_buffer_in = eval ? ppu->secondary_oam[ppu->oam2_addr] : ppu->oam[ppu->oam_addr];

    if (eval) {
        switch (sub) {
            case 0:
                ppu->vbus = (uint16_t)(0x2000u | (ppu->v & 0x0FFFu));
                oam2_increment(ppu);
                break;
            case 1:
                ppu->spr_fetch_row = (uint16_t)((ppu->scanline & 0xFFu) - ppu->oam_bus_latch);
                ppu->vbus = (uint16_t)(0x2000u | (ppu->v & 0x0F00u) | ppu->octal_latch);
                (void)vram_fetch(ppu);
                ppu->commit |= COMMIT_NT;
                oam2_increment(ppu);
                break;
            case 2:
                ppu->spr_fetch_tile = ppu->oam_bus_latch;
                ppu->vbus = (uint16_t)(0x2000u | (ppu->v & 0x0FFFu));
                oam2_increment(ppu);
                break;
            case 3:
                ppu->spr_attr[idx] = ppu->oam_bus_latch;
                ppu->spr_fetch_attr = ppu->oam_bus_latch;
                ppu->vbus = (uint16_t)(0x2000u | (ppu->v & 0x0F00u) | ppu->octal_latch);
                (void)vram_fetch(ppu);
                ppu->commit |= COMMIT_NT;
                break;
            case 4:
                ppu->spr_x[idx] = ppu->oam_bus_latch;
                par_context(ppu);
                ppu->par_chr &= 0x1FF7u;
                ppu->vbus = ppu->par_chr;
                break;
            case 5: {
                ppu->vbus = (uint16_t)((ppu->par_chr & 0xFF00u) | ppu->octal_latch);
                uint8_t lo = vram_fetch(ppu);
                uint8_t height = (ppu->ctrl & RICOH_2C02_CTRL_SPR_8X16) ? 16u : 8u;
                if (ppu->spr_fetch_row >= height) {
                    lo = 0x00u;
                } else if (ppu->spr_attr[idx] & 0x40u) {
                    lo = reverse_bits(lo);
                }
                ppu->spr_pat_lo[idx] = lo;
                break;
            }
            case 6:
                par_context(ppu);
                ppu->par_chr |= 8u;
                ppu->vbus = ppu->par_chr;
                break;
            case 7: {
                ppu->vbus = (uint16_t)((ppu->par_chr & 0xFF00u) | ppu->octal_latch);
                uint8_t hi = vram_fetch(ppu);
                uint8_t height = (ppu->ctrl & RICOH_2C02_CTRL_SPR_8X16) ? 16u : 8u;
                if (ppu->spr_fetch_row >= height) {
                    hi = 0x00u;
                } else if (ppu->spr_attr[idx] & 0x40u) {
                    hi = reverse_bits(hi);
                }
                ppu->spr_pat_hi[idx] = hi;
                oam2_increment(ppu);
                break;
            }
        }
    }

    if (ppu->ale && !ppu->rd) {
        ppu->octal_latch = (uint8_t)ppu->vbus;
    }
}

/* ==================== 7. Pixel Output ==================== */

static void compute_pixel(Ricoh2C02 *ppu, uint16_t dot) {
    uint8_t color = 0u;
    uint8_t pal = 0u;

    if (ppu->show_bg && (dot > 8u || ppu->show_bg8)) {
        uint16_t mux = (uint16_t)(0x8000u >> ppu->x);
        uint8_t p0 = (ppu->bg_shift_pat_lo & mux) ? 1u : 0u;
        uint8_t p1 = (ppu->bg_shift_pat_hi & mux) ? 2u : 0u;
        uint8_t a0 = (ppu->bg_shift_attr_lo & mux) ? 1u : 0u;
        uint8_t a1 = (ppu->bg_shift_attr_hi & mux) ? 2u : 0u;
        color = (uint8_t)(p0 | p1);
        pal = (uint8_t)(a0 | a1);
    }

    if (ppu->show_spr && (dot > 8u || ppu->show_spr8)) {
        for (uint8_t i = 0u; i < 8u; i++) {
            if (ppu->spr_x[i] != 0u && !ppu->skipped_dot) {
                continue;
            }
            uint8_t p0 = (ppu->spr_pat_lo[i] & 0x80u) ? 1u : 0u;
            uint8_t p1 = (ppu->spr_pat_hi[i] & 0x80u) ? 2u : 0u;
            uint8_t sc = (uint8_t)(p0 | p1);
            if (sc != 0u) {
                if (i == 0u && ppu->can_s0hit && ppu->sprite0_on_line &&
                    ppu->show_bg && color != 0u && dot < 256u) {
                    ppu->s0hit_pending1 = true;
                    ppu->can_s0hit = false;
                }
                if (color == 0u || (ppu->spr_attr[i] & 0x20u) == 0u) {
                    color = sc;
                    pal = (uint8_t)(0x04u | (ppu->spr_attr[i] & 0x03u));
                }
                break;
            }
        }
    }

    uint8_t pal_idx;
    if (color != 0u) {
        pal_idx = (uint8_t)((pal << 2) | color);
    } else if (!rendering_now(ppu) && (ppu->v & 0x3FFFu) >= 0x3F00u) {
        pal_idx = palette_index(ppu->v);
    } else {
        pal_idx = 0u;
    }

    ppu->color_pipe[0] = (uint8_t)(ppu->palette_ram[pal_idx] & 0x3Fu);
}

static void shift_sprites(Ricoh2C02 *ppu) {
    bool ren = rendering_now(ppu);
    for (uint8_t i = 0u; i < 8u; i++) {
        if (ppu->spr_x[i] > 0u && !ppu->skipped_dot) {
            ppu->spr_x[i]--;
        } else if (ren) {
            ppu->spr_pat_lo[i] <<= 1;
            ppu->spr_pat_hi[i] <<= 1;
        }
    }
}

static void output_pixel(Ricoh2C02 *ppu, uint16_t dot, uint16_t sl) {
    uint8_t color = ppu->color_pipe[3];
    if (ppu->greyscale) {
        color &= 0x30u;
    }
    uint16_t raw = (uint16_t)(((uint16_t)ppu->emphasis << 6) | color);
    size_t offset = (size_t)sl * RICOH_2C02_WIDTH + (size_t)(dot - 4u);
    ppu->raw_frame[offset] = raw;
    ppu->framebuffer[offset] = ppu->palette_lut[raw];
}

/* ==================== 8. Lifecycle, Step & MMIO ==================== */

static void sm_reset_rest(Ricoh2C02 *ppu) {
    ppu->rd_sr = false;
    ppu->wr_sr = false;
    ppu->wr_sr_delay = 0u;
    ppu->rl[0] = false; ppu->rl[1] = true; ppu->rl[2] = false; ppu->rl[3] = true; ppu->rl[4] = false;
    ppu->wl[0] = false; ppu->wl[1] = true; ppu->wl[2] = false; ppu->wl[3] = true; ppu->wl[4] = false;
    ppu->pd_rb = false;
    ppu->rd_ale = false;
    ppu->wr_ale = false;
    ppu->db_par = false;
    ppu->tstep_latch = false;
    ppu->rd = false;
    ppu->wr = false;
    ppu->ale = false;
    ppu->pal_enable = false;
    ppu->sm_rest = true;
}

void ricoh_2c02_power_on(Ricoh2C02 *ppu, const Ricoh2C02Bus *bus) {
    memset(ppu, 0, sizeof(*ppu));
    if (bus != NULL) {
        ppu->bus = *bus;
    }
    memset(ppu->oam, 0xFF, sizeof(ppu->oam));
    memset(ppu->secondary_oam, 0xFF, sizeof(ppu->secondary_oam));
    ppu->odd_frame = true;
    sm_reset_rest(ppu);
    build_palette_lut(ppu->palette_lut);
}

void ricoh_2c02_reset(Ricoh2C02 *ppu) {
    ppu->ctrl = 0u;
    ppu->mask = 0u;
    ppu->w = false;
    ppu->x = 0u;
    ppu->v_delay = 0u;
    ppu->copy_v = false;
    ppu->show_bg8 = false;
    ppu->show_spr8 = false;
    ppu->show_bg = false;
    ppu->show_spr = false;
    ppu->instant_bg = false;
    ppu->instant_spr = false;
    ppu->eval_bg = false;
    ppu->eval_spr = false;
    ppu->greyscale = false;
    ppu->emphasis = 0u;
    ppu->w2001_delay = 0u;
    ppu->w2001_oam_delay = 0u;
    ppu->w2001_emph_delay = 0u;
    ppu->render_count = 0u;
    ppu->read_buffer = 0u;
    ppu->io_latch = 0u;
    memset(ppu->io_decay, 0, sizeof(ppu->io_decay));
    ppu->status &= RICOH_2C02_STATUS_VBL;
    ppu->scanline = 0u;
    ppu->dot = 0u;
    ppu->odd_frame = true;
    ppu->skipped_dot = false;
    ppu->vblank_pending = false;
    ppu->vset = false;
    ppu->vset_latch1 = false;
    ppu->vset_latch2 = false;
    ppu->read2002 = false;
    ppu->can_s0hit = false;
    ppu->s0hit_pending1 = false;
    ppu->s0hit_pending2 = false;
    ppu->s0hit_flag = false;
    ppu->s0hit_late = false;
    ppu->overflow_late = false;
    ppu->oamc_disabled = false;
    ppu->oamc_disabled_now = false;
    ppu->oamc_pending = false;
    ppu->oamc_index = 0u;
    ppu->oam2_reset = 0u;
    ppu->commit = 0u;
    ppu->bg_shift_pat_lo = 0u;
    ppu->bg_shift_pat_hi = 0u;
    ppu->bg_shift_attr_lo = 0u;
    ppu->bg_shift_attr_hi = 0u;
    memset(ppu->color_pipe, 0, sizeof(ppu->color_pipe));
    sm_reset_rest(ppu);
    update_nmi(ppu);
}

void ricoh_2c02_step(Ricoh2C02 *ppu) {
    uint16_t sl = ppu->scanline;
    uint16_t dot = ppu->dot;
    bool visible_line = (sl < 240u);
    bool prerender_line = (sl == 261u);
    bool active_line = visible_line || prerender_line;

    if (dot == 0u) {
        for (uint8_t b = 0u; b < 8u; b++) {
            if (ppu->io_decay[b] > 0u) {
                ppu->io_decay[b]--;
                if (ppu->io_decay[b] == 0u) {
                    ppu->io_latch &= (uint8_t)(~(1u << b));
                }
            }
        }
    }

    if (ppu->skipped_dot && dot == 2u) {
        ppu->skipped_dot = false;
    }

    if (sl == 260u && dot == 340u) {
        ppu->odd_frame = !ppu->odd_frame;
    }

    /* Cap nhat VBlank / clear dau dot theo dung latch VSET + read2002 */
    if (sl == 241u && dot == 0u) {
        ppu->vblank_pending = true;
    } else if (prerender_line && dot == 1u) {
        ppu->status &= (uint8_t)(~(RICOH_2C02_STATUS_VBL |
                                   RICOH_2C02_STATUS_SPRITE0 |
                                   RICOH_2C02_STATUS_OVERFLOW));
        ppu->can_s0hit = true;
        ppu->s0hit_flag = false;
        ppu->s0hit_pending1 = false;
        ppu->s0hit_pending2 = false;
        ppu->s0hit_late = false;
    }

    ppu->vset_latch1 = !ppu->vset;
    if (ppu->vset && !ppu->vset_latch2) {
        ppu->status |= RICOH_2C02_STATUS_VBL;
    }
    if (ppu->read2002) {
        ppu->read2002 = false;
        ppu->status &= (uint8_t)(~RICOH_2C02_STATUS_VBL);
    }
    ppu->overflow_late = (ppu->status & RICOH_2C02_STATUS_OVERFLOW) != 0u;
    update_nmi(ppu);

    ppu->eval_bg = ppu->show_bg;
    ppu->eval_spr = ppu->show_spr;

    bool blnk_now = !rendering_now(ppu) || (sl >= 240u && sl < 261u);
    data_sm_dot(ppu, blnk_now);
    ppu->oam_latch = ppu->oam_bus_latch;

    ppu->copy_v = false;
    if (ppu->v_delay > 0u && --ppu->v_delay == 0u) {
        ppu->w2006_old_v = ppu->v;
        ppu->copy_v = true;
        ppu->v = ppu->t;
        ppu->vbus = ppu->v;
    }

    if (ppu->oam2_reset > 0u) {
        ppu->oam2_reset--;
    }

    if (active_line) {
        if ((ppu->instant_bg || ppu->instant_spr) && ppu->oamc_pending) {
            ppu->oamc_pending = false;
            oamc_corrupt(ppu);
        }

        if (dot <= 64u) {
            eval_clear(ppu, dot, prerender_line);
        } else if (dot <= 256u) {
            eval_objects(ppu, dot, prerender_line);
        } else if (dot <= 320u) {
            eval_load(ppu, dot);
        } else {
            if (ppu->oamc_disabled || ppu->oamc_disabled_now) {
                oamc_record(ppu, true);
            }
            ppu->oam_buffer_in = rendering_now(ppu) ? ppu->secondary_oam[ppu->oam2_addr]
                                                    : ppu->oam[ppu->oam_addr];
            if (dot == 339u && !rendering_now(ppu)) {
                memset(ppu->spr_x, 0, sizeof(ppu->spr_x));
            }
        }

        if (eval_rendering(ppu) && (dot == 63u || dot == 255u || dot == 339u)) {
            ppu->oam2_reset = 3u;
        }
    }

    if (!rendering_now(ppu)) {
        ppu->render_count = 0u;
        if (ppu->sm_rest) {
            ppu->vbus = ppu->v;
        }
    } else {
        if (ppu->render_count < 5u) {
            ppu->render_count++;
        }
        if (blnk_now && !(sl == 240u && dot == 0u) && ppu->sm_rest) {
            ppu->vbus = ppu->v;
        }
    }

    if (ppu->w2001_delay > 0u && --ppu->w2001_delay == 0u) {
        ppu->show_bg8    = (ppu->w2001_value & RICOH_2C02_MASK_BG_LEFT) != 0u;
        ppu->show_spr8   = (ppu->w2001_value & RICOH_2C02_MASK_SPR_LEFT) != 0u;
        ppu->show_bg     = (ppu->w2001_value & RICOH_2C02_MASK_BG) != 0u;
        ppu->show_spr    = (ppu->w2001_value & RICOH_2C02_MASK_SPR) != 0u;
        ppu->instant_bg  = ppu->show_bg;
        ppu->instant_spr = ppu->show_spr;
    }
    if (ppu->w2001_oam_delay > 0u && --ppu->w2001_oam_delay == 0u) {
        ppu->oamc_disabled_now = false;
        if (ppu->w2001_was_rendering &&
            (ppu->w2001_value & (RICOH_2C02_MASK_BG | RICOH_2C02_MASK_SPR)) == 0u &&
            active_line && !ppu->oamc_pending) {
            ppu->oamc_disabled = true;
        }
    }
    if (ppu->w2001_emph_delay > 0u && --ppu->w2001_emph_delay == 0u) {
        ppu->greyscale = (ppu->w2001_value & RICOH_2C02_MASK_GREYSCALE) != 0u;
        ppu->emphasis  = (uint8_t)(ppu->w2001_value >> 5);
    }

    ppu->color_pipe[3] = ppu->color_pipe[2];
    ppu->color_pipe[2] = ppu->color_pipe[1];
    ppu->color_pipe[1] = ppu->color_pipe[0];

    if (active_line || (sl == 240u && dot == 0u)) {
        if ((dot >= 1u && dot <= 256u) || (dot >= 321u && dot <= 336u)) {
            if (eval_rendering(ppu)) {
                step_bg_fetch_step(ppu, (uint8_t)((dot - 1u) & 7u));
            }
        } else {
            if (eval_rendering(ppu)) {
                step_bg_fetch_tail(ppu, dot);
            }
        }

        if (dot >= 1u && dot <= 256u) {
            if (visible_line) {
                compute_pixel(ppu, dot);
            }
            shift_sprites(ppu);
        }
        if (visible_line && dot > 3u && dot <= 259u) {
            output_pixel(ppu, dot, sl);
        }
    }

    data_sm_dot_end(ppu);

    /* Nua sau cua dot (half_dot) */
    if (active_line && rendering_now(ppu)) {
        if ((dot >= 1u && dot <= 257u) || (dot >= 321u && dot <= 336u)) {
            shift_bg_shifters(ppu);
        }
    }

    if (ppu->oam2_reset > 0u && --ppu->oam2_reset == 0u) {
        ppu->oam2_addr = 0u;
        ppu->oam2_overflow = false;
    }

    if (ppu->commit != 0u) {
        bg_commit(ppu, dot);
    }

    if (eval_rendering(ppu) || rendering_now(ppu)) {
        ppu->oam_bus_latch = ppu->oam_buffer_in;
    }

    ppu->vset = ppu->vblank_pending;
    ppu->vblank_pending = false;
    ppu->vset_latch2 = !ppu->vset_latch1;

    ppu->s0hit_late = ppu->s0hit_flag;
    if (ppu->s0hit_pending2) {
        ppu->s0hit_pending2 = false;
        ppu->s0hit_flag = true;
        ppu->status |= RICOH_2C02_STATUS_SPRITE0;
    }
    if (ppu->s0hit_pending1) {
        ppu->s0hit_pending1 = false;
        ppu->s0hit_pending2 = true;
    }

    data_sm_half(ppu);

    if (ppu->wr_sr_delay > 0u && --ppu->wr_sr_delay == 0u) {
        ppu->wr_sr = true;
        ppu->sm_rest = false;
    }

    if (active_line && rendering_now(ppu) && ppu->render_count >= 1u) {
        if (dot == 256u) {
            if (ppu->copy_v) {
                ppu->v = (uint16_t)(ppu->w2006_old_v & ppu->t);
            } else {
                inc_vert_y(ppu);
            }
        } else if (dot == 257u) {
            copy_hori_v_from_t(ppu);
        }
        if (prerender_line && dot >= 280u && dot <= 304u) {
            copy_vert_v_from_t(ppu);
        }
    }

    ppu->total_dots++;
    if (dot == 340u) {
        if (prerender_line) {
            ppu->scanline = 0u;
            ppu->frame_count++;
            if (ppu->odd_frame && rendering_now(ppu)) {
                ppu->dot = 1u;
                ppu->skipped_dot = true;
            } else {
                ppu->dot = 0u;
            }
        } else {
            ppu->dot = 0u;
            ppu->scanline++;
            if (ppu->scanline == 240u) {
                ppu->frame_ready = true;
            }
        }
    } else {
        ppu->dot++;
    }
}

void ricoh_2c02_tick_cpu_cycle(Ricoh2C02 *ppu) {
    ricoh_2c02_step(ppu);
    ricoh_2c02_step(ppu);
    ricoh_2c02_step(ppu);
}

void ricoh_2c02_read_begin(Ricoh2C02 *ppu, uint16_t addr, uint8_t *out_pre) {
    uint8_t reg = (uint8_t)(addr & 0x07u);
    switch (reg) {
        case 2:
            *out_pre = (ppu->status & RICOH_2C02_STATUS_VBL) ? 0x80u : 0x00u;
            ppu->read2002 = true;
            break;
        case 7:
            if ((ppu->vbus & 0x3FFFu) >= 0x3F00u) {
                uint8_t pal = ppu->palette_ram[palette_index(ppu->v)];
                if (ppu->greyscale) {
                    pal &= 0x30u;
                }
                *out_pre = (uint8_t)((ppu->io_latch & 0xC0u) | (pal & 0x3Fu));
            } else {
                *out_pre = ppu->read_buffer;
            }
            break;
        default:
            *out_pre = ppu->io_latch;
            break;
    }
}

uint8_t ricoh_2c02_read_end(Ricoh2C02 *ppu, uint16_t addr, uint8_t pre_val) {
    uint8_t reg = (uint8_t)(addr & 0x07u);
    uint8_t res = ppu->io_latch;

    switch (reg) {
        case 2:
            res = (uint8_t)((pre_val & 0x80u) |
                            (ppu->s0hit_late ? RICOH_2C02_STATUS_SPRITE0 : 0u) |
                            (ppu->overflow_late ? RICOH_2C02_STATUS_OVERFLOW : 0u) |
                            (ppu->io_latch & 0x1Fu));
            ppu->w = false;
            refresh_io_latch(ppu, res, 0xE0u);
            break;
        case 4:
            if (rendering_now(ppu) && (ppu->scanline < 240u)) {
                res = ppu->oam_latch;
            } else {
                res = ppu->oam[ppu->oam_addr];
                if ((ppu->oam_addr & 0x03u) == 2u) {
                    res &= 0xE3u;
                }
            }
            refresh_io_latch(ppu, res, 0xFFu);
            break;
        case 7:
            res = pre_val;
            refresh_io_latch(ppu, res, ((ppu->vbus & 0x3FFFu) >= 0x3F00u) ? 0x3Fu : 0xFFu);
            ppu->rd_sr = true;
            ppu->sm_rest = false;
            break;
        default:
            break;
    }

    return res;
}

uint8_t ricoh_2c02_read(Ricoh2C02 *ppu, uint16_t addr) {
    uint8_t pre_val = 0u;
    ricoh_2c02_read_begin(ppu, addr, &pre_val);
    return ricoh_2c02_read_end(ppu, addr, pre_val);
}

void ricoh_2c02_write(Ricoh2C02 *ppu, uint16_t addr, uint8_t val) {
    uint8_t reg = (uint8_t)(addr & 0x07u);
    refresh_io_latch(ppu, val, 0xFFu);

    switch (reg) {
        case 0: /* $2000 PPUCTRL */
            ppu->ctrl = val;
            ppu->t = (uint16_t)((ppu->t & (uint16_t)(~0x0C00u)) | ((uint16_t)(val & 0x03u) << 10));
            update_nmi(ppu);
            break;

        case 1: { /* $2001 PPUMASK */
            bool was = rendering_now(ppu);
            bool now = (val & (RICOH_2C02_MASK_BG | RICOH_2C02_MASK_SPR)) != 0u;
            ppu->mask = val;
            ppu->w2001_value = val;
            ppu->w2001_delay = 2u;
            ppu->w2001_oam_delay = 2u;
            ppu->w2001_emph_delay = 2u;
            ppu->w2001_was_rendering = was;
            ppu->instant_bg = ppu->show_bg;
            ppu->instant_spr = ppu->show_spr;
            if (was && !now && (ppu->scanline < 240u || ppu->scanline == 261u) && !ppu->oamc_pending) {
                ppu->oamc_disabled_now = true;
            }
            break;
        }

        case 3: /* $2003 OAMADDR */
            ppu->oam_addr = val;
            break;

        case 4: /* $2004 OAMDATA */
            if (rendering_now(ppu) && (ppu->scanline < 240u || ppu->scanline == 261u)) {
                ppu->oam_addr = (uint8_t)((ppu->oam_addr + 4u) & 0xFCu);
            } else {
                uint8_t stored = ((ppu->oam_addr & 0x03u) == 2u) ? (uint8_t)(val & 0xE3u) : val;
                ppu->oam[ppu->oam_addr] = stored;
                ppu->oam_addr++;
            }
            break;

        case 5: /* $2005 PPUSCROLL */
            if (!ppu->w) {
                ppu->t = (uint16_t)((ppu->t & (uint16_t)(~0x001Fu)) | (val >> 3));
                ppu->x = (uint8_t)(val & 0x07u);
                ppu->w = true;
            } else {
                ppu->t = (uint16_t)((ppu->t & (uint16_t)(~0x73E0u)) |
                                    ((uint16_t)(val & 0x07u) << 12) |
                                    ((uint16_t)(val & 0xF8u) << 2));
                ppu->w = false;
            }
            break;

        case 6: /* $2006 PPUADDR */
            if (!ppu->w) {
                ppu->t = (uint16_t)((ppu->t & (uint16_t)(~0x7F00u)) | ((uint16_t)(val & 0x3Fu) << 8));
                ppu->w = true;
            } else {
                ppu->t = (uint16_t)((ppu->t & (uint16_t)(~0x00FFu)) | val);
                ppu->v_delay = 4u;
                ppu->w = false;
            }
            break;

        case 7: /* $2007 PPUDATA */
            ppu->write_data = val;
            ppu->wr_sr_delay = 1u;
            break;

        default:
            break;
    }
}