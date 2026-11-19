#ifndef RICOH_2C02_H
#define RICOH_2C02_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RICOH_2C02_WIDTH         256u
#define RICOH_2C02_HEIGHT        240u
#define RICOH_2C02_DOTS_PER_LINE 341u
#define RICOH_2C02_SCANLINES     262u

#define RICOH_2C02_CTRL_NT_MASK   0x03u
#define RICOH_2C02_CTRL_INC32     0x04u
#define RICOH_2C02_CTRL_SPR_TBL   0x08u
#define RICOH_2C02_CTRL_BG_TBL    0x10u
#define RICOH_2C02_CTRL_SPR_8X16  0x20u
#define RICOH_2C02_CTRL_SLAVE_EXT 0x40u
#define RICOH_2C02_CTRL_NMI       0x80u

#define RICOH_2C02_MASK_GREYSCALE 0x01u
#define RICOH_2C02_MASK_BG_LEFT   0x02u
#define RICOH_2C02_MASK_SPR_LEFT  0x04u
#define RICOH_2C02_MASK_BG        0x08u
#define RICOH_2C02_MASK_SPR       0x10u
#define RICOH_2C02_MASK_EMPH_R    0x20u
#define RICOH_2C02_MASK_EMPH_G    0x40u
#define RICOH_2C02_MASK_EMPH_B    0x80u

#define RICOH_2C02_STATUS_OVERFLOW 0x20u
#define RICOH_2C02_STATUS_SPRITE0  0x40u
#define RICOH_2C02_STATUS_VBL      0x80u

typedef struct Ricoh2C02Bus {
    void *ctx;
    uint8_t (*read)(void *ctx, uint16_t addr);
    void (*write)(void *ctx, uint16_t addr, uint8_t val);
    void (*on_nmi)(void *ctx, bool asserted);
} Ricoh2C02Bus;

typedef struct Ricoh2C02 {
    /* Thanh ghi CPU */
    uint8_t  ctrl;
    uint8_t  mask;
    uint8_t  status;
    uint8_t  oam_addr;
    uint8_t  io_latch;
    uint16_t io_decay[8];
    uint8_t  read_buffer;

    /* Thanh ghi cuon Loopy & tre $2006 */
    uint16_t v;
    uint16_t t;
    uint8_t  x;
    bool     w;
    uint8_t  v_delay;
    uint16_t w2006_old_v;
    bool     copy_v;

    /* Tre 3 phan cua $2001 (rendering, eval, emphasis) */
    bool     show_bg8;
    bool     show_spr8;
    bool     show_bg;
    bool     show_spr;
    bool     instant_bg;
    bool     instant_spr;
    bool     eval_bg;
    bool     eval_spr;
    bool     greyscale;
    uint8_t  emphasis;
    uint8_t  w2001_delay;
    uint8_t  w2001_oam_delay;
    uint8_t  w2001_emph_delay;
    uint8_t  w2001_value;
    bool     w2001_was_rendering;
    uint8_t  render_count;

    /* Quet khung hinh & VBlank / Status latches */
    uint16_t scanline;
    uint16_t dot;
    uint64_t total_dots;
    uint64_t frame_count;
    bool     odd_frame;
    bool     skipped_dot;
    bool     frame_ready;
    bool     nmi_output;
    bool     vblank_pending;
    bool     vset;
    bool     vset_latch1;
    bool     vset_latch2;
    bool     read2002;
    bool     can_s0hit;
    bool     s0hit_pending1;
    bool     s0hit_pending2;
    bool     s0hit_flag;
    bool     s0hit_late;
    bool     overflow_late;

    /* OAM corruption */
    bool     oamc_disabled;
    bool     oamc_disabled_now;
    bool     oamc_pending;
    uint8_t  oamc_index;

    /* $2007 SR-latch data state machine & VRAM bus */
    uint16_t vbus;
    uint16_t par_chr;
    uint8_t  octal_latch;
    uint8_t  fetch_data;
    uint8_t  write_data;
    uint8_t  commit;
    bool     rd_sr;
    bool     wr_sr;
    uint8_t  wr_sr_delay;
    bool     rl[5];
    bool     wl[5];
    bool     pd_rb;
    bool     rd_ale;
    bool     wr_ale;
    bool     db_par;
    bool     tstep_latch;
    bool     rd;
    bool     wr;
    bool     ale;
    bool     blnk_latch;
    bool     pal_enable;
    bool     sm_rest;

    /* Bo nho noi bo */
    uint8_t  palette_ram[32];
    uint8_t  oam[256];
    uint8_t  secondary_oam[32];

    /* Background pipeline */
    uint8_t  bg_next_tile_attr;
    uint8_t  bg_loaded_attr;
    uint8_t  bg_next_tile_lo;
    uint8_t  bg_next_tile_hi;
    uint16_t bg_shift_pat_lo;
    uint16_t bg_shift_pat_hi;
    uint16_t bg_shift_attr_lo;
    uint16_t bg_shift_attr_hi;

    /* Sprite evaluation & OAM2 state machine */
    uint8_t  oam2_addr;
    uint8_t  oam2_reset;
    uint8_t  eval_tick;
    uint8_t  oam_bus_latch;
    uint8_t  oam_buffer_in;
    uint8_t  oam_latch;
    bool     oam2_overflow;
    bool     eval_done;
    bool     eval_nine;
    bool     eval_odd_corrupt;
    bool     sprite0_next;
    bool     sprite0_on_line;

    /* 8 don vi xuat Sprite */
    uint8_t  spr_pat_lo[8];
    uint8_t  spr_pat_hi[8];
    uint8_t  spr_attr[8];
    uint8_t  spr_x[8];
    uint16_t spr_fetch_row;
    uint8_t  spr_fetch_tile;
    uint8_t  spr_fetch_attr;

    /* Video & Palette */
    uint8_t  color_pipe[4];
    uint16_t raw_frame[RICOH_2C02_WIDTH * RICOH_2C02_HEIGHT];
    uint32_t framebuffer[RICOH_2C02_WIDTH * RICOH_2C02_HEIGHT];
    uint32_t palette_lut[512];

    Ricoh2C02Bus bus;
} Ricoh2C02;

void ricoh_2c02_power_on(Ricoh2C02 *ppu, const Ricoh2C02Bus *bus);
void ricoh_2c02_reset(Ricoh2C02 *ppu);
void ricoh_2c02_step(Ricoh2C02 *ppu);
void ricoh_2c02_tick_cpu_cycle(Ricoh2C02 *ppu);
void ricoh_2c02_read_begin(Ricoh2C02 *ppu, uint16_t addr, uint8_t *out_pre);
uint8_t ricoh_2c02_read_end(Ricoh2C02 *ppu, uint16_t addr, uint8_t pre_val);
uint8_t ricoh_2c02_read(Ricoh2C02 *ppu, uint16_t addr);
void ricoh_2c02_write(Ricoh2C02 *ppu, uint16_t addr, uint8_t val);

#ifdef __cplusplus
}
#endif

#endif /* RICOH_2C02_H */