#ifndef NINTENDO_MMC5_H
#define NINTENDO_MMC5_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MMC5_EXRAM_SIZE        1024u
#define MMC5_MAX_PRG_RAM_SIZE  131072u

/* IRQ source masks compatible with Ricoh2A03 irq_line */
#define MMC5_IRQ_SCANLINE      0x04u
#define MMC5_IRQ_PCM           0x08u
#define MMC5_IRQ_TIMER         0x10u

typedef struct Mmc5Bus {
    void *ctx;
    void (*on_irq)(void *ctx, uint8_t source_mask, bool asserted);
} Mmc5Bus;

typedef struct Mmc5Config {
    const uint8_t *prg_rom;
    uint32_t       prg_size;
    uint8_t       *chr_mem;
    uint32_t       chr_size;
    bool           chr_is_ram;
    uint32_t       prg_ram_size;
    uint32_t       prg_nvram_size;
    bool           is_nes2;
    bool           is_mmc5a;
    bool           sl_mode;
} Mmc5Config;

typedef struct Mmc5Envelope {
    bool    start;
    bool    loop;
    bool    constant_vol;
    uint8_t volume;
    uint8_t divider;
    uint8_t decay;
} Mmc5Envelope;

typedef struct Mmc5Pulse {
    uint8_t      duty_mode;
    uint8_t      duty_step;
    uint16_t     period;
    uint16_t     timer;
    Mmc5Envelope env;
    uint8_t      lc_count;
    bool         lc_halt;
    bool         lc_next_halt;
    bool         lc_enabled;
} Mmc5Pulse;

typedef struct Mmc5 {
    /* Cartridge memory & board wiring */
    const uint8_t *prg_rom;
    uint32_t       prg_size;
    uint8_t       *chr_mem;
    uint32_t       chr_size;
    bool           chr_is_ram;
    uint8_t        prg_ram[MMC5_MAX_PRG_RAM_SIZE];
    uint32_t       prg_ram_chip0_size;
    uint32_t       prg_ram_chip1_size;
    bool           prg_ram_is_128k;
    bool           is_mmc5a;
    bool           sl_mode;

    /* PRG banking & protection */
    uint8_t  prg_mode;           /* $5100 bits 1..0 */
    uint8_t  prg_ram_protect1;   /* $5102 bits 1..0 */
    uint8_t  prg_ram_protect2;   /* $5103 bits 1..0 */
    uint8_t  prg_banks[5];       /* $5113..$5117 */

    /* CHR banking */
    uint8_t  chr_mode;           /* $5101 bits 1..0 */
    uint8_t  chr_hi;             /* $5130 bits 1..0 */
    uint16_t chr_regs[12];       /* 10-bit latched regs $5120..$512B */
    bool     chr_last_written_b; /* true if $5128..$512B written more recently than $5120..$5127 */

    /* ExRAM & Nametable mapping */
    uint8_t  exram[MMC5_EXRAM_SIZE];
    uint8_t  exram_mode;         /* $5104 bits 1..0 */
    bool     exram_written_since_reset;
    uint8_t  exattr_latch;
    uint8_t  nt_mapping;         /* $5105 */
    uint8_t  fill_tile;          /* $5106 */
    uint8_t  fill_color;         /* $5107 bits 1..0 */

    /* Vertical split screen */
    uint8_t  vsplit_ctrl;        /* $5200 */
    uint8_t  vsplit_scroll;      /* $5201 */
    uint8_t  vsplit_chr_bank;    /* $5202 */
    uint16_t vsplit_y;
    bool     vsplit_active_tile;

    /* Hardware snooping & scanline IRQ */
    bool     sprite_8x16;        /* $2000 bit 5 */
    bool     ppu_subst_enabled;  /* $2001 bits 3 or 4 non-zero */
    bool     ppu_scroll_second;  /* $2005 2nd-write tracker */
    uint8_t  ppu_scroll_fine_y;  /* $2005 bits 2..0 on 2nd write */
    bool     ppu_addr_second;    /* $2006 2nd-write tracker (MMC5A) */

    uint16_t ppu_last_addr;
    uint8_t  ppu_match_count;
    uint8_t  ppu_idle_count;
    bool     in_frame;
    uint8_t  scanline_counter;
    uint8_t  fetch_count;
    uint8_t  irq_scanline;       /* $5203 */
    bool     scanline_irq_enable;
    bool     scanline_irq_pending;
    bool     abnormal_m2_a0;

    /* 8x8 -> 16 Multiplier */
    uint8_t  mul_a;              /* $5205 */
    uint8_t  mul_b;              /* $5206 */

    /* Expansion Audio ($5000-$5015) */
    Mmc5Pulse pulse1;
    Mmc5Pulse pulse2;
    bool      pulse_half_clk;
    uint16_t  audio_frame_div;
    bool      pcm_read_mode;     /* $5010 bit 0 */
    bool      pcm_irq_enable;    /* $5010 bit 7 */
    bool      pcm_irq_pending;
    uint8_t   pcm_dac;           /* $5011 */

    /* MMC5A extensions ($5207-$520A, $5800-$5BFF) */
    uint8_t  cl3_sl3_ctrl;       /* $5207 */
    uint8_t  cl3_sl3_out;        /* $5208 */
    uint16_t timer_counter;      /* $5209 / $520A */
    bool     timer_running;
    bool     timer_irq_pending;
    uint8_t  ext_5800[1024];

    Mmc5Bus  bus;
} Mmc5;

void mmc5_power_on(Mmc5 *m, const Mmc5Config *cfg, const Mmc5Bus *bus);
void mmc5_reset(Mmc5 *m);

/* Called on each CPU cycle M2 rising edge (once per CPU/DMA cycle) */
void mmc5_cpu_cycle_start(Mmc5 *m);

/* CPU reads/writes for $5000-$FFFF (also handles PCM read mode & $FFFA/$FFFB snoop) */
uint8_t mmc5_cpu_read(Mmc5 *m, uint16_t addr, uint8_t open_bus, bool *driven_out);
void    mmc5_cpu_write(Mmc5 *m, uint16_t addr, uint8_t val);

/* Snooping hooks for non-cartridge addresses ($2000-$2007, $4014) */
void mmc5_snoop_cpu_read(Mmc5 *m, uint16_t addr, uint8_t val);
void mmc5_snoop_cpu_write(Mmc5 *m, uint16_t addr, uint8_t val);

/* PPU bus reads/writes ($0000-$3FFF) */
uint8_t mmc5_ppu_read(Mmc5 *m, uint16_t addr, bool count_rd_edge, uint8_t *ciram);
void    mmc5_ppu_write(Mmc5 *m, uint16_t addr, uint8_t val, uint8_t *ciram);

/* Analog audio output (inverted relative to 2A03, pre-RC filter) */
float mmc5_audio_output(const Mmc5 *m);

#ifdef __cplusplus
}
#endif

#endif /* NINTENDO_MMC5_H */