#ifndef FFE_MAPPER6_H
#define FFE_MAPPER6_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAPPER6_MAX_PRG_SIZE   524288u
#define MAPPER6_CHR_RAM_SIZE   262144u
#define MAPPER6_WRAM_SIZE      32768u
#define MAPPER6_SCRATCH_SIZE   4096u

/* IRQ source masks compatible with Ricoh2A03 irq_line */
#define MAPPER6_IRQ_SMC        0x04u
#define MAPPER6_IRQ_FDS        0x08u

typedef enum Mapper6Mirror {
    MAPPER6_MIRROR_SINGLE_LO  = 0,
    MAPPER6_MIRROR_SINGLE_HI  = 1,
    MAPPER6_MIRROR_VERTICAL   = 2,
    MAPPER6_MIRROR_HORIZONTAL = 3
} Mapper6Mirror;

typedef enum Mapper6PrgMode {
    MAPPER6_PRG_LATCH = 0,
    MAPPER6_PRG_2M    = 1,
    MAPPER6_PRG_4M    = 2
} Mapper6PrgMode;

typedef struct Mapper6Bus {
    void *ctx;
    void (*on_irq)(void *ctx, uint8_t source_mask, bool asserted);
} Mapper6Bus;

typedef struct Mapper6Config {
    const uint8_t *prg_rom;
    uint32_t       prg_size;
    const uint8_t *chr_rom;
    uint32_t       chr_size;
    bool           chr_is_ram;
    const uint8_t *trainer;
    bool           has_trainer;
    uint8_t        submapper;     /* 0-7 latch-based banking mode (NES 1.0 = 1, Mapper 8 = 4) */
    bool           vertical_mirroring;
} Mapper6Config;

typedef struct Mapper6 {
    /* Memories */
    uint8_t prg_dram[MAPPER6_MAX_PRG_SIZE];
    uint32_t prg_size;
    uint8_t chr_ram[MAPPER6_CHR_RAM_SIZE];
    uint8_t wram[MAPPER6_WRAM_SIZE];
    uint8_t scratch_ram[MAPPER6_SCRATCH_SIZE];

    /* 1M ($42FC-$42FF) & 2M/4M ($43FC-$43FF) control */
    uint8_t       latch_mode;       /* 0..7 */
    bool          prg_write_protect;/* true = write-protected & latch enabled */
    Mapper6Mirror mirroring;
    uint8_t       raw_42fx;         /* cached A0, A1, D4, D5..D7 for $4500 read */
    uint8_t       raw_43fx;         /* cached A0, A1 for $4501 read */

    Mapper6PrgMode prg_mode_2m4m;
    uint8_t        latch;           /* $8000-$FFFF latch value */
    uint8_t        chr_8k_reg;      /* common 8 KiB CHR bank #0..#3 */
    uint8_t        prg_2m[4];       /* 8 KiB PRG banks for 2M mode */
    uint8_t        prg_4m[4];       /* 8 KiB PRG banks for 4M mode */

    /* SMC mode ($4500) & 1 KiB CHR / MMC4 ($4510-$451B) */
    uint8_t smc_mode;               /* $4500 write value */
    uint8_t wram_bank;              /* 0..3 (8 KiB bank at $6000-$7FFF) */
    bool    chr_1k_mode;            /* $4500 bit 0 */
    bool    ciram_nt_mode;          /* $4500 bit 1 */
    bool    mmc4_enabled;           /* $4500 bit 2 == 0 (effective when chr_1k_mode) */
    bool    irq_source_pa12;        /* $4500 bit 3 */
    uint8_t chr_1k[8];              /* $4510-$4517 */
    uint8_t chr_nt[4];              /* $4518-$451B */
    uint8_t mmc4_latch0;            /* 0xFD or 0xFE */
    uint8_t mmc4_latch1;            /* 0xFD or 0xFE */

    /* SMC 16-bit IRQ counter ($4501-$4503) */
    uint16_t irq_counter;
    bool     irq_enable;
    bool     irq_pending;
    bool     prev_a12;

    /* Legacy FDS Disk Data IRQ ($4024 / $4025 / $4030) */
    uint8_t  fds_ctrl;
    uint16_t fds_mclk_accum;
    bool     fds_irq_pending;

    /* Hard-reset trainer boot stub (JSR $7003 ; JMP ($FFFC)) */
    uint8_t boot_stage;             /* 0 = inactive, 1 = reset vec intercept, 2 = $41ED stub */

    Mapper6Bus bus;
} Mapper6;

void mapper6_power_on(Mapper6 *m, const Mapper6Config *cfg, const Mapper6Bus *bus);
void mapper6_reset(Mapper6 *m);

/* Called once per CPU/DMA cycle (M2 rising edge) */
void mapper6_cpu_cycle(Mapper6 *m);

/* Called per PPU dot with current PPU vbus (monitors unfiltered PPU A12 rising edges) */
void mapper6_watch_a12(Mapper6 *m, uint16_t ppu_addr);

/* CPU reads & writes for $4020-$FFFF */
uint8_t mapper6_cpu_read(Mapper6 *m, uint16_t addr, uint8_t open_bus, bool *driven_out);
void    mapper6_cpu_write(Mapper6 *m, uint16_t addr, uint8_t val);

/* Snooping hook for $2000-$3FFF & $4000-$4015 writes cached in SMC scratch RAM */
void mapper6_snoop_cpu_write(Mapper6 *m, uint16_t addr, uint8_t val);

/* PPU bus reads & writes ($0000-$3FFF) */
uint8_t mapper6_ppu_read(Mapper6 *m, uint16_t addr, bool count_rd_edge, uint8_t *ciram);
void    mapper6_ppu_write(Mapper6 *m, uint16_t addr, uint8_t val, uint8_t *ciram);

#ifdef __cplusplus
}
#endif

#endif /* FFE_MAPPER6_H */