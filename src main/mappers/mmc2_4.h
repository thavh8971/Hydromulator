#ifndef NINTENDO_MMC2_4_H
#define NINTENDO_MMC2_4_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Mmc24Config {
    const uint8_t *prg_rom;
    uint32_t       prg_size;
    uint8_t       *chr_mem;
    uint32_t       chr_size;
    bool           chr_is_ram;
    uint8_t       *prg_ram;
    bool           prg_ram_present;
    bool           is_mmc4;           /* false = Mapper 9 (MMC2), true = Mapper 10 (MMC4) */
    uint8_t        initial_mirroring; /* 0=H, 1=V */
} Mmc24Config;

typedef struct Mmc24 {
    const uint8_t *prg_rom;
    uint32_t       prg_size;
    uint8_t       *chr_mem;
    uint32_t       chr_size;
    bool           chr_is_ram;
    uint8_t       *prg_ram;
    bool           prg_ram_present;
    bool           is_mmc4;

    uint8_t        initial_mirroring;
    uint8_t        mirroring;

    uint8_t        prg_bank;
    uint8_t        chr_fd0;
    uint8_t        chr_fe0;
    uint8_t        chr_fd1;
    uint8_t        chr_fe1;
    uint8_t        latch0;
    uint8_t        latch1;
} Mmc24;

void     mmc24_power_on(Mmc24 *m, const Mmc24Config *cfg);
void     mmc24_reset(Mmc24 *m);
uint8_t  mmc24_cpu_read(const Mmc24 *m, uint16_t addr, uint8_t open_bus, bool *driven_out);
void     mmc24_cpu_write(Mmc24 *m, uint16_t addr, uint8_t val);
uint8_t  mmc24_ppu_chr_read(Mmc24 *m, uint16_t addr, bool latch_edge);
void     mmc24_ppu_chr_write(Mmc24 *m, uint16_t addr, uint8_t val);

#ifdef __cplusplus
}
#endif

#endif /* NINTENDO_MMC2_4_H */