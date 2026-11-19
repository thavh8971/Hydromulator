#ifndef NINTENDO_MMC1_H
#define NINTENDO_MMC1_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Mmc1Config {
    const uint8_t *prg_rom;
    uint32_t       prg_size;
    uint8_t       *chr_mem;
    uint32_t       chr_size;
    bool           chr_is_ram;
    uint8_t       *prg_ram;
    bool           prg_ram_present;
    uint8_t        initial_mirroring; /* 0=H, 1=V, 2=SingleLo, 3=SingleHi, 4=Four */
} Mmc1Config;

typedef struct Mmc1 {
    const uint8_t *prg_rom;
    uint32_t       prg_size;
    uint8_t       *chr_mem;
    uint32_t       chr_size;
    bool           chr_is_ram;
    uint8_t       *prg_ram;
    bool           prg_ram_present;

    uint8_t        initial_mirroring;
    uint8_t        mirroring;

    uint8_t        shift;
    uint8_t        ctrl;
    uint8_t        chr0;
    uint8_t        chr1;
    uint8_t        prg;
    uint64_t       last_write_cycle;
} Mmc1;

void    mmc1_power_on(Mmc1 *m, const Mmc1Config *cfg);
void    mmc1_reset(Mmc1 *m);
uint8_t mmc1_cpu_read(const Mmc1 *m, uint16_t addr, uint8_t open_bus, bool *driven_out);
void    mmc1_cpu_write(Mmc1 *m, uint16_t addr, uint8_t val, uint64_t cpu_cycles);
uint32_t mmc1_map_chr(const Mmc1 *m, uint16_t addr);

#ifdef __cplusplus
}
#endif

#endif /* NINTENDO_MMC1_H */