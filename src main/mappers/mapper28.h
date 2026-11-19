#ifndef ACTION53_MAPPER28_H
#define ACTION53_MAPPER28_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Mapper28Config {
    const uint8_t *prg_rom;
    uint32_t       prg_size;
    uint8_t       *chr_mem;
    uint32_t       chr_size;
    bool           chr_is_ram;
    uint8_t       *prg_ram;
    bool           prg_ram_present;
    bool           initial_vertical_mirror;
} Mapper28Config;

typedef struct Mapper28 {
    const uint8_t *prg_rom;
    uint32_t       prg_size;
    uint8_t       *chr_mem;
    uint32_t       chr_size;
    bool           chr_is_ram;
    uint8_t       *prg_ram;
    bool           prg_ram_present;

    uint8_t        reg_sel;    /* 0=$00 CHR, 1=$01 Inner, 2=$80 Mode, 3=$81 Outer */
    uint8_t        chr_bank;   /* $00 bits 1..0 */
    uint8_t        prg_inner;  /* $01 bits 3..0 */
    uint8_t        mode;       /* $80 bits 5..0: SS PP MM */
    uint8_t        prg_outer;  /* $81 bits 7..0 (up to 8 MiB) */
} Mapper28;

void     mapper28_power_on(Mapper28 *m, const Mapper28Config *cfg);
void     mapper28_reset(Mapper28 *m);

uint8_t  mapper28_cpu_read(const Mapper28 *m, uint16_t addr, uint8_t open_bus, bool *driven_out);
void     mapper28_cpu_write(Mapper28 *m, uint16_t addr, uint8_t val);

uint32_t mapper28_map_chr(const Mapper28 *m, uint16_t addr);
uint8_t  mapper28_mirror_mode(const Mapper28 *m); /* Returns MirrorMode enum value 0..3 */

#ifdef __cplusplus
}
#endif

#endif /* ACTION53_MAPPER28_H */