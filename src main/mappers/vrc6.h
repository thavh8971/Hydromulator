#ifndef KONAMI_VRC6_H
#define KONAMI_VRC6_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VRC6_IRQ_MASK 0x04u

typedef struct Vrc6Bus {
    void *ctx;
    void (*on_irq)(void *ctx, uint8_t source_mask, bool asserted);
} Vrc6Bus;

typedef struct Vrc6Config {
    const uint8_t *prg_rom;
    uint32_t       prg_size;
    uint8_t       *chr_mem;
    uint32_t       chr_size;
    bool           chr_is_ram;
    uint8_t       *prg_ram;
    bool           prg_ram_present;
    bool           is_vrc6b; /* false = Mapper 24 (VRC6a), true = Mapper 26 (VRC6b) */
} Vrc6Config;

typedef struct Vrc6Pulse {
    uint8_t  volume;      /* 0..15 */
    uint8_t  duty;        /* 0..7 */
    bool     mode_ignore; /* true = 100% duty */
    uint16_t period;      /* 12-bit */
    bool     enabled;
    uint16_t timer;
    uint8_t  duty_step;   /* 15..0 countdown */
} Vrc6Pulse;

typedef struct Vrc6Saw {
    uint8_t  accum_rate;  /* 0..63 */
    uint16_t period;      /* 12-bit */
    bool     enabled;
    uint16_t timer;
    uint8_t  step;        /* 0..13 */
    uint8_t  accum;       /* 8-bit internal accumulator */
} Vrc6Saw;

typedef struct Vrc6 {
    const uint8_t *prg_rom;
    uint32_t       prg_size;
    uint8_t       *chr_mem;
    uint32_t       chr_size;
    bool           chr_is_ram;
    uint8_t       *prg_ram;
    bool           prg_ram_present;
    bool           is_vrc6b;

    /* PRG & PPU Banking */
    uint8_t  prg_16k;     /* $8000 bits 3..0 */
    uint8_t  prg_8k;      /* $C000 bits 4..0 */
    uint8_t  b003;        /* $B003 PPU banking style & PRG-RAM enable */
    uint8_t  r[8];        /* R0..R7 ($D000-$D003, $E000-$E003) */

    /* VRC IRQ ($F000-$F002) */
    uint8_t  irq_latch;
    uint8_t  irq_counter;
    int16_t  irq_prescaler;
    bool     irq_mode_cycle;
    bool     irq_enable;
    bool     irq_ack_enable;
    bool     irq_pending;

    /* VRC6 Expansion Audio */
    uint8_t   freq_ctrl;  /* $9003 */
    Vrc6Pulse pulse1;
    Vrc6Pulse pulse2;
    Vrc6Saw   saw;

    Vrc6Bus   bus;
} Vrc6;

void    vrc6_power_on(Vrc6 *m, const Vrc6Config *cfg, const Vrc6Bus *bus);
void    vrc6_reset(Vrc6 *m);

void    vrc6_cpu_cycle(Vrc6 *m);

uint8_t vrc6_cpu_read(const Vrc6 *m, uint16_t addr, uint8_t open_bus, bool *driven_out);
void    vrc6_cpu_write(Vrc6 *m, uint16_t addr, uint8_t val);

uint8_t vrc6_ppu_read(const Vrc6 *m, uint16_t addr, const uint8_t *ciram);
void    vrc6_ppu_write(Vrc6 *m, uint16_t addr, uint8_t val, uint8_t *ciram);

float   vrc6_audio_output(const Vrc6 *m);

#ifdef __cplusplus
}
#endif

#endif /* KONAMI_VRC6_H */