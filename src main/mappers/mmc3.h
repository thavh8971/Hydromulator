#ifndef NINTENDO_MMC3_H
#define NINTENDO_MMC3_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MMC3_IRQ_MASK 0x04u

typedef struct Mmc3Bus {
    void *ctx;
    void (*on_irq)(void *ctx, uint8_t source_mask, bool asserted);
} Mmc3Bus;

typedef struct Mmc3Config {
    const uint8_t *prg_rom;
    uint32_t       prg_size;
    uint8_t       *chr_mem;
    uint32_t       chr_size;
    bool           chr_is_ram;
    uint8_t       *prg_ram;
    bool           prg_ram_present;
    uint8_t        initial_mirroring; /* 0=H, 1=V, 4=Four */
    bool           rev_a;
    bool           mc_acc;
    bool           is_mmc6;
} Mmc3Config;

typedef struct Mmc3 {
    const uint8_t *prg_rom;
    uint32_t       prg_size;
    uint8_t       *chr_mem;
    uint32_t       chr_size;
    bool           chr_is_ram;
    uint8_t       *prg_ram;
    bool           prg_ram_present;

    uint8_t        initial_mirroring;
    uint8_t        mirroring;

    uint8_t        select;
    uint8_t        regs[8];
    uint8_t        irq_latch;
    uint8_t        irq_counter;
    bool           irq_reload;
    bool           irq_enable;
    bool           prev_a12;
    bool           a12_current;
    uint8_t        a12_low_m2;
    bool           rev_a;
    bool           mc_acc;
    uint8_t        mc_acc_div;
    bool           is_mmc6;
    bool           prg_ram_enable;
    bool           prg_ram_ro;
    uint8_t        mmc6_ram_ctrl;

    Mmc3Bus        bus;
} Mmc3;

void     mmc3_power_on(Mmc3 *m, const Mmc3Config *cfg, const Mmc3Bus *bus);
void     mmc3_reset(Mmc3 *m);
void     mmc3_sync_a12(Mmc3 *m, uint16_t ppu_vbus);
void     mmc3_watch_a12(Mmc3 *m, uint16_t ppu_vbus);
void     mmc3_on_m2_falling(Mmc3 *m);
uint8_t  mmc3_cpu_read(const Mmc3 *m, uint16_t addr, uint8_t open_bus, bool *driven_out);
void     mmc3_cpu_write(Mmc3 *m, uint16_t addr, uint8_t val);
uint32_t mmc3_map_chr(const Mmc3 *m, uint16_t addr);

#ifdef __cplusplus
}
#endif

#endif /* NINTENDO_MMC3_H */