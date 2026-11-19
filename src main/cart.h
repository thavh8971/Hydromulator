#ifndef HYDROMULATOR_CART_H
#define HYDROMULATOR_CART_H

#include "mmc1.h"
#include "mmc3.h"
#include "mmc2_4.h"
#include "mmc5.h"
#include "mapper6.h"
#include "mapper28.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum MirrorMode {
    MIRROR_HORIZONTAL  = 0,
    MIRROR_VERTICAL    = 1,
    MIRROR_SINGLE_LO   = 2,
    MIRROR_SINGLE_HI   = 3,
    MIRROR_FOUR_SCREEN = 4
} MirrorMode;

typedef struct CartBus {
    void *ctx;
    void (*on_irq)(void *ctx, uint8_t source_mask, bool asserted);
} CartBus;

typedef struct Cartridge {
    uint8_t   *prg_rom;
    uint32_t   prg_size;
    uint8_t   *chr_mem;
    uint32_t   chr_size;
    bool       chr_is_ram;
    uint8_t    prg_ram[0x2000];
    bool       prg_ram_present;
    uint32_t   prg_ram_size;
    uint32_t   prg_nvram_size;
    uint8_t    trainer[512];
    bool       has_trainer;

    uint16_t   mapper;
    uint8_t    submapper;
    bool       is_nes2;
    MirrorMode mirroring;
    MirrorMode initial_mirroring;

    /* Discrete 74xx mappers (0, 2, 3, 7) */
    uint8_t    ux_prg_bank;
    uint8_t    cn_chr_bank;
    uint8_t    ax_prg_bank;
    bool       ax_bus_conflicts;

    /* ASIC / Complex mapper modules */
    Mmc1       mmc1;
    Mmc3       mmc3;
    Mmc24      mmc24;
    Mmc5       mmc5;
    Mapper6    mapper6;
    Mapper28   mapper28;

    /* Runtime config flags */
    bool       mmc3_rev_a;
    bool       mmc3_mc_acc;
    bool       mmc3_is_mmc6;
    bool       mmc5_is_5a;
    bool       mmc5_sl_mode;
    uint8_t    m6_init_mode;

    CartBus    bus;
} Cartridge;

bool     cart_load_ines(Cartridge *cart, const char *path);
void     cart_free(Cartridge *cart);
void     cart_power_on(Cartridge *cart, const CartBus *bus);
void     cart_reset(Cartridge *cart);
void     cart_sync_ppu_addr(Cartridge *cart, uint16_t ppu_vbus);

uint8_t  cart_cpu_read(Cartridge *cart, uint16_t addr, uint8_t open_bus, bool *driven_out);
void     cart_cpu_write(Cartridge *cart, uint16_t addr, uint8_t val, uint64_t cpu_cycles);

uint8_t  cart_ppu_read(Cartridge *cart, uint16_t addr, bool ppu_pd_rb, bool is_dup_pd_rb, uint8_t *ciram);
void     cart_ppu_write(Cartridge *cart, uint16_t addr, uint8_t val, uint8_t *ciram);

void     cart_cpu_cycle(Cartridge *cart);
void     cart_on_m2_falling(Cartridge *cart);
void     cart_watch_ppu_addr(Cartridge *cart, uint16_t ppu_vbus);

void     cart_snoop_cpu_read(Cartridge *cart, uint16_t addr, uint8_t val);
void     cart_snoop_cpu_write(Cartridge *cart, uint16_t addr, uint8_t val);

float    cart_audio_output(const Cartridge *cart);
void     cart_format_tag(const Cartridge *cart, char *out, size_t cap);

#ifdef __cplusplus
}
#endif

#endif /* HYDROMULATOR_CART_H */