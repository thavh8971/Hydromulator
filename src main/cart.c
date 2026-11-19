#include "cart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static inline bool cart_is_mmc1(const Cartridge *cart) {
    return cart->mapper == 1u || cart->mapper == 155u;
}

static inline bool cart_is_mapper6(const Cartridge *cart) {
    return cart->mapper == 6u || cart->mapper == 8u;
}

static inline bool cart_is_mmc24(const Cartridge *cart) {
    return cart->mapper == 9u || cart->mapper == 10u;
}

static inline bool cart_is_vrc6(const Cartridge *cart) {
    return cart->mapper == 24u || cart->mapper == 26u;
}

static uint16_t mirror_nametable_addr(MirrorMode mode, uint16_t addr) {
    uint16_t nt_off = (uint16_t)(addr & 0x03FFu);
    uint16_t table  = (uint16_t)((addr >> 10) & 0x03u);

    switch (mode) {
        case MIRROR_HORIZONTAL:  return (uint16_t)(((table >> 1) << 10) | nt_off);
        case MIRROR_VERTICAL:    return (uint16_t)(((table & 0x01u) << 10) | nt_off);
        case MIRROR_SINGLE_LO:   return nt_off;
        case MIRROR_SINGLE_HI:   return (uint16_t)(0x0400u | nt_off);
        case MIRROR_FOUR_SCREEN: return (uint16_t)(addr & 0x0FFFu);
    }
    return nt_off;
}

void cart_free(Cartridge *cart) {
    free(cart->prg_rom);
    free(cart->chr_mem);
    memset(cart, 0, sizeof(*cart));
}

bool cart_load_ines(Cartridge *cart, const char *path) {
    cart_free(cart);

    FILE *fp = fopen(path, "rb");
    if (!fp) {
        fprintf(stderr, "Khong the mo ROM: %s\n", path);
        return false;
    }

    uint8_t hdr[16];
    if (fread(hdr, 1, 16, fp) != 16) {
        fclose(fp);
        return false;
    }
    if (hdr[0] != 'N' || hdr[1] != 'E' || hdr[2] != 'S' || hdr[3] != 0x1Au) {
        fprintf(stderr, "Khong phai dinh dang iNES (.nes).\n");
        fclose(fp);
        return false;
    }

    uint8_t flags6 = hdr[6];
    uint8_t flags7 = hdr[7];
    bool is_nes2 = ((flags7 & 0x0Cu) == 0x08u);

    uint32_t prg_banks_16k = hdr[4];
    uint32_t chr_banks_8k  = hdr[5];
    uint16_t mapper = 0u;
    uint8_t  submapper = 0u;

    if (is_nes2) {
        prg_banks_16k |= (uint32_t)(hdr[9] & 0x0Fu) << 8;
        chr_banks_8k  |= (uint32_t)(hdr[9] & 0xF0u) << 4;
        mapper = (uint16_t)(((uint16_t)(hdr[8] & 0x0Fu) << 8) |
                            (flags7 & 0xF0u) |
                            (flags6 >> 4));
        submapper = (uint8_t)((hdr[8] >> 4) & 0x0Fu);
    } else {
        bool dirty_header = (hdr[12] | hdr[13] | hdr[14] | hdr[15]) != 0u;
        mapper = (uint16_t)((dirty_header ? 0u : (flags7 & 0xF0u)) | (flags6 >> 4));
    }

    if (prg_banks_16k == 0u) {
        fclose(fp);
        return false;
    }
    if (mapper > 10u && mapper != 24u && mapper != 26u && mapper != 28u && mapper != 155u) {
        fprintf(stderr, "Mapper %u chua duoc ho tro.\n", (unsigned)mapper);
        fclose(fp);
        return false;
    }

    if (flags6 & 0x04u) {
        if (fread(cart->trainer, 1, 512, fp) != 512) {
            fclose(fp);
            return false;
        }
        cart->has_trainer = true;
    }

    uint32_t prg_size = prg_banks_16k * 16384u;
    uint32_t chr_size = chr_banks_8k * 8192u;
    bool chr_is_ram = (chr_size == 0u);
    if (chr_is_ram) {
        uint32_t default_chr_ram = (mapper == 28u) ? 32768u : 8192u;
        if (is_nes2) {
            uint8_t chr_ram_shift   = (uint8_t)(hdr[11] & 0x0Fu);
            uint8_t chr_nvram_shift = (uint8_t)((hdr[11] >> 4) & 0x0Fu);
            uint32_t total_chr_ram  = (chr_ram_shift   ? (64u << chr_ram_shift)   : 0u) +
                                      (chr_nvram_shift ? (64u << chr_nvram_shift) : 0u);
            chr_size = (total_chr_ram > 0u) ? total_chr_ram : default_chr_ram;
        } else {
            chr_size = default_chr_ram;
        }
    }

    cart->prg_rom = (uint8_t *)malloc(prg_size);
    cart->chr_mem = (uint8_t *)calloc(1u, chr_size);
    if (!cart->prg_rom || !cart->chr_mem) {
        cart_free(cart);
        fclose(fp);
        return false;
    }

    if (fread(cart->prg_rom, 1, prg_size, fp) != prg_size ||
        (!chr_is_ram && fread(cart->chr_mem, 1, chr_size, fp) != chr_size)) {
        cart_free(cart);
        fclose(fp);
        return false;
    }
    fclose(fp);

    cart->prg_size          = prg_size;
    cart->chr_size          = chr_size;
    cart->chr_is_ram        = chr_is_ram;
    cart->mapper            = mapper;
    cart->submapper         = submapper;
    cart->is_nes2           = is_nes2;
    cart->initial_mirroring = (mapper == 7u)   ? MIRROR_SINGLE_LO
                            : (flags6 & 0x08u) ? MIRROR_FOUR_SCREEN
                            : (flags6 & 0x01u) ? MIRROR_VERTICAL
                                               : MIRROR_HORIZONTAL;
    cart->mirroring         = cart->initial_mirroring;

    if (is_nes2) {
        uint8_t ram_shift   = (uint8_t)(hdr[10] & 0x0Fu);
        uint8_t nvram_shift = (uint8_t)((hdr[10] >> 4) & 0x0Fu);
        cart->prg_ram_size   = ram_shift   ? (64u << ram_shift)   : 0u;
        cart->prg_nvram_size = nvram_shift ? (64u << nvram_shift) : 0u;
        uint32_t prg_ram_sz  = cart->prg_ram_size + cart->prg_nvram_size;
        cart->prg_ram_present = (mapper == 4u) ? ((prg_ram_sz > 0u) || (submapper == 1u))
                              : (mapper == 1u || mapper == 155u || mapper == 7u ||
                                 mapper == 9u || mapper == 10u || mapper == 24u ||
                                 mapper == 26u || mapper == 28u) ? (prg_ram_sz > 0u)
                                                                 : true;
    } else {
        cart->prg_ram_size    = ((mapper == 1u || mapper == 155u) && prg_size > 262144u) ? 32768u : 8192u;
        cart->prg_nvram_size  = 0u;
        cart->prg_ram_present = (mapper != 7u && mapper != 9u);
    }

    cart->mmc3_is_mmc6 = (mapper == 4u && is_nes2 && submapper == 1u);
    cart->mmc3_mc_acc  = (mapper == 4u && is_nes2 && submapper == 3u);
    cart->mmc3_rev_a   = (mapper == 4u && is_nes2 && (submapper == 4u || submapper == 1u));

    if (mapper == 8u) {
        cart->m6_init_mode = 4u;
    } else if (mapper == 6u) {
        cart->m6_init_mode = is_nes2 ? (uint8_t)(submapper & 0x07u) : 1u;
    }

    cart->ax_bus_conflicts = (mapper == 7u && is_nes2 && submapper == 2u);

    char tag[64];
    cart_format_tag(cart, tag, sizeof(tag));
    printf("Loaded ROM: %s | Mapper %u.%u (%s) | PRG %u KB | CHR %u KB (%s) | Mirror %u%s\n",
           path, (unsigned)cart->mapper, (unsigned)cart->submapper,
           tag[0] ? tag : "std",
           (unsigned)(cart->prg_size / 1024u),
           (unsigned)(cart->chr_size / 1024u), cart->chr_is_ram ? "RAM" : "ROM",
           (unsigned)cart->mirroring,
           cart->has_trainer ? " | Trainer" : "");
    return true;
}

void cart_power_on(Cartridge *cart, const CartBus *bus) {
    if (bus != NULL) {
        cart->bus = *bus;
    }
    memset(cart->prg_ram, 0, sizeof(cart->prg_ram));
    if (cart->has_trainer && !cart_is_mapper6(cart)) {
        memcpy(&cart->prg_ram[0x1000u], cart->trainer, 512u);
    }

    cart->ux_prg_bank = 0u;
    cart->cn_chr_bank = 0u;
    cart->ax_prg_bank = 0u;
    cart->mirroring   = cart->initial_mirroring;

    switch (cart->mapper) {
        case 1:
        case 155: {
            uint32_t ram_sz = cart->is_nes2
                ? (cart->prg_ram_size + cart->prg_nvram_size)
                : cart->prg_ram_size;
            if (ram_sz > sizeof(cart->prg_ram)) {
                ram_sz = sizeof(cart->prg_ram);
            }
            Mmc1Config cfg = {
                .prg_rom           = cart->prg_rom,
                .prg_size          = cart->prg_size,
                .chr_mem           = cart->chr_mem,
                .chr_size          = cart->chr_size,
                .chr_is_ram        = cart->chr_is_ram,
                .prg_ram           = cart->prg_ram,
                .prg_ram_size      = ram_sz,
                .prg_ram_present   = cart->prg_ram_present && (ram_sz > 0u),
                .initial_mirroring = (uint8_t)cart->initial_mirroring,
                .submapper         = cart->submapper,
                .is_mmc1a          = (cart->mapper == 155u),
            };
            mmc1_power_on(&cart->mmc1, &cfg);
            break;
        }
        case 4: {
            Mmc3Config cfg = {
                .prg_rom           = cart->prg_rom,
                .prg_size          = cart->prg_size,
                .chr_mem           = cart->chr_mem,
                .chr_size          = cart->chr_size,
                .chr_is_ram        = cart->chr_is_ram,
                .prg_ram           = cart->prg_ram,
                .prg_ram_present   = cart->prg_ram_present,
                .initial_mirroring = (uint8_t)cart->initial_mirroring,
                .rev_a             = cart->mmc3_rev_a,
                .mc_acc            = cart->mmc3_mc_acc,
                .is_mmc6           = cart->mmc3_is_mmc6,
            };
            Mmc3Bus mbus = {
                .ctx    = cart->bus.ctx,
                .on_irq = cart->bus.on_irq,
            };
            mmc3_power_on(&cart->mmc3, &cfg, &mbus);
            break;
        }
        case 5: {
            Mmc5Config mcfg = {
                .prg_rom        = cart->prg_rom,
                .prg_size       = cart->prg_size,
                .chr_mem        = cart->chr_mem,
                .chr_size       = cart->chr_size,
                .chr_is_ram     = cart->chr_is_ram,
                .prg_ram_size   = cart->prg_ram_size,
                .prg_nvram_size = cart->prg_nvram_size,
                .is_nes2        = cart->is_nes2,
                .is_mmc5a       = cart->mmc5_is_5a,
                .sl_mode        = cart->mmc5_sl_mode,
            };
            Mmc5Bus mbus = {
                .ctx    = cart->bus.ctx,
                .on_irq = cart->bus.on_irq,
            };
            mmc5_power_on(&cart->mmc5, &mcfg, &mbus);
            break;
        }
        case 6:
        case 8: {
            Mapper6Config m6cfg = {
                .prg_rom            = cart->prg_rom,
                .prg_size           = cart->prg_size,
                .chr_rom            = cart->chr_mem,
                .chr_size           = cart->chr_size,
                .chr_is_ram         = cart->chr_is_ram,
                .trainer            = cart->trainer,
                .has_trainer        = cart->has_trainer,
                .submapper          = cart->m6_init_mode,
                .vertical_mirroring = (cart->initial_mirroring == MIRROR_VERTICAL),
            };
            Mapper6Bus m6bus = {
                .ctx    = cart->bus.ctx,
                .on_irq = cart->bus.on_irq,
            };
            mapper6_power_on(&cart->mapper6, &m6cfg, &m6bus);
            break;
        }
        case 9:
        case 10: {
            Mmc24Config cfg = {
                .prg_rom           = cart->prg_rom,
                .prg_size          = cart->prg_size,
                .chr_mem           = cart->chr_mem,
                .chr_size          = cart->chr_size,
                .chr_is_ram        = cart->chr_is_ram,
                .prg_ram           = cart->prg_ram,
                .prg_ram_present   = cart->prg_ram_present,
                .is_mmc4           = (cart->mapper == 10u),
                .initial_mirroring = (uint8_t)cart->initial_mirroring,
            };
            mmc24_power_on(&cart->mmc24, &cfg);
            break;
        }
        case 24:
        case 26: {
            Vrc6Config cfg = {
                .prg_rom         = cart->prg_rom,
                .prg_size        = cart->prg_size,
                .chr_mem         = cart->chr_mem,
                .chr_size        = cart->chr_size,
                .chr_is_ram      = cart->chr_is_ram,
                .prg_ram         = cart->prg_ram,
                .prg_ram_present = cart->prg_ram_present,
                .is_vrc6b        = (cart->mapper == 26u),
            };
            Vrc6Bus vbus = {
                .ctx    = cart->bus.ctx,
                .on_irq = cart->bus.on_irq,
            };
            vrc6_power_on(&cart->vrc6, &cfg, &vbus);
            break;
        }
        case 28: {
            Mapper28Config cfg = {
                .prg_rom                 = cart->prg_rom,
                .prg_size                = cart->prg_size,
                .chr_mem                 = cart->chr_mem,
                .chr_size                = cart->chr_size,
                .chr_is_ram              = cart->chr_is_ram,
                .prg_ram                 = cart->prg_ram,
                .prg_ram_present         = cart->prg_ram_present,
                .initial_vertical_mirror = (cart->initial_mirroring == MIRROR_VERTICAL),
            };
            mapper28_power_on(&cart->mapper28, &cfg);
            break;
        }
        default:
            break;
    }
}

void cart_reset(Cartridge *cart) {
    cart->ux_prg_bank = 0u;
    cart->cn_chr_bank = 0u;
    if (cart->mapper != 7u && cart->mapper != 28u) {
        cart->mirroring = cart->initial_mirroring;
    }

    switch (cart->mapper) {
        case 1:
        case 155: mmc1_reset(&cart->mmc1); break;
        case 4:   mmc3_reset(&cart->mmc3); break;
        case 5:   mmc5_reset(&cart->mmc5); break;
        case 6:
        case 8:   mapper6_reset(&cart->mapper6); break;
        case 9:
        case 10:  mmc24_reset(&cart->mmc24); break;
        case 24:
        case 26:  vrc6_reset(&cart->vrc6); break;
        case 28:  mapper28_reset(&cart->mapper28); break;
        default:  break;
    }
}

void cart_sync_ppu_addr(Cartridge *cart, uint16_t ppu_vbus) {
    if (cart->mapper == 4u) {
        mmc3_sync_a12(&cart->mmc3, ppu_vbus);
    }
}

uint8_t cart_cpu_read(Cartridge *cart, uint16_t addr, uint8_t open_bus, bool *driven_out) {
    if (cart->mapper == 5u) {
        if (addr < 0x5000u) {
            if (driven_out) *driven_out = false;
            return open_bus;
        }
        return mmc5_cpu_read(&cart->mmc5, addr, open_bus, driven_out);
    }
    if (cart_is_mapper6(cart)) {
        return mapper6_cpu_read(&cart->mapper6, addr, open_bus, driven_out);
    }
    if (cart_is_mmc1(cart)) {
        return mmc1_cpu_read(&cart->mmc1, addr, open_bus, driven_out);
    }
    if (cart->mapper == 4u) {
        return mmc3_cpu_read(&cart->mmc3, addr, open_bus, driven_out);
    }
    if (cart_is_mmc24(cart)) {
        return mmc24_cpu_read(&cart->mmc24, addr, open_bus, driven_out);
    }
    if (cart_is_vrc6(cart)) {
        return vrc6_cpu_read(&cart->vrc6, addr, open_bus, driven_out);
    }
    if (cart->mapper == 28u) {
        return mapper28_cpu_read(&cart->mapper28, addr, open_bus, driven_out);
    }

    if (addr >= 0x6000u && addr < 0x8000u) {
        if (cart->mapper == 7u && !cart->prg_ram_present) {
            if (driven_out) *driven_out = false;
            return open_bus;
        }
        if (driven_out) *driven_out = true;
        return cart->prg_ram[addr - 0x6000u];
    }
    if (addr < 0x8000u) {
        if (driven_out) *driven_out = false;
        return open_bus;
    }

    if (driven_out) *driven_out = true;
    switch (cart->mapper) {
        case 0:
        case 3:
            return cart->prg_rom[(addr - 0x8000u) % cart->prg_size];

        case 2: {
            uint32_t banks_16k = cart->prg_size / 16384u;
            if (banks_16k == 0u) return 0xFFu;
            uint32_t bank = (addr < 0xC000u) ? ((uint32_t)cart->ux_prg_bank % banks_16k)
                                             : (banks_16k - 1u);
            return cart->prg_rom[(bank * 16384u) | (addr & 0x3FFFu)];
        }

        case 7: {
            uint32_t banks_32k = cart->prg_size / 32768u;
            if (banks_32k == 0u) return cart->prg_rom[(addr - 0x8000u) % cart->prg_size];
            uint32_t bank = (uint32_t)cart->ax_prg_bank % banks_32k;
            return cart->prg_rom[(bank * 32768u) | (addr & 0x7FFFu)];
        }
    }
    return open_bus;
}

void cart_cpu_write(Cartridge *cart, uint16_t addr, uint8_t val, uint64_t cpu_cycles) {
    if (cart->mapper == 5u) {
        if (addr >= 0x5000u) {
            mmc5_cpu_write(&cart->mmc5, addr, val);
        }
        return;
    }
    if (cart_is_mapper6(cart)) {
        mapper6_cpu_write(&cart->mapper6, addr, val);
        return;
    }
    if (cart_is_mmc1(cart)) {
        mmc1_cpu_write(&cart->mmc1, addr, val, cpu_cycles);
        return;
    }
    if (cart->mapper == 4u) {
        mmc3_cpu_write(&cart->mmc3, addr, val);
        return;
    }
    if (cart_is_mmc24(cart)) {
        mmc24_cpu_write(&cart->mmc24, addr, val);
        return;
    }
    if (cart_is_vrc6(cart)) {
        vrc6_cpu_write(&cart->vrc6, addr, val);
        return;
    }
    if (cart->mapper == 28u) {
        mapper28_cpu_write(&cart->mapper28, addr, val);
        return;
    }

    if (addr >= 0x6000u && addr < 0x8000u) {
        if (cart->mapper == 7u && !cart->prg_ram_present) {
            return;
        }
        cart->prg_ram[addr - 0x6000u] = val;
        return;
    }
    if (addr < 0x8000u) {
        return;
    }

    switch (cart->mapper) {
        case 2: cart->ux_prg_bank = val; break;
        case 3: cart->cn_chr_bank = val; break;
        case 7: {
            uint8_t written = val;
            if (cart->ax_bus_conflicts) {
                uint32_t banks_32k = cart->prg_size / 32768u;
                uint8_t rom_val = (banks_32k == 0u)
                    ? cart->prg_rom[(addr - 0x8000u) % cart->prg_size]
                    : cart->prg_rom[(((uint32_t)cart->ax_prg_bank % banks_32k) * 32768u) | (addr & 0x7FFFu)];
                written &= rom_val;
            }
            cart->ax_prg_bank = (uint8_t)(written & 0x0Fu);
            cart->mirroring   = (written & 0x10u) ? MIRROR_SINGLE_HI : MIRROR_SINGLE_LO;
            break;
        }
        default:
            break;
    }
}

static uint32_t cart_map_discrete_chr(const Cartridge *cart, uint16_t addr) {
    if (cart->mapper == 3u) {
        uint32_t banks_8k = cart->chr_size / 8192u;
        if (banks_8k == 0u) return addr % cart->chr_size;
        return (((uint32_t)cart->cn_chr_bank % banks_8k) * 8192u) | (addr & 0x1FFFu);
    }
    return addr % cart->chr_size;
}

uint8_t cart_ppu_read(Cartridge *cart, uint16_t addr, bool ppu_pd_rb, bool is_dup_pd_rb, uint8_t *ciram) {
    addr &= 0x3FFFu;
    bool count_edge = !is_dup_pd_rb;
    bool latch_edge = !ppu_pd_rb || is_dup_pd_rb;

    if (cart->mapper == 5u) {
        return mmc5_ppu_read(&cart->mmc5, addr, count_edge, ciram);
    }
    if (cart_is_mapper6(cart)) {
        return mapper6_ppu_read(&cart->mapper6, addr, latch_edge, ciram);
    }
    if (cart_is_mmc24(cart)) {
        if (addr < 0x2000u) {
            return mmc24_ppu_chr_read(&cart->mmc24, addr, latch_edge);
        }
        return ciram[mirror_nametable_addr((MirrorMode)cart->mmc24.mirroring, addr)];
    }
    if (cart_is_vrc6(cart)) {
        return vrc6_ppu_read(&cart->vrc6, addr, ciram);
    }

    if (addr < 0x2000u) {
        if (cart_is_mmc1(cart))  return cart->chr_mem[mmc1_map_chr(&cart->mmc1, addr)];
        if (cart->mapper == 4u)  return cart->chr_mem[mmc3_map_chr(&cart->mmc3, addr)];
        if (cart->mapper == 28u) return cart->chr_mem[mapper28_map_chr(&cart->mapper28, addr)];
        return cart->chr_mem[cart_map_discrete_chr(cart, addr)];
    }

    MirrorMode mir = cart_is_mmc1(cart)    ? (MirrorMode)cart->mmc1.mirroring
                   : (cart->mapper == 4u)  ? (MirrorMode)cart->mmc3.mirroring
                   : (cart->mapper == 28u) ? (MirrorMode)mapper28_mirror_mode(&cart->mapper28)
                                           : cart->mirroring;
    return ciram[mirror_nametable_addr(mir, addr)];
}

void cart_ppu_write(Cartridge *cart, uint16_t addr, uint8_t val, uint8_t *ciram) {
    addr &= 0x3FFFu;

    if (cart->mapper == 5u) {
        mmc5_ppu_write(&cart->mmc5, addr, val, ciram);
        return;
    }
    if (cart_is_mapper6(cart)) {
        mapper6_ppu_write(&cart->mapper6, addr, val, ciram);
        return;
    }
    if (cart_is_mmc24(cart)) {
        if (addr < 0x2000u) {
            mmc24_ppu_chr_write(&cart->mmc24, addr, val);
        } else {
            ciram[mirror_nametable_addr((MirrorMode)cart->mmc24.mirroring, addr)] = val;
        }
        return;
    }
    if (cart_is_vrc6(cart)) {
        vrc6_ppu_write(&cart->vrc6, addr, val, ciram);
        return;
    }

    if (addr < 0x2000u) {
        if (cart->chr_is_ram) {
            if (cart_is_mmc1(cart))       cart->chr_mem[mmc1_map_chr(&cart->mmc1, addr)] = val;
            else if (cart->mapper == 4u)  cart->chr_mem[mmc3_map_chr(&cart->mmc3, addr)] = val;
            else if (cart->mapper == 28u) cart->chr_mem[mapper28_map_chr(&cart->mapper28, addr)] = val;
            else                          cart->chr_mem[cart_map_discrete_chr(cart, addr)] = val;
        }
        return;
    }

    MirrorMode mir = cart_is_mmc1(cart)    ? (MirrorMode)cart->mmc1.mirroring
                   : (cart->mapper == 4u)  ? (MirrorMode)cart->mmc3.mirroring
                   : (cart->mapper == 28u) ? (MirrorMode)mapper28_mirror_mode(&cart->mapper28)
                                           : cart->mirroring;
    ciram[mirror_nametable_addr(mir, addr)] = val;
}

void cart_cpu_cycle(Cartridge *cart) {
    if (cart->mapper == 5u) {
        mmc5_cpu_cycle_start(&cart->mmc5);
    } else if (cart_is_mapper6(cart)) {
        mapper6_cpu_cycle(&cart->mapper6);
    } else if (cart_is_vrc6(cart)) {
        vrc6_cpu_cycle(&cart->vrc6);
    }
}

void cart_on_m2_falling(Cartridge *cart) {
    if (cart->mapper == 4u) {
        mmc3_on_m2_falling(&cart->mmc3);
    }
}

void cart_watch_ppu_addr(Cartridge *cart, uint16_t ppu_vbus) {
    if (cart->mapper == 4u) {
        mmc3_watch_a12(&cart->mmc3, ppu_vbus);
    } else if (cart_is_mapper6(cart)) {
        mapper6_watch_a12(&cart->mapper6, ppu_vbus);
    }
}

void cart_snoop_cpu_read(Cartridge *cart, uint16_t addr, uint8_t val) {
    if (cart->mapper == 5u) {
        mmc5_snoop_cpu_read(&cart->mmc5, addr, val);
    }
}

void cart_snoop_cpu_write(Cartridge *cart, uint16_t addr, uint8_t val) {
    if (cart->mapper == 5u) {
        mmc5_snoop_cpu_write(&cart->mmc5, addr, val);
    } else if (cart_is_mapper6(cart)) {
        mapper6_snoop_cpu_write(&cart->mapper6, addr, val);
    }
}

float cart_audio_output(const Cartridge *cart) {
    if (cart->mapper == 5u) {
        return mmc5_audio_output(&cart->mmc5);
    }
    if (cart_is_vrc6(cart)) {
        return vrc6_audio_output(&cart->vrc6);
    }
    return 0.0f;
}

void cart_format_tag(const Cartridge *cart, char *out, size_t cap) {
    if (cap == 0u) return;
    out[0] = '\0';

    if (cart_is_mmc1(cart)) {
        uint32_t ram_sz = cart->is_nes2
            ? (cart->prg_ram_size + cart->prg_nvram_size)
            : cart->prg_ram_size;
        bool is_snrom = (cart->mapper == 1u) &&
                        (cart->prg_size <= 262144u) &&
                        (cart->chr_size <= 8192u) &&
                        (ram_sz == 8192u);
        const char *tag = (cart->mapper == 155u)     ? "MMC1A"
                        : (ram_sz >= 32768u)         ? "SXROM"
                        : (ram_sz >= 16384u)         ? "SOROM"
                        : (cart->prg_size > 262144u) ? "SUROM"
                        : is_snrom                   ? "SNROM"
                                                     : "MMC1";
        snprintf(out, cap, "%s", tag);
    } else if (cart->mapper == 4u) {
        const char *tag = cart->mmc3_is_mmc6 ? "MMC6"
                        : cart->mmc3_mc_acc  ? "MC-ACC"
                        : cart->mmc3_rev_a   ? "MMC3A"
                                             : "MMC3B";
        snprintf(out, cap, "%s | F6", tag);
    } else if (cart->mapper == 5u) {
        const char *chip = cart->mmc5_is_5a   ? "MMC5A" : "MMC5";
        const char *pin  = cart->mmc5_sl_mode ? "SL"    : "CL";
        snprintf(out, cap, "%s/%s | F6/F7", chip, pin);
    } else if (cart_is_mapper6(cart)) {
        static const char *const m6_names[8] = {
            "0:UNROM", "1:UN1ROM", "2:UOROM", "3:RevUOROM",
            "4:GNROM", "5:CNROM256", "6:CNROM128", "7:NROM256"
        };
        snprintf(out, cap, "FFE %s | F6", m6_names[cart->m6_init_mode & 0x07u]);
    } else if (cart->mapper == 7u) {
        snprintf(out, cap, "AxROM %s | F6", cart->ax_bus_conflicts ? "BC:AND" : "BC:OFF");
    } else if (cart->mapper == 9u) {
        snprintf(out, cap, "MMC2");
    } else if (cart->mapper == 10u) {
        snprintf(out, cap, "MMC4");
    } else if (cart->mapper == 24u) {
        snprintf(out, cap, "VRC6a");
    } else if (cart->mapper == 26u) {
        snprintf(out, cap, "VRC6b");
    } else if (cart->mapper == 28u) {
        snprintf(out, cap, "Action 53");
    }
}