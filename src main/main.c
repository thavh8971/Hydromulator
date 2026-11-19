#include "2a03.h"
#include "2c02.h"
#include "apu.h"
#include "mmc5.h"
#include "mapper6.h"

#include <windows.h>
#include <commdlg.h>
#include <mmsystem.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define AUDIO_SAMPLE_RATE    96000u
#define AUDIO_NUM_BUFS       16u
#define AUDIO_BUF_SAMPLES    1470u

typedef enum MirrorMode {
    MIRROR_HORIZONTAL  = 0,
    MIRROR_VERTICAL    = 1,
    MIRROR_SINGLE_LO   = 2,
    MIRROR_SINGLE_HI   = 3,
    MIRROR_FOUR_SCREEN = 4
} MirrorMode;

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

    uint8_t    mmc1_shift;
    uint8_t    mmc1_ctrl;
    uint8_t    mmc1_chr0;
    uint8_t    mmc1_chr1;
    uint8_t    mmc1_prg;
    uint64_t   mmc1_last_write_cycle;

    uint8_t    ux_prg_bank;
    uint8_t    cn_chr_bank;

    uint8_t    mmc3_select;
    uint8_t    mmc3_regs[8];
    uint8_t    mmc3_irq_latch;
    uint8_t    mmc3_irq_counter;
    bool       mmc3_irq_reload;
    bool       mmc3_irq_enable;
    bool       mmc3_prev_a12;
    bool       mmc3_a12_current;
    uint8_t    mmc3_a12_low_m2;
    bool       mmc3_rev_a;
    bool       mmc3_mc_acc;
    uint8_t    mmc3_mc_acc_div;
    bool       mmc3_is_mmc6;
    bool       mmc3_prg_ram_enable;
    bool       mmc3_prg_ram_ro;
    uint8_t    mmc6_ram_ctrl;

    bool       mmc5_is_5a;
    bool       mmc5_sl_mode;

    uint8_t    m6_init_mode;
} Cartridge;

typedef struct WinAudioOut {
    HWAVEOUT hwo;
    WAVEHDR  hdrs[AUDIO_NUM_BUFS];
    int16_t  pcm[AUDIO_NUM_BUFS][AUDIO_BUF_SAMPLES];
    uint32_t write_buf;
    uint32_t write_pos;
    double   cycle_accum;
    double   sample_sum;
    uint32_t sample_cnt;
    bool     ready;
} WinAudioOut;

typedef struct NesEmulator {
    Ricoh2A03   cpu;
    Ricoh2C02   ppu;
    RicohApu    apu;
    Cartridge   cart;
    Mmc5        mmc5;
    Mapper6     mapper6;
    WinAudioOut audio;

    uint8_t     wram[0x0800];
    uint8_t     ciram[0x1000];

    uint8_t     joy1_state;
    uint8_t     joy1_shift;
    uint8_t     joy2_state;
    uint8_t     joy2_shift;
    bool        joy_strobe;
    bool        joy_strobe_next;
    bool        joy1_oe_this_cycle;
    bool        joy1_oe_prev_cycle;
    bool        joy2_oe_this_cycle;
    bool        joy2_oe_prev_cycle;
    bool        mid_tick_done;
    uint64_t    ppu_last_rd_dot;

    HWND        hwnd;
    bool        running;
    bool        paused;
    bool        muted;
    bool        overscan_crop;
    char        rom_title[128];
} NesEmulator;

static inline bool is_mapper6(const Cartridge *cart) {
    return cart->mapper == 6u || cart->mapper == 8u;
}

/* ==================== Audio (WinMM waveOut) ==================== */

static void audio_init(WinAudioOut *a) {
    memset(a, 0, sizeof(*a));
    WAVEFORMATEX wfx;
    memset(&wfx, 0, sizeof(wfx));
    wfx.wFormatTag      = WAVE_FORMAT_PCM;
    wfx.nChannels       = 1u;
    wfx.nSamplesPerSec  = AUDIO_SAMPLE_RATE;
    wfx.wBitsPerSample  = 16u;
    wfx.nBlockAlign     = 2u;
    wfx.nAvgBytesPerSec = AUDIO_SAMPLE_RATE * 2u;

    if (waveOutOpen(&a->hwo, WAVE_MAPPER, &wfx, 0u, 0u, CALLBACK_NULL) != MMSYSERR_NOERROR) {
        a->hwo = NULL;
        return;
    }

    for (uint32_t i = 0u; i < AUDIO_NUM_BUFS; i++) {
        a->hdrs[i].lpData         = (LPSTR)a->pcm[i];
        a->hdrs[i].dwBufferLength = AUDIO_BUF_SAMPLES * sizeof(int16_t);
        waveOutPrepareHeader(a->hwo, &a->hdrs[i], sizeof(WAVEHDR));
    }
    a->ready = true;
}

static void audio_shutdown(WinAudioOut *a) {
    if (!a->ready || a->hwo == NULL) return;
    waveOutReset(a->hwo);
    for (uint32_t i = 0u; i < AUDIO_NUM_BUFS; i++) {
        waveOutUnprepareHeader(a->hwo, &a->hdrs[i], sizeof(WAVEHDR));
    }
    waveOutClose(a->hwo);
    a->hwo = NULL;
    a->ready = false;
}

static void audio_push_cpu_cycle(NesEmulator *emu) {
    WinAudioOut *a = &emu->audio;
    if (!a->ready) return;

    float s = emu->muted ? 0.0f : ricoh_apu_output(&emu->apu);
    a->sample_sum += (double)s;
    a->sample_cnt++;
    a->cycle_accum += 1.0;

    const double cycles_per_sample = RICOH_APU_NTSC_CLOCK_HZ / (double)AUDIO_SAMPLE_RATE;
    if (a->cycle_accum >= cycles_per_sample) {
        a->cycle_accum -= cycles_per_sample;
        double avg = a->sample_sum / (double)a->sample_cnt;
        a->sample_sum = 0.0;
        a->sample_cnt = 0u;

        double scaled = avg * 24000.0;
        if (scaled > 32767.0)  scaled = 32767.0;
        if (scaled < -32768.0) scaled = -32768.0;

        a->pcm[a->write_buf][a->write_pos++] = (int16_t)scaled;
        if (a->write_pos >= AUDIO_BUF_SAMPLES) {
            WAVEHDR *hdr = &a->hdrs[a->write_buf];
            if ((hdr->dwFlags & WHDR_INQUEUE) == 0u) {
                waveOutWrite(a->hwo, hdr, sizeof(WAVEHDR));
                a->write_buf = (a->write_buf + 1u) % AUDIO_NUM_BUFS;
            }
            a->write_pos = 0u;
        }
    }
}

/* ==================== ROM Parser & Mappers ==================== */

static void cart_free(Cartridge *cart) {
    free(cart->prg_rom);
    free(cart->chr_mem);
    memset(cart, 0, sizeof(*cart));
}

static void cart_reset_mappers(Cartridge *cart) {
    cart->mirroring = cart->initial_mirroring;

    cart->mmc1_shift = 0x10u;
    cart->mmc1_ctrl  = 0x0Cu;
    cart->mmc1_chr0  = 0u;
    cart->mmc1_chr1  = 0u;
    cart->mmc1_prg   = 0u;
    cart->mmc1_last_write_cycle = 0u;

    cart->ux_prg_bank = 0u;
    cart->cn_chr_bank = 0u;

    cart->mmc3_select         = 0u;
    memset(cart->mmc3_regs, 0, sizeof(cart->mmc3_regs));
    cart->mmc3_irq_latch      = 0u;
    cart->mmc3_irq_counter    = 0u;
    cart->mmc3_irq_reload     = false;
    cart->mmc3_irq_enable     = false;
    cart->mmc3_prev_a12       = false;
    cart->mmc3_a12_current    = false;
    cart->mmc3_a12_low_m2     = 3u;
    cart->mmc3_mc_acc_div     = 0u;
    cart->mmc3_prg_ram_enable = cart->prg_ram_present;
    cart->mmc3_prg_ram_ro     = false;
    cart->mmc6_ram_ctrl       = 0u;
}

static bool cart_load_ines(Cartridge *cart, const char *path) {
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
    if (mapper != 0u && mapper != 1u && mapper != 2u &&
        mapper != 3u && mapper != 4u && mapper != 5u &&
        mapper != 6u && mapper != 8u) {
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
        if (is_nes2) {
            uint8_t chr_ram_shift   = (uint8_t)(hdr[11] & 0x0Fu);
            uint8_t chr_nvram_shift = (uint8_t)((hdr[11] >> 4) & 0x0Fu);
            uint32_t total_chr_ram  = (chr_ram_shift   ? (64u << chr_ram_shift)   : 0u) +
                                      (chr_nvram_shift ? (64u << chr_nvram_shift) : 0u);
            chr_size = (total_chr_ram > 0u) ? total_chr_ram : 8192u;
        } else {
            chr_size = 8192u;
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
    cart->initial_mirroring = (flags6 & 0x08u) ? MIRROR_FOUR_SCREEN
                            : (flags6 & 0x01u) ? MIRROR_VERTICAL
                                               : MIRROR_HORIZONTAL;
    cart->mirroring         = cart->initial_mirroring;

    if (is_nes2) {
        uint8_t ram_shift   = (uint8_t)(hdr[10] & 0x0Fu);
        uint8_t nvram_shift = (uint8_t)((hdr[10] >> 4) & 0x0Fu);
        cart->prg_ram_size   = ram_shift   ? (64u << ram_shift)   : 0u;
        cart->prg_nvram_size = nvram_shift ? (64u << nvram_shift) : 0u;
        uint32_t prg_ram_sz  = cart->prg_ram_size + cart->prg_nvram_size;
        cart->prg_ram_present = (mapper != 4u) || (prg_ram_sz > 0u) || (submapper == 1u);
    } else {
        cart->prg_ram_size    = 0u;
        cart->prg_nvram_size  = 0u;
        cart->prg_ram_present = true;
    }

    cart->mmc3_is_mmc6 = (mapper == 4u && is_nes2 && submapper == 1u);
    cart->mmc3_mc_acc  = (mapper == 4u && is_nes2 && submapper == 3u);
    cart->mmc3_rev_a   = (mapper == 4u && is_nes2 && (submapper == 4u || submapper == 1u));

    if (mapper == 8u) {
        cart->m6_init_mode = 4u;
    } else if (mapper == 6u) {
        cart->m6_init_mode = is_nes2 ? (uint8_t)(submapper & 0x07u) : 1u;
    }

    cart_reset_mappers(cart);

    const char *variant = "std";
    if (cart->mapper == 4u) {
        variant = cart->mmc3_is_mmc6 ? "MMC6"
                : cart->mmc3_mc_acc  ? "MC-ACC"
                : cart->mmc3_rev_a   ? "MMC3A"
                                     : "MMC3B/C";
    } else if (cart->mapper == 5u) {
        variant = cart->mmc5_is_5a ? "MMC5A" : "MMC5";
    } else if (is_mapper6(cart)) {
        static const char *const m6_names[8] = {
            "UNROM", "UN1ROM+CHRSW", "UOROM", "Rev-UOROM",
            "GNROM", "CNROM-256", "CNROM-128", "NROM-256"
        };
        variant = m6_names[cart->m6_init_mode & 0x07u];
    }

    printf("Loaded ROM: %s | Mapper %u.%u (%s) | PRG %u KB | CHR %u KB (%s) | Mirror %u%s\n",
           path, (unsigned)cart->mapper, (unsigned)cart->submapper,
           variant,
           (unsigned)(cart->prg_size / 1024u),
           (unsigned)(cart->chr_size / 1024u), cart->chr_is_ram ? "RAM" : "ROM",
           (unsigned)cart->mirroring,
           cart->has_trainer ? " | Trainer" : "");
    return true;
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

static uint32_t mmc3_chr_offset(const Cartridge *cart, uint16_t addr) {
    uint32_t banks_1k = cart->chr_size / 1024u;
    if (banks_1k == 0u) return addr % cart->chr_size;

    uint8_t slot = (uint8_t)((addr >> 10) & 0x07u);
    if (cart->mmc3_select & 0x80u) slot ^= 0x04u;

    uint32_t bank = 0u;
    switch (slot) {
        case 0: bank = (uint32_t)(cart->mmc3_regs[0] & 0xFEu); break;
        case 1: bank = (uint32_t)(cart->mmc3_regs[0] | 0x01u); break;
        case 2: bank = (uint32_t)(cart->mmc3_regs[1] & 0xFEu); break;
        case 3: bank = (uint32_t)(cart->mmc3_regs[1] | 0x01u); break;
        case 4: bank = cart->mmc3_regs[2]; break;
        case 5: bank = cart->mmc3_regs[3]; break;
        case 6: bank = cart->mmc3_regs[4]; break;
        case 7: bank = cart->mmc3_regs[5]; break;
    }
    return ((bank % banks_1k) * 1024u) | (addr & 0x03FFu);
}

static uint32_t map_chr_addr(const Cartridge *cart, uint16_t addr) {
    switch (cart->mapper) {
        case 1: {
            uint32_t banks_4k = cart->chr_size / 4096u;
            if (banks_4k == 0u) return addr % cart->chr_size;
            if ((cart->mmc1_ctrl & 0x10u) == 0u) {
                uint32_t bank4k = (uint32_t)(cart->mmc1_chr0 & 0x1Eu) + ((addr >= 0x1000u) ? 1u : 0u);
                return ((bank4k % banks_4k) * 4096u) | (addr & 0x0FFFu);
            } else {
                uint32_t bank4k = (addr < 0x1000u) ? cart->mmc1_chr0 : cart->mmc1_chr1;
                return ((bank4k % banks_4k) * 4096u) | (addr & 0x0FFFu);
            }
        }
        case 3: {
            uint32_t banks_8k = cart->chr_size / 8192u;
            if (banks_8k == 0u) return addr % cart->chr_size;
            uint32_t b = (uint32_t)cart->cn_chr_bank % banks_8k;
            return (b * 8192u) | (addr & 0x1FFFu);
        }
        case 4:
            return mmc3_chr_offset(cart, addr);
        default:
            return addr % cart->chr_size;
    }
}

static uint8_t cart_prg_read(const Cartridge *cart, uint16_t addr, uint8_t open_bus, bool *driven_out) {
    if (addr >= 0x6000u && addr < 0x8000u) {
        if (cart->mapper == 4u) {
            if (!cart->prg_ram_present) {
                if (driven_out) *driven_out = false;
                return open_bus;
            }
            if (cart->mmc3_is_mmc6) {
                if ((cart->mmc3_select & 0x20u) == 0u || addr < 0x7000u) {
                    if (driven_out) *driven_out = false;
                    return open_bus;
                }
                bool low_r  = (cart->mmc6_ram_ctrl & 0x20u) != 0u;
                bool high_r = (cart->mmc6_ram_ctrl & 0x80u) != 0u;
                if (!low_r && !high_r) {
                    if (driven_out) *driven_out = false;
                    return open_bus;
                }
                bool in_high = (addr & 0x0200u) != 0u;
                if (driven_out) *driven_out = true;
                if ((!in_high && !low_r) || (in_high && !high_r)) {
                    return 0x00u;
                }
                return cart->prg_ram[addr & 0x03FFu];
            }
            if (!cart->mmc3_prg_ram_enable) {
                if (driven_out) *driven_out = false;
                return open_bus;
            }
        }
        if (driven_out) *driven_out = true;
        return cart->prg_ram[addr - 0x6000u];
    }
    if (addr < 0x8000u) {
        if (driven_out) *driven_out = false;
        return open_bus;
    }

    if (driven_out) *driven_out = true;
    uint32_t banks_16k = cart->prg_size / 16384u;
    switch (cart->mapper) {
        case 0:
        case 3:
            return cart->prg_rom[(addr - 0x8000u) % cart->prg_size];

        case 1: {
            if (banks_16k == 0u) return 0xFFu;
            uint8_t mode = (uint8_t)((cart->mmc1_ctrl >> 2) & 0x03u);
            uint32_t bank = 0u;
            if (mode <= 1u) {
                bank = (uint32_t)(cart->mmc1_prg & 0x0Eu) + ((addr >= 0xC000u) ? 1u : 0u);
            } else if (mode == 2u) {
                bank = (addr < 0xC000u) ? 0u : (uint32_t)cart->mmc1_prg;
            } else {
                bank = (addr < 0xC000u) ? (uint32_t)cart->mmc1_prg : (banks_16k - 1u);
            }
            return cart->prg_rom[((bank % banks_16k) * 16384u) | (addr & 0x3FFFu)];
        }

        case 2: {
            if (banks_16k == 0u) return 0xFFu;
            uint32_t bank = (addr < 0xC000u) ? ((uint32_t)cart->ux_prg_bank % banks_16k)
                                             : (banks_16k - 1u);
            return cart->prg_rom[(bank * 16384u) | (addr & 0x3FFFu)];
        }

        case 4: {
            uint32_t banks_8k = cart->prg_size / 8192u;
            if (banks_8k < 2u) return cart->prg_rom[(addr - 0x8000u) % cart->prg_size];
            uint8_t slot = (uint8_t)((addr - 0x8000u) >> 13);
            uint32_t bank = 0u;
            bool prg_mode = (cart->mmc3_select & 0x40u) != 0u;
            if (slot == 0u)      bank = prg_mode ? (banks_8k - 2u) : cart->mmc3_regs[6];
            else if (slot == 1u) bank = cart->mmc3_regs[7];
            else if (slot == 2u) bank = prg_mode ? cart->mmc3_regs[6] : (banks_8k - 2u);
            else                 bank = banks_8k - 1u;
            return cart->prg_rom[((bank % banks_8k) * 8192u) | (addr & 0x1FFFu)];
        }
    }
    return 0xFFu;
}

static void cart_prg_write(NesEmulator *emu, uint16_t addr, uint8_t val) {
    Cartridge *cart = &emu->cart;

    if (addr >= 0x6000u && addr < 0x8000u) {
        if (cart->mapper == 4u) {
            if (!cart->prg_ram_present) {
                return;
            }
            if (cart->mmc3_is_mmc6) {
                if ((cart->mmc3_select & 0x20u) == 0u || addr < 0x7000u) {
                    return;
                }
                bool in_high = (addr & 0x0200u) != 0u;
                bool can_w = in_high ? ((cart->mmc6_ram_ctrl & 0xC0u) == 0xC0u)
                                     : ((cart->mmc6_ram_ctrl & 0x30u) == 0x30u);
                if (can_w) {
                    cart->prg_ram[addr & 0x03FFu] = val;
                }
                return;
            }
            if (!cart->mmc3_prg_ram_enable || cart->mmc3_prg_ram_ro) {
                return;
            }
        }
        cart->prg_ram[addr - 0x6000u] = val;
        return;
    }
    if (addr < 0x8000u) {
        return;
    }

    switch (cart->mapper) {
        case 1: {
            if ((emu->cpu.cycles - cart->mmc1_last_write_cycle) <= 1u) {
                cart->mmc1_last_write_cycle = emu->cpu.cycles;
                return;
            }
            cart->mmc1_last_write_cycle = emu->cpu.cycles;

            if (val & 0x80u) {
                cart->mmc1_shift = 0x10u;
                cart->mmc1_ctrl |= 0x0Cu;
                return;
            }
            bool complete = (cart->mmc1_shift & 0x01u) != 0u;
            cart->mmc1_shift = (uint8_t)((cart->mmc1_shift >> 1) | ((val & 0x01u) << 4));
            if (complete) {
                uint8_t reg = (uint8_t)((addr >> 13) & 0x03u);
                switch (reg) {
                    case 0:
                        cart->mmc1_ctrl = cart->mmc1_shift;
                        switch (cart->mmc1_ctrl & 0x03u) {
                            case 0: cart->mirroring = MIRROR_SINGLE_LO; break;
                            case 1: cart->mirroring = MIRROR_SINGLE_HI; break;
                            case 2: cart->mirroring = MIRROR_VERTICAL;  break;
                            case 3: cart->mirroring = MIRROR_HORIZONTAL; break;
                        }
                        break;
                    case 1: cart->mmc1_chr0 = cart->mmc1_shift; break;
                    case 2: cart->mmc1_chr1 = cart->mmc1_shift; break;
                    case 3: cart->mmc1_prg  = (uint8_t)(cart->mmc1_shift & 0x0Fu); break;
                }
                cart->mmc1_shift = 0x10u;
            }
            break;
        }
        case 2: cart->ux_prg_bank = val; break;
        case 3: cart->cn_chr_bank = val; break;
        case 4: {
            bool even = (addr & 1u) == 0u;
            if (addr < 0xA000u) {
                if (even) {
                    cart->mmc3_select = val;
                    if (cart->mmc3_is_mmc6 && (val & 0x20u) == 0u) {
                        cart->mmc6_ram_ctrl = 0u;
                    }
                } else {
                    uint8_t reg = (uint8_t)(cart->mmc3_select & 0x07u);
                    if (reg == 6u || reg == 7u) val &= 0x3Fu;
                    cart->mmc3_regs[reg] = val;
                }
            } else if (addr < 0xC000u) {
                if (even) {
                    if (cart->mirroring != MIRROR_FOUR_SCREEN) {
                        cart->mirroring = (val & 0x01u) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
                    }
                } else {
                    if (cart->mmc3_is_mmc6) {
                        if (cart->mmc3_select & 0x20u) {
                            cart->mmc6_ram_ctrl = (uint8_t)(val & 0xF0u);
                        }
                    } else {
                        cart->mmc3_prg_ram_enable = (val & 0x80u) != 0u;
                        cart->mmc3_prg_ram_ro     = (val & 0x40u) != 0u;
                    }
                }
            } else if (addr < 0xE000u) {
                if (even) {
                    cart->mmc3_irq_latch = val;
                } else {
                    cart->mmc3_irq_counter = 0u;
                    cart->mmc3_irq_reload  = true;
                    if (cart->mmc3_mc_acc) {
                        cart->mmc3_mc_acc_div = 0u;
                    }
                }
            } else {
                if (even) {
                    cart->mmc3_irq_enable = false;
                    ricoh_2a03_set_irq(&emu->cpu, 0x04u, false);
                } else {
                    cart->mmc3_irq_enable = true;
                }
            }
            break;
        }
        default:
            break;
    }
}

/* ==================== PPU, APU, DMA & CPU Bus ==================== */

static void mmc3_clock_irq_counter(NesEmulator *emu) {
    Cartridge *cart = &emu->cart;

    if (cart->mmc3_irq_reload) {
        cart->mmc3_irq_counter = cart->mmc3_irq_latch;
        cart->mmc3_irq_reload  = false;
        if (cart->mmc3_irq_counter == 0u && cart->mmc3_irq_enable) {
            ricoh_2a03_set_irq(&emu->cpu, 0x04u, true);
        }
    } else if (cart->mmc3_irq_counter == 0u) {
        cart->mmc3_irq_counter = cart->mmc3_irq_latch;
        if (!cart->mmc3_rev_a && cart->mmc3_irq_counter == 0u && cart->mmc3_irq_enable) {
            ricoh_2a03_set_irq(&emu->cpu, 0x04u, true);
        }
    } else {
        cart->mmc3_irq_counter--;
        if (cart->mmc3_irq_counter == 0u && cart->mmc3_irq_enable) {
            ricoh_2a03_set_irq(&emu->cpu, 0x04u, true);
        }
    }
}

static void mmc3_on_m2_falling(NesEmulator *emu) {
    Cartridge *cart = &emu->cart;
    if (cart->mapper != 4u || cart->mmc3_mc_acc) return;

    if (!cart->mmc3_a12_current) {
        if (cart->mmc3_a12_low_m2 < 3u) {
            cart->mmc3_a12_low_m2++;
        }
    } else {
        cart->mmc3_a12_low_m2 = 0u;
    }
}

static void mmc3_watch_a12(NesEmulator *emu, uint16_t addr) {
    Cartridge *cart = &emu->cart;
    if (cart->mapper != 4u) return;

    bool a12 = (addr & 0x1000u) != 0u;
    if (cart->mmc3_mc_acc) {
        if (a12 && !cart->mmc3_prev_a12) {
            if (cart->mmc3_mc_acc_div == 0u) {
                mmc3_clock_irq_counter(emu);
            }
            cart->mmc3_mc_acc_div = (uint8_t)((cart->mmc3_mc_acc_div + 1u) & 0x07u);
        }
    } else {
        if (a12) {
            if (!cart->mmc3_a12_current && cart->mmc3_a12_low_m2 >= 3u) {
                mmc3_clock_irq_counter(emu);
            }
            cart->mmc3_a12_low_m2 = 0u;
        }
        cart->mmc3_a12_current = a12;
    }
    cart->mmc3_prev_a12 = a12;
}

static inline void ppu_step_and_watch(NesEmulator *emu) {
    ricoh_2c02_step(&emu->ppu);
    if (emu->cart.mapper == 4u) {
        mmc3_watch_a12(emu, emu->ppu.vbus);
    } else if (is_mapper6(&emu->cart)) {
        mapper6_watch_a12(&emu->mapper6, emu->ppu.vbus);
    }
}

static uint8_t ppu_bus_read(void *ctx, uint16_t addr) {
    NesEmulator *emu = (NesEmulator *)ctx;
    addr &= 0x3FFFu;

    if (emu->cart.mapper == 5u) {
        bool count_edge = !(emu->ppu.pd_rb && emu->ppu_last_rd_dot == emu->ppu.total_dots);
        emu->ppu_last_rd_dot = emu->ppu.total_dots;
        return mmc5_ppu_read(&emu->mmc5, addr, count_edge, emu->ciram);
    }
    if (is_mapper6(&emu->cart)) {
        bool count_edge = !(emu->ppu.pd_rb && emu->ppu_last_rd_dot == emu->ppu.total_dots);
        emu->ppu_last_rd_dot = emu->ppu.total_dots;
        return mapper6_ppu_read(&emu->mapper6, addr, count_edge, emu->ciram);
    }

    if (addr < 0x2000u) {
        return emu->cart.chr_mem[map_chr_addr(&emu->cart, addr)];
    }
    return emu->ciram[mirror_nametable_addr(emu->cart.mirroring, addr)];
}

static void ppu_bus_write(void *ctx, uint16_t addr, uint8_t val) {
    NesEmulator *emu = (NesEmulator *)ctx;
    addr &= 0x3FFFu;

    if (emu->cart.mapper == 5u) {
        mmc5_ppu_write(&emu->mmc5, addr, val, emu->ciram);
        return;
    }
    if (is_mapper6(&emu->cart)) {
        mapper6_ppu_write(&emu->mapper6, addr, val, emu->ciram);
        return;
    }

    if (addr < 0x2000u) {
        if (emu->cart.chr_is_ram) {
            emu->cart.chr_mem[map_chr_addr(&emu->cart, addr)] = val;
        }
    } else {
        emu->ciram[mirror_nametable_addr(emu->cart.mirroring, addr)] = val;
    }
}

static void ppu_on_nmi(void *ctx, bool asserted) {
    NesEmulator *emu = (NesEmulator *)ctx;
    ricoh_2a03_set_nmi(&emu->cpu, asserted);
}

static void apu_on_irq(void *ctx, uint8_t source_mask, bool asserted) {
    NesEmulator *emu = (NesEmulator *)ctx;
    ricoh_2a03_set_irq(&emu->cpu, source_mask, asserted);
}

static void mapper_on_irq(void *ctx, uint8_t source_mask, bool asserted) {
    NesEmulator *emu = (NesEmulator *)ctx;
    ricoh_2a03_set_irq(&emu->cpu, source_mask, asserted);
}

static uint8_t read_controller_bit(NesEmulator *emu, uint8_t port) {
    if (port == 0u) {
        emu->joy1_oe_this_cycle = true;
        return emu->joy_strobe ? (uint8_t)(emu->joy1_state & 0x01u)
                               : (uint8_t)(emu->joy1_shift & 0x01u);
    } else {
        emu->joy2_oe_this_cycle = true;
        return emu->joy_strobe ? (uint8_t)(emu->joy2_state & 0x01u)
                               : (uint8_t)(emu->joy2_shift & 0x01u);
    }
}

static uint8_t ppu_read_through_dot1(NesEmulator *emu, uint16_t addr) {
    uint8_t pre_val = 0u;
    ricoh_2c02_read_begin(&emu->ppu, addr, &pre_val);
    ppu_step_and_watch(emu);
    emu->mid_tick_done = true;
    return ricoh_2c02_read_end(&emu->ppu, addr, pre_val);
}

static uint8_t cpu_bus_read(void *ctx, uint16_t addr) {
    NesEmulator *emu = (NesEmulator *)ctx;
    if (addr < 0x2000u) {
        return emu->wram[addr & 0x07FFu];
    }
    if (addr < 0x4000u) {
        uint8_t val = ppu_read_through_dot1(emu, addr);
        if (emu->cart.mapper == 5u) {
            mmc5_snoop_cpu_read(&emu->mmc5, addr, val);
        }
        return val;
    }
    if (addr == 0x4015u) {
        return ricoh_apu_read_status(&emu->apu, emu->cpu.internal_bus);
    }
    if (addr == 0x4016u) {
        return (uint8_t)((emu->cpu.data_bus & 0xE0u) | read_controller_bit(emu, 0u));
    }
    if (addr == 0x4017u) {
        return (uint8_t)((emu->cpu.data_bus & 0xE0u) | read_controller_bit(emu, 1u));
    }
    if (addr >= 0x4020u && is_mapper6(&emu->cart)) {
        return mapper6_cpu_read(&emu->mapper6, addr, emu->cpu.data_bus, NULL);
    }
    if (addr < 0x5000u) {
        return emu->cpu.data_bus;
    }
    if (addr < 0x6000u) {
        if (emu->cart.mapper == 5u) {
            return mmc5_cpu_read(&emu->mmc5, addr, emu->cpu.data_bus, NULL);
        }
        return emu->cpu.data_bus;
    }
    if (emu->cart.mapper == 5u) {
        return mmc5_cpu_read(&emu->mmc5, addr, emu->cpu.data_bus, NULL);
    }
    return cart_prg_read(&emu->cart, addr, emu->cpu.data_bus, NULL);
}

static void cpu_bus_write(void *ctx, uint16_t addr, uint8_t val) {
    NesEmulator *emu = (NesEmulator *)ctx;
    ricoh_apu_notify_cpu_write(&emu->apu);

    if (addr < 0x2000u) {
        emu->wram[addr & 0x07FFu] = val;
    } else if (addr < 0x4000u) {
        ricoh_2c02_write(&emu->ppu, addr, val);
        if (emu->cart.mapper == 5u) {
            mmc5_snoop_cpu_write(&emu->mmc5, addr, val);
        } else if (is_mapper6(&emu->cart)) {
            mapper6_snoop_cpu_write(&emu->mapper6, addr, val);
        }
    } else if (addr == 0x4014u) {
        ricoh_apu_trigger_oam_dma(&emu->apu, val);
        if (emu->cart.mapper == 5u) {
            mmc5_snoop_cpu_write(&emu->mmc5, addr, val);
        } else if (is_mapper6(&emu->cart)) {
            mapper6_snoop_cpu_write(&emu->mapper6, addr, val);
        }
    } else if (addr == 0x4016u) {
        emu->joy_strobe_next = (val & 0x01u) != 0u;
    } else if ((addr >= 0x4000u && addr <= 0x4013u) || addr == 0x4015u || addr == 0x4017u) {
        ricoh_apu_write(&emu->apu, addr, val);
        if (is_mapper6(&emu->cart)) {
            mapper6_snoop_cpu_write(&emu->mapper6, addr, val);
        }
    } else if (addr >= 0x4020u && is_mapper6(&emu->cart)) {
        mapper6_cpu_write(&emu->mapper6, addr, val);
    } else if (addr >= 0x5000u) {
        if (emu->cart.mapper == 5u) {
            mmc5_cpu_write(&emu->mmc5, addr, val);
        } else if (addr >= 0x6000u) {
            cart_prg_write(emu, addr, val);
        }
    }
}

static void clock_joypads_end_of_cycle(NesEmulator *emu) {
    if (!emu->joy_strobe) {
        if (emu->joy1_oe_prev_cycle && !emu->joy1_oe_this_cycle) {
            emu->joy1_shift = (uint8_t)((emu->joy1_shift >> 1) | 0x80u);
        }
        if (emu->joy2_oe_prev_cycle && !emu->joy2_oe_this_cycle) {
            emu->joy2_shift = (uint8_t)((emu->joy2_shift >> 1) | 0x80u);
        }
    }
    if (!emu->apu.put) {
        emu->joy_strobe = emu->joy_strobe_next;
        if (emu->joy_strobe) {
            emu->joy1_shift = emu->joy1_state;
            emu->joy2_shift = emu->joy2_state;
        }
    }
}

static void cpu_bus_tick_pre(void *ctx) {
    NesEmulator *emu = (NesEmulator *)ctx;
    emu->mid_tick_done = false;
    emu->joy1_oe_prev_cycle = emu->joy1_oe_this_cycle;
    emu->joy1_oe_this_cycle = false;
    emu->joy2_oe_prev_cycle = emu->joy2_oe_this_cycle;
    emu->joy2_oe_this_cycle = false;
    ppu_step_and_watch(emu);
    if (emu->cart.mapper == 5u) {
        mmc5_cpu_cycle_start(&emu->mmc5);
        ricoh_apu_set_ext_audio(&emu->apu, mmc5_audio_output(&emu->mmc5));
    } else if (is_mapper6(&emu->cart)) {
        mapper6_cpu_cycle(&emu->mapper6);
    }
    ricoh_apu_step(&emu->apu);
    audio_push_cpu_cycle(emu);
}

static void cpu_bus_tick_mid(void *ctx) {
    NesEmulator *emu = (NesEmulator *)ctx;
    if (emu->mid_tick_done) {
        emu->mid_tick_done = false;
        return;
    }
    ppu_step_and_watch(emu);
}

static void cpu_bus_tick(void *ctx) {
    NesEmulator *emu = (NesEmulator *)ctx;
    ppu_step_and_watch(emu);
    mmc3_on_m2_falling(emu);
    clock_joypads_end_of_cycle(emu);
}

static uint8_t dma_bus_read(NesEmulator *emu, uint16_t dma_addr, uint16_t halted_addr) {
    uint8_t ext = emu->cpu.data_bus;
    bool ext_driven = false;

    if (dma_addr < 0x2000u) {
        ext = emu->wram[dma_addr & 0x07FFu];
        ext_driven = true;
    } else if (dma_addr < 0x4000u) {
        ext = ppu_read_through_dot1(emu, dma_addr);
        if (emu->cart.mapper == 5u) {
            mmc5_snoop_cpu_read(&emu->mmc5, dma_addr, ext);
        }
        ext_driven = true;
    } else if (dma_addr >= 0x4020u && is_mapper6(&emu->cart)) {
        ext = mapper6_cpu_read(&emu->mapper6, dma_addr, emu->cpu.data_bus, &ext_driven);
    } else if (dma_addr >= 0x5000u) {
        if (emu->cart.mapper == 5u) {
            ext = mmc5_cpu_read(&emu->mmc5, dma_addr, emu->cpu.data_bus, &ext_driven);
        } else if (dma_addr >= 0x6000u) {
            ext = cart_prg_read(&emu->cart, dma_addr, emu->cpu.data_bus, &ext_driven);
        }
    }

    emu->cpu.data_bus = ext;

    if ((halted_addr & 0xFFE0u) == 0x4000u && (dma_addr < 0x2000u || dma_addr >= 0x4000u)) {
        uint8_t reg = (uint8_t)(dma_addr & 0x1Fu);
        if (reg == 0x15u) {
            uint8_t status = ricoh_apu_read_status(&emu->apu, emu->cpu.internal_bus);
            emu->cpu.internal_bus = status;
            return status;
        } else if (reg == 0x16u || reg == 0x17u) {
            uint8_t bit = read_controller_bit(emu, (uint8_t)(reg - 0x16u));
            uint8_t ctrl = (uint8_t)((emu->cpu.data_bus & 0xE0u) | bit);
            if (emu->apu.oam_dma_active && ext_driven) {
                emu->cpu.internal_bus = ext;
                return ext;
            }
            emu->cpu.data_bus = ctrl;
            emu->cpu.internal_bus = ctrl;
            return ctrl;
        }
    }

    emu->cpu.internal_bus = emu->cpu.data_bus;
    return emu->cpu.data_bus;
}

static bool cpu_bus_dma_cycle(void *ctx, uint16_t halted_addr) {
    NesEmulator *emu = (NesEmulator *)ctx;
    RicohApu *apu = &emu->apu;

    if (!ricoh_apu_dma_wants_halt(apu)) {
        return false;
    }

    bool dmc_was_pending = apu->dmc_dma_pending ||
                           (apu->dmc_load_delay == 1u &&
                            apu->dmc.buffer_empty &&
                            apu->dmc.bytes_remaining > 0u);
    bool dmc_was_abort_one = apu->dmc_dma_abort_one;

    cpu_bus_tick_pre(emu);

    bool is_get = !apu->put;
    bool handled_bus = false;
    bool advance_from_halt = false;

    if (dmc_was_abort_one && apu->dmc_dma_phase == APU_DMC_DMA_IDLE && !is_get) {
        apu->dmc_dma_abort_one = false;
        apu->dmc_dma_pending   = false;
    } else if (dmc_was_pending && apu->dmc_dma_phase == APU_DMC_DMA_IDLE) {
        apu->dmc_dma_pending = false;
        apu->dmc_dma_phase   = APU_DMC_DMA_HALT;
        advance_from_halt    = true;
    }

    if (apu->dmc_dma_phase == APU_DMC_DMA_GET && is_get) {
        uint8_t dmc_val = dma_bus_read(emu, apu->dmc.current_addr, halted_addr);
        ricoh_apu_dmc_finish_dma(apu, dmc_val);
        apu->dmc_dma_phase = APU_DMC_DMA_IDLE;
        handled_bus = true;
    }

    if (apu->oam_dma_active) {
        if (apu->oam_dma_halt) {
            apu->oam_dma_halt = false;
        } else if (!handled_bus) {
            if (!apu->oam_dma_put_phase && is_get) {
                uint16_t src = (uint16_t)(((uint16_t)apu->oam_dma_page << 8) | apu->oam_dma_index);
                apu->oam_dma_val = dma_bus_read(emu, src, halted_addr);
                apu->oam_dma_put_phase = true;
                handled_bus = true;
            } else if (apu->oam_dma_put_phase && !is_get) {
                emu->cpu.data_bus = apu->oam_dma_val;
                emu->cpu.internal_bus = apu->oam_dma_val;
                ricoh_2c02_write(&emu->ppu, 0x2004u, apu->oam_dma_val);
                if (emu->cart.mapper == 5u) {
                    mmc5_snoop_cpu_write(&emu->mmc5, 0x2004u, apu->oam_dma_val);
                } else if (is_mapper6(&emu->cart)) {
                    mapper6_snoop_cpu_write(&emu->mapper6, 0x2004u, apu->oam_dma_val);
                }
                apu->oam_dma_index++;
                apu->oam_dma_put_phase = false;
                if (apu->oam_dma_index == 0u) {
                    apu->oam_dma_active = false;
                }
                handled_bus = true;
            }
        }
    }

    if (!handled_bus) {
        (void)dma_bus_read(emu, halted_addr, halted_addr);
    }

    if (advance_from_halt) {
        apu->dmc_dma_phase = APU_DMC_DMA_DUMMY;
    } else if (apu->dmc_dma_phase == APU_DMC_DMA_DUMMY) {
        apu->dmc_dma_phase = APU_DMC_DMA_GET;
    }

    cpu_bus_tick_mid(emu);
    ricoh_2a03_sample_nmi(&emu->cpu);
    cpu_bus_tick(emu);
    return true;
}

static void emu_step_frame(NesEmulator *emu) {
    emu->ppu.frame_ready = false;
    while (!emu->ppu.frame_ready && emu->running) {
        ricoh_2a03_step(&emu->cpu);
    }
}

static void emu_power_on(NesEmulator *emu) {
    memset(emu->wram, 0, sizeof(emu->wram));
    memset(emu->ciram, 0, sizeof(emu->ciram));
    emu->joy1_state         = 0u;
    emu->joy1_shift         = 0u;
    emu->joy2_state         = 0u;
    emu->joy2_shift         = 0u;
    emu->joy_strobe         = false;
    emu->joy_strobe_next    = false;
    emu->joy1_oe_this_cycle = false;
    emu->joy1_oe_prev_cycle = false;
    emu->joy2_oe_this_cycle = false;
    emu->joy2_oe_prev_cycle = false;
    emu->mid_tick_done      = false;
    emu->ppu_last_rd_dot    = UINT64_MAX;
    cart_reset_mappers(&emu->cart);

    if (emu->cart.has_trainer && !is_mapper6(&emu->cart)) {
        memcpy(&emu->cart.prg_ram[0x1000u], emu->cart.trainer, 512u);
    }

    if (emu->cart.mapper == 5u) {
        Mmc5Config mcfg = {
            .prg_rom        = emu->cart.prg_rom,
            .prg_size       = emu->cart.prg_size,
            .chr_mem        = emu->cart.chr_mem,
            .chr_size       = emu->cart.chr_size,
            .chr_is_ram     = emu->cart.chr_is_ram,
            .prg_ram_size   = emu->cart.prg_ram_size,
            .prg_nvram_size = emu->cart.prg_nvram_size,
            .is_nes2        = emu->cart.is_nes2,
            .is_mmc5a       = emu->cart.mmc5_is_5a,
            .sl_mode        = emu->cart.mmc5_sl_mode,
        };
        Mmc5Bus mbus = {
            .ctx    = emu,
            .on_irq = mapper_on_irq,
        };
        mmc5_power_on(&emu->mmc5, &mcfg, &mbus);
    } else if (is_mapper6(&emu->cart)) {
        Mapper6Config m6cfg = {
            .prg_rom            = emu->cart.prg_rom,
            .prg_size           = emu->cart.prg_size,
            .chr_rom            = emu->cart.chr_mem,
            .chr_size           = emu->cart.chr_size,
            .chr_is_ram         = emu->cart.chr_is_ram,
            .trainer            = emu->cart.trainer,
            .has_trainer        = emu->cart.has_trainer,
            .submapper          = emu->cart.m6_init_mode,
            .vertical_mirroring = (emu->cart.initial_mirroring == MIRROR_VERTICAL),
        };
        Mapper6Bus m6bus = {
            .ctx    = emu,
            .on_irq = mapper_on_irq,
        };
        mapper6_power_on(&emu->mapper6, &m6cfg, &m6bus);
    }

    Ricoh2C02Bus pbus = {
        .ctx = emu,
        .read = ppu_bus_read,
        .write = ppu_bus_write,
        .on_nmi = ppu_on_nmi,
    };
    ricoh_2c02_power_on(&emu->ppu, &pbus);

    RicohApuBus abus = {
        .ctx = emu,
        .on_irq = apu_on_irq,
    };
    ricoh_apu_power_on(&emu->apu, &abus);

    Ricoh2A03Bus cbus = {
        .ctx = emu,
        .read = cpu_bus_read,
        .write = cpu_bus_write,
        .tick_pre = cpu_bus_tick_pre,
        .tick_mid = cpu_bus_tick_mid,
        .tick = cpu_bus_tick,
        .dma_cycle = cpu_bus_dma_cycle,
    };
    ricoh_2a03_power_on(&emu->cpu, &cbus);
}

static void emu_reset(NesEmulator *emu) {
    emu->mid_tick_done = false;
    emu->ppu_last_rd_dot = UINT64_MAX;
    cart_reset_mappers(&emu->cart);
    if (emu->cart.mapper == 5u) {
        mmc5_reset(&emu->mmc5);
    } else if (is_mapper6(&emu->cart)) {
        mapper6_reset(&emu->mapper6);
    } else {
        ricoh_2a03_set_irq(&emu->cpu, 0x04u, false);
    }
    ricoh_apu_reset(&emu->apu);
    ricoh_2a03_reset(&emu->cpu);
    ricoh_2c02_reset(&emu->ppu);

    bool a12 = (emu->ppu.vbus & 0x1000u) != 0u;
    emu->cart.mmc3_a12_current = a12;
    emu->cart.mmc3_prev_a12    = a12;
    emu->cart.mmc3_a12_low_m2  = a12 ? 0u : 3u;
}

/* ==================== Win32 GUI (256 x 240 chuan NES) ==================== */

static void update_window_title(NesEmulator *emu) {
    if (!emu->hwnd) return;
    char title[256];
    const char *res = emu->overscan_crop ? "256x224" : "256x240";
    if (emu->cart.mapper == 4u) {
        const char *tag = emu->cart.mmc3_is_mmc6 ? "MMC6"
                        : emu->cart.mmc3_mc_acc  ? "MC-ACC"
                        : emu->cart.mmc3_rev_a   ? "MMC3A"
                                                 : "MMC3B";
        snprintf(title, sizeof(title), "Hydromulator [%s] - %s [%s | F6]",
                 res, emu->rom_title, tag);
    } else if (emu->cart.mapper == 5u) {
        const char *chip = emu->mmc5.is_mmc5a ? "MMC5A" : "MMC5";
        const char *pin  = emu->mmc5.sl_mode  ? "SL"    : "CL";
        snprintf(title, sizeof(title), "Hydromulator [%s] - %s [%s/%s | F6/F7]",
                 res, emu->rom_title, chip, pin);
    } else if (is_mapper6(&emu->cart)) {
        static const char *const m6_names[8] = {
            "0:UNROM", "1:UN1ROM", "2:UOROM", "3:RevUOROM",
            "4:GNROM", "5:CNROM256", "6:CNROM128", "7:NROM256"
        };
        snprintf(title, sizeof(title), "Hydromulator [%s] - %s [FFE %s | F6]",
                 res, emu->rom_title, m6_names[emu->cart.m6_init_mode & 0x07u]);
    } else {
        snprintf(title, sizeof(title), "Hydromulator [%s] - %s", res, emu->rom_title);
    }
    SetWindowTextA(emu->hwnd, title);
}

static void blit_framebuffer(HWND hwnd, const Ricoh2C02 *ppu, HDC hdc_opt, bool crop_overscan) {
    HDC hdc = hdc_opt ? hdc_opt : GetDC(hwnd);
    if (!hdc) return;

    RECT client;
    GetClientRect(hwnd, &client);
    int dst_w = client.right - client.left;
    int dst_h = client.bottom - client.top;

    int src_h = crop_overscan ? 224 : (int)RICOH_2C02_HEIGHT;
    const uint32_t *src_ptr = crop_overscan
        ? (ppu->framebuffer + 8u * RICOH_2C02_WIDTH)
        : ppu->framebuffer;

    BITMAPINFO bmi;
    memset(&bmi, 0, sizeof(bmi));
    bmi.bmiHeader.biSize        = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biWidth       = (LONG)RICOH_2C02_WIDTH;
    bmi.bmiHeader.biHeight      = -(LONG)src_h;
    bmi.bmiHeader.biPlanes      = 1;
    bmi.bmiHeader.biBitCount    = 32;
    bmi.bmiHeader.biCompression = BI_RGB;

    SetStretchBltMode(hdc, COLORONCOLOR);
    StretchDIBits(hdc,
                  0, 0, dst_w, dst_h,
                  0, 0, (int)RICOH_2C02_WIDTH, src_h,
                  src_ptr, &bmi, DIB_RGB_COLORS, SRCCOPY);

    if (!hdc_opt) ReleaseDC(hwnd, hdc);
}

static void resize_client_scale(HWND hwnd, int scale, bool crop_overscan) {
    DWORD style = (DWORD)GetWindowLongPtrA(hwnd, GWL_STYLE);
    int h = crop_overscan ? 224 : (int)RICOH_2C02_HEIGHT;
    RECT rc = { 0, 0, (LONG)RICOH_2C02_WIDTH * scale, (LONG)h * scale };
    AdjustWindowRectEx(&rc, style, FALSE, 0);
    SetWindowPos(hwnd, NULL, 0, 0, rc.right - rc.left, rc.bottom - rc.top,
                 SWP_NOMOVE | SWP_NOZORDER);
}

static void update_joypad_key(NesEmulator *emu, WPARAM vk, bool pressed) {
    uint8_t mask = 0u;
    switch (vk) {
        case 'Z': case 'J':           mask = 0x01u; break;
        case 'X': case 'K':           mask = 0x02u; break;
        case VK_SHIFT: case VK_BACK:  mask = 0x04u; break;
        case VK_RETURN:               mask = 0x08u; break;
        case VK_UP:    case 'W':      mask = 0x10u; break;
        case VK_DOWN:  case 'S':      mask = 0x20u; break;
        case VK_LEFT:  case 'A':      mask = 0x40u; break;
        case VK_RIGHT: case 'D':      mask = 0x80u; break;
        default: break;
    }
    if (mask != 0u) {
        if (pressed) emu->joy1_state |= mask;
        else         emu->joy1_state &= (uint8_t)(~mask);
    }
}

static LRESULT CALLBACK wnd_proc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    NesEmulator *emu = (NesEmulator *)GetWindowLongPtrA(hwnd, GWLP_USERDATA);

    switch (msg) {
        case WM_CREATE: {
            CREATESTRUCTA *cs = (CREATESTRUCTA *)lParam;
            SetWindowLongPtrA(hwnd, GWLP_USERDATA, (LONG_PTR)cs->lpCreateParams);
            return 0;
        }
        case WM_ERASEBKGND:
            return 1;
        case WM_PAINT: {
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hwnd, &ps);
            if (emu) blit_framebuffer(hwnd, &emu->ppu, hdc, emu->overscan_crop);
            EndPaint(hwnd, &ps);
            return 0;
        }
        case WM_KEYDOWN:
            if (emu) {
                bool first_press = (lParam & 0x40000000L) == 0;
                if (first_press) {
                    if (wParam == VK_ESCAPE) {
                        emu->running = false;
                        PostQuitMessage(0);
                    } else if (wParam == 'R') {
                        emu_reset(emu);
                    } else if (wParam == 'P') {
                        emu->paused = !emu->paused;
                    } else if (wParam == 'M') {
                        emu->muted = !emu->muted;
                    } else if (wParam == '1') {
                        resize_client_scale(hwnd, 1, emu->overscan_crop);
                    } else if (wParam == '2') {
                        resize_client_scale(hwnd, 2, emu->overscan_crop);
                    } else if (wParam == '3') {
                        resize_client_scale(hwnd, 3, emu->overscan_crop);
                    } else if (wParam == VK_F6 && emu->cart.mapper == 4u &&
                               !emu->cart.mmc3_mc_acc && !emu->cart.mmc3_is_mmc6) {
                        emu->cart.mmc3_rev_a = !emu->cart.mmc3_rev_a;
                        update_window_title(emu);
                    } else if (wParam == VK_F6 && emu->cart.mapper == 5u) {
                        emu->cart.mmc5_is_5a = !emu->cart.mmc5_is_5a;
                        emu->mmc5.is_mmc5a   = emu->cart.mmc5_is_5a;
                        if (!emu->mmc5.is_mmc5a) {
                            ricoh_2a03_set_irq(&emu->cpu, MMC5_IRQ_TIMER, false);
                        }
                        update_window_title(emu);
                    } else if (wParam == VK_F7 && emu->cart.mapper == 5u) {
                        emu->cart.mmc5_sl_mode = !emu->cart.mmc5_sl_mode;
                        emu->mmc5.sl_mode      = emu->cart.mmc5_sl_mode;
                        update_window_title(emu);
                    } else if (wParam == VK_F6 && is_mapper6(&emu->cart)) {
                        emu->cart.m6_init_mode = (uint8_t)((emu->cart.m6_init_mode + 1u) & 0x07u);
                        emu_power_on(emu);
                        update_window_title(emu);
                    } else if (wParam == VK_F8) {
                        emu->overscan_crop = !emu->overscan_crop;
                        update_window_title(emu);
                        InvalidateRect(hwnd, NULL, FALSE);
                    }
                }
                update_joypad_key(emu, wParam, true);
            }
            return 0;
        case WM_KEYUP:
            if (emu) update_joypad_key(emu, wParam, false);
            return 0;
        case WM_KILLFOCUS:
            if (emu) emu->joy1_state = 0u;
            return 0;
        case WM_DESTROY:
            if (emu) emu->running = false;
            PostQuitMessage(0);
            return 0;
    }
    return DefWindowProcA(hwnd, msg, wParam, lParam);
}

static bool open_rom_dialog(char *out_path, size_t max_len) {
    OPENFILENAMEA ofn;
    memset(&ofn, 0, sizeof(ofn));
    out_path[0] = '\0';
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "NES ROMs (*.nes)\0*.nes\0All Files (*.*)\0*.*\0";
    ofn.lpstrFile   = out_path;
    ofn.nMaxFile    = (DWORD)max_len;
    ofn.Flags       = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
    ofn.lpstrTitle  = "Chon file NES ROM (.nes)";
    return GetOpenFileNameA(&ofn) != 0;
}

static void extract_filename(const char *path, char *out, size_t out_cap) {
    const char *base = path;
    for (const char *p = path; *p != '\0'; p++) {
        if (*p == '\\' || *p == '/') base = p + 1;
    }
    snprintf(out, out_cap, "%s", base);
}

int main(int argc, char **argv) {
    bool force_mmc3a = false;
    bool force_mmc5a = false;
    bool force_mmc5_sl = false;
    bool force_overscan = false;
    int  force_m6_mode = -1;
    const char *arg_rom = NULL;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--mmc3a") == 0 || strcmp(argv[i], "-mmc3a") == 0) {
            force_mmc3a = true;
        } else if (strcmp(argv[i], "--mmc5a") == 0 || strcmp(argv[i], "-mmc5a") == 0) {
            force_mmc5a = true;
        } else if (strcmp(argv[i], "--mmc5-sl") == 0 || strcmp(argv[i], "-mmc5-sl") == 0) {
            force_mmc5_sl = true;
        } else if (strcmp(argv[i], "--overscan") == 0 || strcmp(argv[i], "-overscan") == 0) {
            force_overscan = true;
        } else if ((strcmp(argv[i], "--m6-mode") == 0 || strcmp(argv[i], "-m6-mode") == 0) &&
                   i + 1 < argc) {
            force_m6_mode = atoi(argv[++i]) & 7;
        } else if (!arg_rom) {
            arg_rom = argv[i];
        }
    }

    char rom_path[MAX_PATH];
    if (arg_rom) {
        snprintf(rom_path, sizeof(rom_path), "%s", arg_rom);
    } else if (!open_rom_dialog(rom_path, sizeof(rom_path))) {
        return 0;
    }

    NesEmulator *emu = (NesEmulator *)calloc(1u, sizeof(NesEmulator));
    if (!emu) return 1;

    if (!cart_load_ines(&emu->cart, rom_path)) {
        free(emu);
        return 1;
    }
    if (force_mmc3a && emu->cart.mapper == 4u && !emu->cart.mmc3_mc_acc) {
        emu->cart.mmc3_rev_a = true;
    }
    if (force_mmc5a && emu->cart.mapper == 5u) {
        emu->cart.mmc5_is_5a = true;
    }
    if (force_mmc5_sl && emu->cart.mapper == 5u) {
        emu->cart.mmc5_sl_mode = true;
    }
    if (force_m6_mode >= 0 && is_mapper6(&emu->cart)) {
        emu->cart.m6_init_mode = (uint8_t)(force_m6_mode & 7);
    }
    emu->overscan_crop = force_overscan;

    extract_filename(rom_path, emu->rom_title, sizeof(emu->rom_title));
    audio_init(&emu->audio);
    emu_power_on(emu);

    WNDCLASSEXA wc;
    memset(&wc, 0, sizeof(wc));
    wc.cbSize        = sizeof(wc);
    wc.style         = CS_OWNDC;
    wc.lpfnWndProc   = wnd_proc;
    wc.hInstance     = GetModuleHandleA(NULL);
    wc.hCursor       = LoadCursorA(NULL, IDC_ARROW);
    wc.lpszClassName = "HydromulatorNESWnd";
    if (!RegisterClassExA(&wc)) {
        audio_shutdown(&emu->audio);
        cart_free(&emu->cart);
        free(emu);
        return 1;
    }

    DWORD style = WS_OVERLAPPEDWINDOW;
    int init_h = emu->overscan_crop ? 224 : (int)RICOH_2C02_HEIGHT;
    RECT rc = { 0, 0, (LONG)RICOH_2C02_WIDTH, (LONG)init_h };
    AdjustWindowRectEx(&rc, style, FALSE, 0);

    emu->hwnd = CreateWindowExA(0, wc.lpszClassName, "Hydromulator [256x240]", style,
                                CW_USEDEFAULT, CW_USEDEFAULT,
                                rc.right - rc.left, rc.bottom - rc.top,
                                NULL, NULL, wc.hInstance, emu);
    if (!emu->hwnd) {
        audio_shutdown(&emu->audio);
        cart_free(&emu->cart);
        free(emu);
        return 1;
    }

    update_window_title(emu);
    ShowWindow(emu->hwnd, SW_SHOW);
    emu->running = true;

    LARGE_INTEGER freq, next_tick, now;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&next_tick);
    const double frame_ticks = (double)freq.QuadPart / 60.0988138974;
    double accum_ticks = 0.0;

    MSG msg;
    while (emu->running) {
        while (PeekMessageA(&msg, NULL, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) {
                emu->running = false;
                break;
            }
            TranslateMessage(&msg);
            DispatchMessageA(&msg);
        }
        if (!emu->running) break;

        if (!emu->paused) {
            emu_step_frame(emu);
            blit_framebuffer(emu->hwnd, &emu->ppu, NULL, emu->overscan_crop);
        }

        accum_ticks += frame_ticks;
        LONGLONG target = next_tick.QuadPart + (LONGLONG)accum_ticks;
        QueryPerformanceCounter(&now);
        if (now.QuadPart < target) {
            LONGLONG rem_ms = ((target - now.QuadPart) * 1000LL) / freq.QuadPart;
            if (rem_ms > 2) Sleep((DWORD)(rem_ms - 1));
            do {
                QueryPerformanceCounter(&now);
            } while (now.QuadPart < target);
        } else if ((now.QuadPart - target) > freq.QuadPart / 4) {
            next_tick = now;
            accum_ticks = 0.0;
        }
    }

    audio_shutdown(&emu->audio);
    cart_free(&emu->cart);
    free(emu);
    return 0;
}