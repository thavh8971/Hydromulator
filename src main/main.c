#include "2a03.h"
#include "2c02.h"
#include "apu.h"
#include "cart.h"

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

/* ==================== PPU, APU, DMA & CPU Bus ==================== */

static inline void ppu_step_and_watch(NesEmulator *emu) {
    ricoh_2c02_step(&emu->ppu);
    cart_watch_ppu_addr(&emu->cart, emu->ppu.vbus);
}

static uint8_t ppu_bus_read(void *ctx, uint16_t addr) {
    NesEmulator *emu = (NesEmulator *)ctx;
    bool is_dup_pd_rb = (emu->ppu.pd_rb && emu->ppu_last_rd_dot == emu->ppu.total_dots);
    emu->ppu_last_rd_dot = emu->ppu.total_dots;
    return cart_ppu_read(&emu->cart, addr, emu->ppu.pd_rb, is_dup_pd_rb, emu->ciram);
}

static void ppu_bus_write(void *ctx, uint16_t addr, uint8_t val) {
    NesEmulator *emu = (NesEmulator *)ctx;
    cart_ppu_write(&emu->cart, addr, val, emu->ciram);
}

static void ppu_on_nmi(void *ctx, bool asserted) {
    NesEmulator *emu = (NesEmulator *)ctx;
    ricoh_2a03_set_nmi(&emu->cpu, asserted);
}

static void irq_on_line(void *ctx, uint8_t source_mask, bool asserted) {
    NesEmulator *emu = (NesEmulator *)ctx;
    ricoh_2a03_set_irq(&emu->cpu, source_mask, asserted);
}

static uint8_t read_controller_bit(NesEmulator *emu, uint8_t port) {
    if (port == 0u) {
        emu->joy1_oe_this_cycle = true;
        return emu->joy_strobe ? (uint8_t)(emu->joy1_state & 0x01u)
                               : (uint8_t)(emu->joy1_shift & 0x01u);
    }
    emu->joy2_oe_this_cycle = true;
    return emu->joy_strobe ? (uint8_t)(emu->joy2_state & 0x01u)
                           : (uint8_t)(emu->joy2_shift & 0x01u);
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
        cart_snoop_cpu_read(&emu->cart, addr, val);
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
    if (addr >= 0x4020u) {
        return cart_cpu_read(&emu->cart, addr, emu->cpu.data_bus, NULL);
    }
    return emu->cpu.data_bus;
}

static void cpu_bus_write(void *ctx, uint16_t addr, uint8_t val) {
    NesEmulator *emu = (NesEmulator *)ctx;
    ricoh_apu_notify_cpu_write(&emu->apu);

    if (addr < 0x2000u) {
        emu->wram[addr & 0x07FFu] = val;
    } else if (addr < 0x4000u) {
        ricoh_2c02_write(&emu->ppu, addr, val);
        cart_snoop_cpu_write(&emu->cart, addr, val);
    } else if (addr == 0x4014u) {
        ricoh_apu_trigger_oam_dma(&emu->apu, val);
        cart_snoop_cpu_write(&emu->cart, addr, val);
    } else if (addr == 0x4016u) {
        emu->joy_strobe_next = (val & 0x01u) != 0u;
    } else if ((addr >= 0x4000u && addr <= 0x4013u) || addr == 0x4015u || addr == 0x4017u) {
        ricoh_apu_write(&emu->apu, addr, val);
        cart_snoop_cpu_write(&emu->cart, addr, val);
    } else if (addr >= 0x4020u) {
        cart_cpu_write(&emu->cart, addr, val, emu->cpu.cycles);
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
    cart_cpu_cycle(&emu->cart);
    ricoh_apu_set_ext_audio(&emu->apu, cart_audio_output(&emu->cart));
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
    cart_on_m2_falling(&emu->cart);
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
        cart_snoop_cpu_read(&emu->cart, dma_addr, ext);
        ext_driven = true;
    } else if (dma_addr >= 0x4020u) {
        ext = cart_cpu_read(&emu->cart, dma_addr, emu->cpu.data_bus, &ext_driven);
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
                cart_snoop_cpu_write(&emu->cart, 0x2004u, apu->oam_dma_val);
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

    CartBus cart_bus = {
        .ctx    = emu,
        .on_irq = irq_on_line,
    };
    cart_power_on(&emu->cart, &cart_bus);

    Ricoh2C02Bus pbus = {
        .ctx    = emu,
        .read   = ppu_bus_read,
        .write  = ppu_bus_write,
        .on_nmi = ppu_on_nmi,
    };
    ricoh_2c02_power_on(&emu->ppu, &pbus);

    RicohApuBus abus = {
        .ctx    = emu,
        .on_irq = irq_on_line,
    };
    ricoh_apu_power_on(&emu->apu, &abus);

    Ricoh2A03Bus cbus = {
        .ctx       = emu,
        .read      = cpu_bus_read,
        .write     = cpu_bus_write,
        .tick_pre  = cpu_bus_tick_pre,
        .tick_mid  = cpu_bus_tick_mid,
        .tick      = cpu_bus_tick,
        .dma_cycle = cpu_bus_dma_cycle,
    };
    ricoh_2a03_power_on(&emu->cpu, &cbus);
}

static void emu_reset(NesEmulator *emu) {
    emu->mid_tick_done   = false;
    emu->ppu_last_rd_dot = UINT64_MAX;
    cart_reset(&emu->cart);
    ricoh_apu_reset(&emu->apu);
    ricoh_2a03_reset(&emu->cpu);
    ricoh_2c02_reset(&emu->ppu);
    cart_sync_ppu_addr(&emu->cart, emu->ppu.vbus);
}

/* ==================== Win32 GUI (256 x 240 chuan NES) ==================== */

static void update_window_title(NesEmulator *emu) {
    if (!emu->hwnd) return;
    char tag[64];
    cart_format_tag(&emu->cart, tag, sizeof(tag));

    char title[256];
    const char *res = emu->overscan_crop ? "256x224" : "256x240";
    if (tag[0] != '\0') {
        snprintf(title, sizeof(title), "Hydromulator [%s] - %s [%s]",
                 res, emu->rom_title, tag);
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
                        emu->cart.mmc3_rev_a  = !emu->cart.mmc3_rev_a;
                        emu->cart.mmc3.rev_a  = emu->cart.mmc3_rev_a;
                        update_window_title(emu);
                    } else if (wParam == VK_F6 && emu->cart.mapper == 5u) {
                        emu->cart.mmc5_is_5a    = !emu->cart.mmc5_is_5a;
                        emu->cart.mmc5.is_mmc5a = emu->cart.mmc5_is_5a;
                        if (!emu->cart.mmc5.is_mmc5a) {
                            ricoh_2a03_set_irq(&emu->cpu, MMC5_IRQ_TIMER, false);
                        }
                        update_window_title(emu);
                    } else if (wParam == VK_F7 && emu->cart.mapper == 5u) {
                        emu->cart.mmc5_sl_mode = !emu->cart.mmc5_sl_mode;
                        emu->cart.mmc5.sl_mode = emu->cart.mmc5_sl_mode;
                        update_window_title(emu);
                    } else if (wParam == VK_F6 && (emu->cart.mapper == 6u || emu->cart.mapper == 8u)) {
                        emu->cart.m6_init_mode = (uint8_t)((emu->cart.m6_init_mode + 1u) & 0x07u);
                        emu_power_on(emu);
                        update_window_title(emu);
                    } else if (wParam == VK_F6 && emu->cart.mapper == 7u) {
                        emu->cart.ax_bus_conflicts = !emu->cart.ax_bus_conflicts;
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
    bool force_axrom_bc = false;
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
        } else if (strcmp(argv[i], "--axrom-bc") == 0 || strcmp(argv[i], "-axrom-bc") == 0) {
            force_axrom_bc = true;
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
    if (force_axrom_bc && emu->cart.mapper == 7u) {
        emu->cart.ax_bus_conflicts = true;
    }
    if (force_m6_mode >= 0 && (emu->cart.mapper == 6u || emu->cart.mapper == 8u)) {
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