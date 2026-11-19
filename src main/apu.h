#ifndef RICOH_2A03_APU_H
#define RICOH_2A03_APU_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RICOH_APU_NTSC_CLOCK_HZ 1789772.727272727

typedef struct ApuLengthCounter {
    uint8_t count;
    bool    halt;
    bool    next_halt;
    bool    enabled;
} ApuLengthCounter;

typedef struct ApuEnvelope {
    bool    start;
    bool    loop;
    bool    constant_vol;
    uint8_t volume;
    uint8_t divider;
    uint8_t decay;
} ApuEnvelope;

typedef struct ApuPulse {
    uint8_t          duty_mode;
    uint8_t          duty_step;
    uint16_t         period;
    uint16_t         timer;
    ApuEnvelope      env;
    ApuLengthCounter lc;

    bool             sweep_enabled;
    uint8_t          sweep_period;
    bool             sweep_negate;
    uint8_t          sweep_shift;
    bool             sweep_reload;
    uint8_t          sweep_divider;
} ApuPulse;

typedef struct ApuTriangle {
    uint16_t         period;
    uint16_t         timer;
    uint8_t          step;
    uint8_t          linear;
    uint8_t          linear_reload_val;
    bool             linear_control;
    bool             linear_reload_flag;
    ApuLengthCounter lc;
} ApuTriangle;

typedef struct ApuNoise {
    bool             mode;
    uint8_t          period_idx;
    uint16_t         timer;
    uint16_t         lfsr;
    ApuEnvelope      env;
    ApuLengthCounter lc;
} ApuNoise;

typedef struct ApuDmc {
    bool     irq_enable;
    bool     loop;
    uint8_t  rate_idx;
    uint16_t period;
    uint16_t timer;
    uint8_t  output_level;

    uint16_t sample_addr;
    uint16_t sample_length;
    uint16_t current_addr;
    uint16_t bytes_remaining;

    uint8_t  sample_buffer;
    bool     buffer_empty;
    uint8_t  shift_reg;
    uint8_t  bits_remaining;
    bool     silence;
    bool     enabled;
} ApuDmc;

typedef enum ApuDmcDmaPhase {
    APU_DMC_DMA_IDLE  = 0,
    APU_DMC_DMA_HALT  = 1,
    APU_DMC_DMA_DUMMY = 2,
    APU_DMC_DMA_GET   = 3
} ApuDmcDmaPhase;

typedef struct RicohApuBus {
    void *ctx;
    void (*on_irq)(void *ctx, uint8_t source_mask, bool asserted);
} RicohApuBus;

typedef struct RicohApu {
    ApuPulse    pulse1;
    ApuPulse    pulse2;
    ApuTriangle tri;
    ApuNoise    noise;
    ApuDmc      dmc;

    /* Frame Counter & GET/PUT phase */
    bool     put;
    bool     fc_five_step;
    bool     fc_next_five_step;
    bool     fc_irq_inhibit;
    uint8_t  fc_last_write;
    uint8_t  fc_reset_delay;
    uint16_t fc_counter;
    bool     frame_irq;
    bool     frame_irq_clear_pending;
    bool     dmc_irq;
    bool     irq_asserted;

    /* OAM DMA ($4014) */
    bool     oam_dma_active;
    bool     oam_dma_halt;
    bool     oam_dma_put_phase;
    uint8_t  oam_dma_page;
    uint8_t  oam_dma_index;
    uint8_t  oam_dma_val;

    /* DMC DMA */
    uint8_t        dmc_load_delay;
    uint8_t        dmc_abort_delay;
    uint8_t        dmc_implicit_window;
    bool           dmc_just_reloaded;
    bool           dmc_dma_pending;
    bool           dmc_dma_abort_one;
    ApuDmcDmaPhase dmc_dma_phase;

    /* Analog mixer filters & cartridge expansion audio input */
    float    ext_input;
    float    hp_prev_in;
    float    hp_out;
    float    lp_out;

    RicohApuBus bus;
} RicohApu;

void ricoh_apu_power_on(RicohApu *apu, const RicohApuBus *bus);
void ricoh_apu_reset(RicohApu *apu);
void ricoh_apu_step(RicohApu *apu);

uint8_t ricoh_apu_read_status(RicohApu *apu, uint8_t data_bus);
void ricoh_apu_write(RicohApu *apu, uint16_t addr, uint8_t val);

void ricoh_apu_trigger_oam_dma(RicohApu *apu, uint8_t page);
void ricoh_apu_notify_cpu_write(RicohApu *apu);
bool ricoh_apu_dma_wants_halt(const RicohApu *apu);
void ricoh_apu_dmc_finish_dma(RicohApu *apu, uint8_t val);

void  ricoh_apu_set_ext_audio(RicohApu *apu, float ext_sample);
float ricoh_apu_output(const RicohApu *apu);

#ifdef __cplusplus
}
#endif

#endif /* RICOH_2A03_APU_H */