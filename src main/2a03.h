#ifndef RICOH_2A03_H
#define RICOH_2A03_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Status register (P) bit flags */
#define RICOH_2A03_FLAG_C 0x01u
#define RICOH_2A03_FLAG_Z 0x02u
#define RICOH_2A03_FLAG_I 0x04u
#define RICOH_2A03_FLAG_D 0x08u
#define RICOH_2A03_FLAG_B 0x10u
#define RICOH_2A03_FLAG_U 0x20u
#define RICOH_2A03_FLAG_V 0x40u
#define RICOH_2A03_FLAG_N 0x80u

/* External 16-bit memory/IO bus interface */
typedef struct Ricoh2A03Bus {
    void *ctx;
    void (*read_start)(void *ctx, uint16_t addr);
    uint8_t (*read)(void *ctx, uint16_t addr);
    void (*write)(void *ctx, uint16_t addr, uint8_t val);
    void (*tick_pre)(void *ctx);
    void (*tick_mid)(void *ctx);
    void (*tick)(void *ctx);
    bool (*dma_cycle)(void *ctx, uint16_t halted_addr);
} Ricoh2A03Bus;

typedef struct Ricoh2A03 {
    /* Architectural registers */
    uint8_t  A;
    uint8_t  X;
    uint8_t  Y;
    uint8_t  SP;
    uint8_t  P;
    uint16_t PC;

    /* Cycle & execution state */
    uint64_t cycles;
    bool     halted;
    uint8_t  data_bus;
    uint8_t  internal_bus;
    uint32_t last_read_stalls;

    /* Interrupt pins & internal edge/level latches */
    bool     nmi_line;
    bool     prev_nmi_line;
    bool     nmi_pending;
    uint8_t  irq_line;

    bool     nmi_latched;
    bool     irq_latched;

    Ricoh2A03Bus bus;
} Ricoh2A03;

void ricoh_2a03_power_on(Ricoh2A03 *cpu, const Ricoh2A03Bus *bus);
void ricoh_2a03_reset(Ricoh2A03 *cpu);
uint32_t ricoh_2a03_step(Ricoh2A03 *cpu);
void ricoh_2a03_set_nmi(Ricoh2A03 *cpu, bool asserted);
void ricoh_2a03_set_irq(Ricoh2A03 *cpu, uint8_t source_mask, bool asserted);
void ricoh_2a03_sample_nmi(Ricoh2A03 *cpu);

#ifdef __cplusplus
}
#endif

#endif /* RICOH_2A03_H */