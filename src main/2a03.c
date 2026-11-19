#include "2a03.h"
#include <stddef.h>
#include <string.h>

#ifndef RICOH_2A03_UNSTABLE_CONST
#define RICOH_2A03_UNSTABLE_CONST 0xEEu
#endif

#ifndef RICOH_2A03_LXA_CONST
#define RICOH_2A03_LXA_CONST 0xFFu
#endif

static inline void sample_interrupt_lines(Ricoh2A03 *cpu) {
    if (!cpu->prev_nmi_line && cpu->nmi_line) {
        cpu->nmi_pending = true;
    }
    cpu->prev_nmi_line = cpu->nmi_line;
}

void ricoh_2a03_sample_nmi(Ricoh2A03 *cpu) {
    sample_interrupt_lines(cpu);
}

/* Chot NMI/IRQ theo kieu OR trong suot cung 1 lenh (giu poll 1 cua branch 4 chu ky) */
static inline void poll_interrupts(Ricoh2A03 *cpu) {
    if (cpu->nmi_pending) {
        cpu->nmi_latched = true;
    }
    if ((cpu->irq_line != 0u) && ((cpu->P & RICOH_2A03_FLAG_I) == 0u)) {
        cpu->irq_latched = true;
    }
}

static uint8_t cpu_read(Ricoh2A03 *cpu, uint16_t addr) {
    uint32_t stalls = 0u;
    while (cpu->bus.dma_cycle != NULL && cpu->bus.dma_cycle(cpu->bus.ctx, addr)) {
        stalls++;
        cpu->cycles++;
        sample_interrupt_lines(cpu);
    }
    cpu->last_read_stalls = stalls;

    if (cpu->bus.tick_pre != NULL) {
        cpu->bus.tick_pre(cpu->bus.ctx);
    }
    uint8_t val = (cpu->bus.read != NULL) ? cpu->bus.read(cpu->bus.ctx, addr) : cpu->data_bus;
    if (addr != 0x4015u) {
        cpu->data_bus = val;
    }
    cpu->internal_bus = val;
    cpu->cycles++;
    if (cpu->bus.tick_mid != NULL) {
        cpu->bus.tick_mid(cpu->bus.ctx);
        sample_interrupt_lines(cpu);
    }
    if (cpu->bus.tick != NULL) {
        cpu->bus.tick(cpu->bus.ctx);
    }
    sample_interrupt_lines(cpu);
    return val;
}

static void cpu_write(Ricoh2A03 *cpu, uint16_t addr, uint8_t val) {
    if (cpu->bus.tick_pre != NULL) {
        cpu->bus.tick_pre(cpu->bus.ctx);
    }
    cpu->data_bus = val;
    cpu->internal_bus = val;
    if (cpu->bus.write != NULL) {
        cpu->bus.write(cpu->bus.ctx, addr, val);
    }
    cpu->cycles++;
    if (cpu->bus.tick_mid != NULL) {
        cpu->bus.tick_mid(cpu->bus.ctx);
        sample_interrupt_lines(cpu);
    }
    if (cpu->bus.tick != NULL) {
        cpu->bus.tick(cpu->bus.ctx);
    }
    sample_interrupt_lines(cpu);
}

static inline void push(Ricoh2A03 *cpu, uint8_t val) {
    cpu_write(cpu, (uint16_t)(0x0100u | cpu->SP), val);
    cpu->SP--;
}

static inline uint8_t pull(Ricoh2A03 *cpu) {
    cpu->SP++;
    return cpu_read(cpu, (uint16_t)(0x0100u | cpu->SP));
}

static inline void set_flag(Ricoh2A03 *cpu, uint8_t mask, bool cond) {
    if (cond) {
        cpu->P |= mask;
    } else {
        cpu->P &= (uint8_t)(~mask);
    }
}

static inline void set_nz(Ricoh2A03 *cpu, uint8_t val) {
    set_flag(cpu, RICOH_2A03_FLAG_Z, val == 0u);
    set_flag(cpu, RICOH_2A03_FLAG_N, (val & 0x80u) != 0u);
}

/* ==================== Read Addressing Modes ==================== */

static uint8_t read_imm(Ricoh2A03 *cpu) {
    poll_interrupts(cpu);
    return cpu_read(cpu, cpu->PC++);
}

static uint8_t read_zp(Ricoh2A03 *cpu) {
    uint16_t addr = cpu_read(cpu, cpu->PC++);
    poll_interrupts(cpu);
    return cpu_read(cpu, addr);
}

static uint8_t read_zp_idx(Ricoh2A03 *cpu, uint8_t idx) {
    uint8_t base = cpu_read(cpu, cpu->PC++);
    (void)cpu_read(cpu, base);
    poll_interrupts(cpu);
    return cpu_read(cpu, (uint8_t)(base + idx));
}

static uint8_t read_abs(Ricoh2A03 *cpu) {
    uint8_t lo = cpu_read(cpu, cpu->PC++);
    uint8_t hi = cpu_read(cpu, cpu->PC++);
    poll_interrupts(cpu);
    return cpu_read(cpu, (uint16_t)lo | ((uint16_t)hi << 8));
}

static uint8_t read_abs_idx(Ricoh2A03 *cpu, uint8_t idx) {
    uint8_t lo = cpu_read(cpu, cpu->PC++);
    uint8_t hi = cpu_read(cpu, cpu->PC++);
    uint16_t base = (uint16_t)lo | ((uint16_t)hi << 8);
    uint16_t addr = (uint16_t)(base + idx);
    if ((base & 0xFF00u) != (addr & 0xFF00u)) {
        (void)cpu_read(cpu, (uint16_t)((base & 0xFF00u) | (addr & 0x00FFu)));
    }
    poll_interrupts(cpu);
    return cpu_read(cpu, addr);
}

static uint8_t read_ind_x(Ricoh2A03 *cpu) {
    uint8_t zp = cpu_read(cpu, cpu->PC++);
    (void)cpu_read(cpu, zp);
    uint8_t ptr = (uint8_t)(zp + cpu->X);
    uint8_t lo = cpu_read(cpu, ptr);
    uint8_t hi = cpu_read(cpu, (uint8_t)(ptr + 1u));
    poll_interrupts(cpu);
    return cpu_read(cpu, (uint16_t)lo | ((uint16_t)hi << 8));
}

static uint8_t read_ind_y(Ricoh2A03 *cpu) {
    uint8_t zp = cpu_read(cpu, cpu->PC++);
    uint8_t lo = cpu_read(cpu, zp);
    uint8_t hi = cpu_read(cpu, (uint8_t)(zp + 1u));
    uint16_t base = (uint16_t)lo | ((uint16_t)hi << 8);
    uint16_t addr = (uint16_t)(base + cpu->Y);
    if ((base & 0xFF00u) != (addr & 0xFF00u)) {
        (void)cpu_read(cpu, (uint16_t)((base & 0xFF00u) | (addr & 0x00FFu)));
    }
    poll_interrupts(cpu);
    return cpu_read(cpu, addr);
}

/* ==================== Write & RMW Address Calculation ==================== */

static uint16_t addr_zp(Ricoh2A03 *cpu) {
    return cpu_read(cpu, cpu->PC++);
}

static uint16_t addr_zp_idx(Ricoh2A03 *cpu, uint8_t idx) {
    uint8_t base = cpu_read(cpu, cpu->PC++);
    (void)cpu_read(cpu, base);
    return (uint8_t)(base + idx);
}

static uint16_t addr_abs(Ricoh2A03 *cpu) {
    uint8_t lo = cpu_read(cpu, cpu->PC++);
    uint8_t hi = cpu_read(cpu, cpu->PC++);
    return (uint16_t)lo | ((uint16_t)hi << 8);
}

static uint16_t addr_abs_idx(Ricoh2A03 *cpu, uint8_t idx) {
    uint8_t lo = cpu_read(cpu, cpu->PC++);
    uint8_t hi = cpu_read(cpu, cpu->PC++);
    uint16_t base = (uint16_t)lo | ((uint16_t)hi << 8);
    uint16_t addr = (uint16_t)(base + idx);
    (void)cpu_read(cpu, (uint16_t)((base & 0xFF00u) | (addr & 0x00FFu)));
    return addr;
}

static uint16_t addr_ind_x(Ricoh2A03 *cpu) {
    uint8_t zp = cpu_read(cpu, cpu->PC++);
    (void)cpu_read(cpu, zp);
    uint8_t ptr = (uint8_t)(zp + cpu->X);
    uint8_t lo = cpu_read(cpu, ptr);
    uint8_t hi = cpu_read(cpu, (uint8_t)(ptr + 1u));
    return (uint16_t)lo | ((uint16_t)hi << 8);
}

static uint16_t addr_ind_y(Ricoh2A03 *cpu) {
    uint8_t zp = cpu_read(cpu, cpu->PC++);
    uint8_t lo = cpu_read(cpu, zp);
    uint8_t hi = cpu_read(cpu, (uint8_t)(zp + 1u));
    uint16_t base = (uint16_t)lo | ((uint16_t)hi << 8);
    uint16_t addr = (uint16_t)(base + cpu->Y);
    (void)cpu_read(cpu, (uint16_t)((base & 0xFF00u) | (addr & 0x00FFu)));
    return addr;
}

/* ==================== ALU & RMW Operations ==================== */

static inline void op_ora(Ricoh2A03 *cpu, uint8_t val) {
    cpu->A |= val;
    set_nz(cpu, cpu->A);
}

static inline void op_and(Ricoh2A03 *cpu, uint8_t val) {
    cpu->A &= val;
    set_nz(cpu, cpu->A);
}

static inline void op_eor(Ricoh2A03 *cpu, uint8_t val) {
    cpu->A ^= val;
    set_nz(cpu, cpu->A);
}

static inline void op_adc(Ricoh2A03 *cpu, uint8_t val) {
    uint16_t carry = (cpu->P & RICOH_2A03_FLAG_C) ? 1u : 0u;
    uint16_t sum = (uint16_t)cpu->A + (uint16_t)val + carry;
    uint8_t res = (uint8_t)sum;
    set_flag(cpu, RICOH_2A03_FLAG_C, sum > 0xFFu);
    set_flag(cpu, RICOH_2A03_FLAG_V, (~(cpu->A ^ val) & (cpu->A ^ res) & 0x80u) != 0u);
    cpu->A = res;
    set_nz(cpu, cpu->A);
}

static inline void op_sbc(Ricoh2A03 *cpu, uint8_t val) {
    op_adc(cpu, (uint8_t)(~val));
}

static inline void op_cmp(Ricoh2A03 *cpu, uint8_t reg, uint8_t val) {
    uint8_t diff = (uint8_t)(reg - val);
    set_flag(cpu, RICOH_2A03_FLAG_C, reg >= val);
    set_nz(cpu, diff);
}

static inline void op_bit(Ricoh2A03 *cpu, uint8_t val) {
    set_flag(cpu, RICOH_2A03_FLAG_Z, (cpu->A & val) == 0u);
    set_flag(cpu, RICOH_2A03_FLAG_V, (val & 0x40u) != 0u);
    set_flag(cpu, RICOH_2A03_FLAG_N, (val & 0x80u) != 0u);
}

typedef uint8_t (*RmwOp)(Ricoh2A03 *cpu, uint8_t val);

static uint8_t rmw_asl(Ricoh2A03 *cpu, uint8_t val) {
    set_flag(cpu, RICOH_2A03_FLAG_C, (val & 0x80u) != 0u);
    uint8_t res = (uint8_t)(val << 1);
    set_nz(cpu, res);
    return res;
}

static uint8_t rmw_lsr(Ricoh2A03 *cpu, uint8_t val) {
    set_flag(cpu, RICOH_2A03_FLAG_C, (val & 0x01u) != 0u);
    uint8_t res = (uint8_t)(val >> 1);
    set_nz(cpu, res);
    return res;
}

static uint8_t rmw_rol(Ricoh2A03 *cpu, uint8_t val) {
    uint8_t c_in = (cpu->P & RICOH_2A03_FLAG_C) ? 1u : 0u;
    set_flag(cpu, RICOH_2A03_FLAG_C, (val & 0x80u) != 0u);
    uint8_t res = (uint8_t)((val << 1) | c_in);
    set_nz(cpu, res);
    return res;
}

static uint8_t rmw_ror(Ricoh2A03 *cpu, uint8_t val) {
    uint8_t c_in = (cpu->P & RICOH_2A03_FLAG_C) ? 0x80u : 0x00u;
    set_flag(cpu, RICOH_2A03_FLAG_C, (val & 0x01u) != 0u);
    uint8_t res = (uint8_t)((val >> 1) | c_in);
    set_nz(cpu, res);
    return res;
}

static uint8_t rmw_dec(Ricoh2A03 *cpu, uint8_t val) {
    uint8_t res = (uint8_t)(val - 1u);
    set_nz(cpu, res);
    return res;
}

static uint8_t rmw_inc(Ricoh2A03 *cpu, uint8_t val) {
    uint8_t res = (uint8_t)(val + 1u);
    set_nz(cpu, res);
    return res;
}

static uint8_t rmw_slo(Ricoh2A03 *cpu, uint8_t val) {
    uint8_t res = rmw_asl(cpu, val);
    op_ora(cpu, res);
    return res;
}

static uint8_t rmw_rla(Ricoh2A03 *cpu, uint8_t val) {
    uint8_t res = rmw_rol(cpu, val);
    op_and(cpu, res);
    return res;
}

static uint8_t rmw_sre(Ricoh2A03 *cpu, uint8_t val) {
    uint8_t res = rmw_lsr(cpu, val);
    op_eor(cpu, res);
    return res;
}

static uint8_t rmw_rra(Ricoh2A03 *cpu, uint8_t val) {
    uint8_t res = rmw_ror(cpu, val);
    op_adc(cpu, res);
    return res;
}

static uint8_t rmw_dcp(Ricoh2A03 *cpu, uint8_t val) {
    uint8_t res = (uint8_t)(val - 1u);
    op_cmp(cpu, cpu->A, res);
    return res;
}

static uint8_t rmw_isc(Ricoh2A03 *cpu, uint8_t val) {
    uint8_t res = (uint8_t)(val + 1u);
    op_sbc(cpu, res);
    return res;
}

static void exec_rmw(Ricoh2A03 *cpu, uint16_t addr, RmwOp op) {
    uint8_t old_val = cpu_read(cpu, addr);
    cpu_write(cpu, addr, old_val);
    uint8_t new_val = op(cpu, old_val);
    poll_interrupts(cpu);
    cpu_write(cpu, addr, new_val);
}

static inline void exec_write(Ricoh2A03 *cpu, uint16_t addr, uint8_t val) {
    poll_interrupts(cpu);
    cpu_write(cpu, addr, val);
}

static void exec_branch(Ricoh2A03 *cpu, bool take) {
    poll_interrupts(cpu);
    uint8_t raw = cpu_read(cpu, cpu->PC++);
    if (!take) {
        return;
    }
    uint16_t offset16 = (raw & 0x80u) ? (uint16_t)(0xFF00u | raw) : (uint16_t)raw;
    uint16_t old_pc = cpu->PC;
    uint16_t new_pc = (uint16_t)(old_pc + offset16);
    (void)cpu_read(cpu, old_pc);
    if ((old_pc & 0xFF00u) != (new_pc & 0xFF00u)) {
        poll_interrupts(cpu);
        (void)cpu_read(cpu, (uint16_t)((old_pc & 0xFF00u) | (new_pc & 0x00FFu)));
    }
    cpu->PC = new_pc;
}

static void exec_interrupt(Ricoh2A03 *cpu, bool is_brk, bool is_nmi) {
    if (is_brk) {
        (void)cpu_read(cpu, cpu->PC++);
    } else {
        (void)cpu_read(cpu, cpu->PC);
        (void)cpu_read(cpu, cpu->PC);
    }

    push(cpu, (uint8_t)(cpu->PC >> 8));
    push(cpu, (uint8_t)(cpu->PC & 0xFFu));

    uint16_t vec = 0xFFFEu;
    if (is_nmi || cpu->nmi_pending) {
        vec = 0xFFFAu;
        cpu->nmi_pending = false;
    }

    uint8_t p_to_push = (uint8_t)(cpu->P | RICOH_2A03_FLAG_U);
    if (is_brk) {
        p_to_push |= RICOH_2A03_FLAG_B;
    } else {
        p_to_push &= (uint8_t)(~RICOH_2A03_FLAG_B);
    }
    push(cpu, p_to_push);

    uint8_t lo = cpu_read(cpu, vec);
    cpu->P |= RICOH_2A03_FLAG_I;
    uint8_t hi = cpu_read(cpu, (uint16_t)(vec + 1u));
    cpu->PC = (uint16_t)lo | ((uint16_t)hi << 8);
}

/* ==================== Public API ==================== */

void ricoh_2a03_power_on(Ricoh2A03 *cpu, const Ricoh2A03Bus *bus) {
    memset(cpu, 0, sizeof(*cpu));
    if (bus != NULL) {
        cpu->bus = *bus;
    }
    cpu->P = RICOH_2A03_FLAG_U | RICOH_2A03_FLAG_I;
    cpu->SP = 0x00u;
    ricoh_2a03_reset(cpu);
}

void ricoh_2a03_reset(Ricoh2A03 *cpu) {
    cpu->halted = false;
    cpu->nmi_pending = false;
    cpu->nmi_latched = false;
    cpu->irq_latched = false;
    cpu->prev_nmi_line = cpu->nmi_line;

    (void)cpu_read(cpu, cpu->PC);
    (void)cpu_read(cpu, cpu->PC);
    (void)cpu_read(cpu, (uint16_t)(0x0100u | cpu->SP--));
    (void)cpu_read(cpu, (uint16_t)(0x0100u | cpu->SP--));
    (void)cpu_read(cpu, (uint16_t)(0x0100u | cpu->SP--));

    cpu->P = (uint8_t)((cpu->P | RICOH_2A03_FLAG_I | RICOH_2A03_FLAG_U) & ~RICOH_2A03_FLAG_B);
    uint8_t lo = cpu_read(cpu, 0xFFFCu);
    uint8_t hi = cpu_read(cpu, 0xFFFDu);
    cpu->PC = (uint16_t)lo | ((uint16_t)hi << 8);
}

void ricoh_2a03_set_nmi(Ricoh2A03 *cpu, bool asserted) {
    cpu->nmi_line = asserted;
}

void ricoh_2a03_set_irq(Ricoh2A03 *cpu, uint8_t source_mask, bool asserted) {
    if (asserted) {
        cpu->irq_line |= source_mask;
    } else {
        cpu->irq_line &= (uint8_t)(~source_mask);
    }
}

uint32_t ricoh_2a03_step(Ricoh2A03 *cpu) {
    uint64_t start_cycles = cpu->cycles;

    if (cpu->halted) {
        (void)cpu_read(cpu, cpu->PC);
        return (uint32_t)(cpu->cycles - start_cycles);
    }

    if (cpu->nmi_latched) {
        cpu->nmi_latched = false;
        cpu->irq_latched = false;
        exec_interrupt(cpu, false, true);
        return (uint32_t)(cpu->cycles - start_cycles);
    }

    if (cpu->irq_latched) {
        cpu->irq_latched = false;
        exec_interrupt(cpu, false, false);
        return (uint32_t)(cpu->cycles - start_cycles);
    }

    cpu->nmi_latched = false;
    cpu->irq_latched = false;

    uint8_t opcode = cpu_read(cpu, cpu->PC++);

    switch (opcode) {
        case 0x00:
            exec_interrupt(cpu, true, false);
            break;

        case 0x20: {
            uint8_t lo = cpu_read(cpu, cpu->PC++);
            (void)cpu_read(cpu, (uint16_t)(0x0100u | cpu->SP));
            push(cpu, (uint8_t)(cpu->PC >> 8));
            push(cpu, (uint8_t)(cpu->PC & 0xFFu));
            poll_interrupts(cpu);
            uint8_t hi = cpu_read(cpu, cpu->PC);
            cpu->PC = (uint16_t)lo | ((uint16_t)hi << 8);
            break;
        }

        case 0x40: {
            (void)cpu_read(cpu, cpu->PC);
            (void)cpu_read(cpu, (uint16_t)(0x0100u | cpu->SP));
            cpu->P = (uint8_t)((pull(cpu) & ~RICOH_2A03_FLAG_B) | RICOH_2A03_FLAG_U);
            uint8_t lo = pull(cpu);
            poll_interrupts(cpu);
            uint8_t hi = pull(cpu);
            cpu->PC = (uint16_t)lo | ((uint16_t)hi << 8);
            break;
        }

        case 0x60: {
            (void)cpu_read(cpu, cpu->PC);
            (void)cpu_read(cpu, (uint16_t)(0x0100u | cpu->SP));
            uint8_t lo = pull(cpu);
            uint8_t hi = pull(cpu);
            cpu->PC = (uint16_t)lo | ((uint16_t)hi << 8);
            poll_interrupts(cpu);
            (void)cpu_read(cpu, cpu->PC++);
            break;
        }

        case 0x4C: {
            uint8_t lo = cpu_read(cpu, cpu->PC++);
            poll_interrupts(cpu);
            uint8_t hi = cpu_read(cpu, cpu->PC++);
            cpu->PC = (uint16_t)lo | ((uint16_t)hi << 8);
            break;
        }

        case 0x6C: {
            uint8_t ptr_lo = cpu_read(cpu, cpu->PC++);
            uint8_t ptr_hi = cpu_read(cpu, cpu->PC++);
            uint16_t ptr = (uint16_t)ptr_lo | ((uint16_t)ptr_hi << 8);
            uint8_t lo = cpu_read(cpu, ptr);
            poll_interrupts(cpu);
            uint16_t hi_addr = (uint16_t)((ptr & 0xFF00u) | (uint8_t)(ptr_lo + 1u));
            uint8_t hi = cpu_read(cpu, hi_addr);
            cpu->PC = (uint16_t)lo | ((uint16_t)hi << 8);
            break;
        }

        case 0x08:
            (void)cpu_read(cpu, cpu->PC);
            poll_interrupts(cpu);
            push(cpu, (uint8_t)(cpu->P | RICOH_2A03_FLAG_B | RICOH_2A03_FLAG_U));
            break;

        case 0x48:
            (void)cpu_read(cpu, cpu->PC);
            poll_interrupts(cpu);
            push(cpu, cpu->A);
            break;

        case 0x28: {
            (void)cpu_read(cpu, cpu->PC);
            (void)cpu_read(cpu, (uint16_t)(0x0100u | cpu->SP));
            poll_interrupts(cpu);
            uint8_t val = pull(cpu);
            cpu->P = (uint8_t)((val & ~RICOH_2A03_FLAG_B) | RICOH_2A03_FLAG_U);
            break;
        }

        case 0x68:
            (void)cpu_read(cpu, cpu->PC);
            (void)cpu_read(cpu, (uint16_t)(0x0100u | cpu->SP));
            poll_interrupts(cpu);
            cpu->A = pull(cpu);
            set_nz(cpu, cpu->A);
            break;

        case 0x10: exec_branch(cpu, (cpu->P & RICOH_2A03_FLAG_N) == 0u); break;
        case 0x30: exec_branch(cpu, (cpu->P & RICOH_2A03_FLAG_N) != 0u); break;
        case 0x50: exec_branch(cpu, (cpu->P & RICOH_2A03_FLAG_V) == 0u); break;
        case 0x70: exec_branch(cpu, (cpu->P & RICOH_2A03_FLAG_V) != 0u); break;
        case 0x90: exec_branch(cpu, (cpu->P & RICOH_2A03_FLAG_C) == 0u); break;
        case 0xB0: exec_branch(cpu, (cpu->P & RICOH_2A03_FLAG_C) != 0u); break;
        case 0xD0: exec_branch(cpu, (cpu->P & RICOH_2A03_FLAG_Z) == 0u); break;
        case 0xF0: exec_branch(cpu, (cpu->P & RICOH_2A03_FLAG_Z) != 0u); break;

        case 0x18: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); set_flag(cpu, RICOH_2A03_FLAG_C, false); break;
        case 0x38: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); set_flag(cpu, RICOH_2A03_FLAG_C, true);  break;
        case 0x58: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); set_flag(cpu, RICOH_2A03_FLAG_I, false); break;
        case 0x78: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); set_flag(cpu, RICOH_2A03_FLAG_I, true);  break;
        case 0xB8: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); set_flag(cpu, RICOH_2A03_FLAG_V, false); break;
        case 0xD8: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); set_flag(cpu, RICOH_2A03_FLAG_D, false); break;
        case 0xF8: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); set_flag(cpu, RICOH_2A03_FLAG_D, true);  break;

        case 0x88: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); cpu->Y--; set_nz(cpu, cpu->Y); break;
        case 0xC8: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); cpu->Y++; set_nz(cpu, cpu->Y); break;
        case 0xCA: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); cpu->X--; set_nz(cpu, cpu->X); break;
        case 0xE8: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); cpu->X++; set_nz(cpu, cpu->X); break;

        case 0x8A: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); cpu->A = cpu->X; set_nz(cpu, cpu->A); break;
        case 0x98: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); cpu->A = cpu->Y; set_nz(cpu, cpu->A); break;
        case 0xA8: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); cpu->Y = cpu->A; set_nz(cpu, cpu->Y); break;
        case 0xAA: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); cpu->X = cpu->A; set_nz(cpu, cpu->X); break;
        case 0xBA: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); cpu->X = cpu->SP; set_nz(cpu, cpu->X); break;
        case 0x9A: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); cpu->SP = cpu->X; break;

        case 0x01: op_ora(cpu, read_ind_x(cpu)); break;
        case 0x05: op_ora(cpu, read_zp(cpu));    break;
        case 0x09: op_ora(cpu, read_imm(cpu));   break;
        case 0x0D: op_ora(cpu, read_abs(cpu));   break;
        case 0x11: op_ora(cpu, read_ind_y(cpu)); break;
        case 0x15: op_ora(cpu, read_zp_idx(cpu, cpu->X)); break;
        case 0x19: op_ora(cpu, read_abs_idx(cpu, cpu->Y)); break;
        case 0x1D: op_ora(cpu, read_abs_idx(cpu, cpu->X)); break;

        case 0x21: op_and(cpu, read_ind_x(cpu)); break;
        case 0x25: op_and(cpu, read_zp(cpu));    break;
        case 0x29: op_and(cpu, read_imm(cpu));   break;
        case 0x2D: op_and(cpu, read_abs(cpu));   break;
        case 0x31: op_and(cpu, read_ind_y(cpu)); break;
        case 0x35: op_and(cpu, read_zp_idx(cpu, cpu->X)); break;
        case 0x39: op_and(cpu, read_abs_idx(cpu, cpu->Y)); break;
        case 0x3D: op_and(cpu, read_abs_idx(cpu, cpu->X)); break;

        case 0x41: op_eor(cpu, read_ind_x(cpu)); break;
        case 0x45: op_eor(cpu, read_zp(cpu));    break;
        case 0x49: op_eor(cpu, read_imm(cpu));   break;
        case 0x4D: op_eor(cpu, read_abs(cpu));   break;
        case 0x51: op_eor(cpu, read_ind_y(cpu)); break;
        case 0x55: op_eor(cpu, read_zp_idx(cpu, cpu->X)); break;
        case 0x59: op_eor(cpu, read_abs_idx(cpu, cpu->Y)); break;
        case 0x5D: op_eor(cpu, read_abs_idx(cpu, cpu->X)); break;

        case 0x61: op_adc(cpu, read_ind_x(cpu)); break;
        case 0x65: op_adc(cpu, read_zp(cpu));    break;
        case 0x69: op_adc(cpu, read_imm(cpu));   break;
        case 0x6D: op_adc(cpu, read_abs(cpu));   break;
        case 0x71: op_adc(cpu, read_ind_y(cpu)); break;
        case 0x75: op_adc(cpu, read_zp_idx(cpu, cpu->X)); break;
        case 0x79: op_adc(cpu, read_abs_idx(cpu, cpu->Y)); break;
        case 0x7D: op_adc(cpu, read_abs_idx(cpu, cpu->X)); break;

        case 0xC1: op_cmp(cpu, cpu->A, read_ind_x(cpu)); break;
        case 0xC5: op_cmp(cpu, cpu->A, read_zp(cpu));    break;
        case 0xC9: op_cmp(cpu, cpu->A, read_imm(cpu));   break;
        case 0xCD: op_cmp(cpu, cpu->A, read_abs(cpu));   break;
        case 0xD1: op_cmp(cpu, cpu->A, read_ind_y(cpu)); break;
        case 0xD5: op_cmp(cpu, cpu->A, read_zp_idx(cpu, cpu->X)); break;
        case 0xD9: op_cmp(cpu, cpu->A, read_abs_idx(cpu, cpu->Y)); break;
        case 0xDD: op_cmp(cpu, cpu->A, read_abs_idx(cpu, cpu->X)); break;

        case 0xE1: op_sbc(cpu, read_ind_x(cpu)); break;
        case 0xE5: op_sbc(cpu, read_zp(cpu));    break;
        case 0xE9:
        case 0xEB: op_sbc(cpu, read_imm(cpu));   break;
        case 0xED: op_sbc(cpu, read_abs(cpu));   break;
        case 0xF1: op_sbc(cpu, read_ind_y(cpu)); break;
        case 0xF5: op_sbc(cpu, read_zp_idx(cpu, cpu->X)); break;
        case 0xF9: op_sbc(cpu, read_abs_idx(cpu, cpu->Y)); break;
        case 0xFD: op_sbc(cpu, read_abs_idx(cpu, cpu->X)); break;

        case 0x24: op_bit(cpu, read_zp(cpu));  break;
        case 0x2C: op_bit(cpu, read_abs(cpu)); break;

        case 0xE0: op_cmp(cpu, cpu->X, read_imm(cpu)); break;
        case 0xE4: op_cmp(cpu, cpu->X, read_zp(cpu));  break;
        case 0xEC: op_cmp(cpu, cpu->X, read_abs(cpu)); break;
        case 0xC0: op_cmp(cpu, cpu->Y, read_imm(cpu)); break;
        case 0xC4: op_cmp(cpu, cpu->Y, read_zp(cpu));  break;
        case 0xCC: op_cmp(cpu, cpu->Y, read_abs(cpu)); break;

        case 0xA1: cpu->A = read_ind_x(cpu); set_nz(cpu, cpu->A); break;
        case 0xA5: cpu->A = read_zp(cpu);    set_nz(cpu, cpu->A); break;
        case 0xA9: cpu->A = read_imm(cpu);   set_nz(cpu, cpu->A); break;
        case 0xAD: cpu->A = read_abs(cpu);   set_nz(cpu, cpu->A); break;
        case 0xB1: cpu->A = read_ind_y(cpu); set_nz(cpu, cpu->A); break;
        case 0xB5: cpu->A = read_zp_idx(cpu, cpu->X); set_nz(cpu, cpu->A); break;
        case 0xB9: cpu->A = read_abs_idx(cpu, cpu->Y); set_nz(cpu, cpu->A); break;
        case 0xBD: cpu->A = read_abs_idx(cpu, cpu->X); set_nz(cpu, cpu->A); break;

        case 0xA2: cpu->X = read_imm(cpu); set_nz(cpu, cpu->X); break;
        case 0xA6: cpu->X = read_zp(cpu);  set_nz(cpu, cpu->X); break;
        case 0xAE: cpu->X = read_abs(cpu); set_nz(cpu, cpu->X); break;
        case 0xB6: cpu->X = read_zp_idx(cpu, cpu->Y); set_nz(cpu, cpu->X); break;
        case 0xBE: cpu->X = read_abs_idx(cpu, cpu->Y); set_nz(cpu, cpu->X); break;

        case 0xA0: cpu->Y = read_imm(cpu); set_nz(cpu, cpu->Y); break;
        case 0xA4: cpu->Y = read_zp(cpu);  set_nz(cpu, cpu->Y); break;
        case 0xAC: cpu->Y = read_abs(cpu); set_nz(cpu, cpu->Y); break;
        case 0xB4: cpu->Y = read_zp_idx(cpu, cpu->X); set_nz(cpu, cpu->Y); break;
        case 0xBC: cpu->Y = read_abs_idx(cpu, cpu->X); set_nz(cpu, cpu->Y); break;

        case 0x81: exec_write(cpu, addr_ind_x(cpu), cpu->A); break;
        case 0x85: exec_write(cpu, addr_zp(cpu),    cpu->A); break;
        case 0x8D: exec_write(cpu, addr_abs(cpu),   cpu->A); break;
        case 0x91: exec_write(cpu, addr_ind_y(cpu), cpu->A); break;
        case 0x95: exec_write(cpu, addr_zp_idx(cpu, cpu->X), cpu->A); break;
        case 0x99: exec_write(cpu, addr_abs_idx(cpu, cpu->Y), cpu->A); break;
        case 0x9D: exec_write(cpu, addr_abs_idx(cpu, cpu->X), cpu->A); break;

        case 0x86: exec_write(cpu, addr_zp(cpu),  cpu->X); break;
        case 0x8E: exec_write(cpu, addr_abs(cpu), cpu->X); break;
        case 0x96: exec_write(cpu, addr_zp_idx(cpu, cpu->Y), cpu->X); break;

        case 0x84: exec_write(cpu, addr_zp(cpu),  cpu->Y); break;
        case 0x8C: exec_write(cpu, addr_abs(cpu), cpu->Y); break;
        case 0x94: exec_write(cpu, addr_zp_idx(cpu, cpu->X), cpu->Y); break;

        case 0x0A: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); cpu->A = rmw_asl(cpu, cpu->A); break;
        case 0x06: exec_rmw(cpu, addr_zp(cpu), rmw_asl); break;
        case 0x16: exec_rmw(cpu, addr_zp_idx(cpu, cpu->X), rmw_asl); break;
        case 0x0E: exec_rmw(cpu, addr_abs(cpu), rmw_asl); break;
        case 0x1E: exec_rmw(cpu, addr_abs_idx(cpu, cpu->X), rmw_asl); break;

        case 0x2A: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); cpu->A = rmw_rol(cpu, cpu->A); break;
        case 0x26: exec_rmw(cpu, addr_zp(cpu), rmw_rol); break;
        case 0x36: exec_rmw(cpu, addr_zp_idx(cpu, cpu->X), rmw_rol); break;
        case 0x2E: exec_rmw(cpu, addr_abs(cpu), rmw_rol); break;
        case 0x3E: exec_rmw(cpu, addr_abs_idx(cpu, cpu->X), rmw_rol); break;

        case 0x4A: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); cpu->A = rmw_lsr(cpu, cpu->A); break;
        case 0x46: exec_rmw(cpu, addr_zp(cpu), rmw_lsr); break;
        case 0x56: exec_rmw(cpu, addr_zp_idx(cpu, cpu->X), rmw_lsr); break;
        case 0x4E: exec_rmw(cpu, addr_abs(cpu), rmw_lsr); break;
        case 0x5E: exec_rmw(cpu, addr_abs_idx(cpu, cpu->X), rmw_lsr); break;

        case 0x6A: poll_interrupts(cpu); (void)cpu_read(cpu, cpu->PC); cpu->A = rmw_ror(cpu, cpu->A); break;
        case 0x66: exec_rmw(cpu, addr_zp(cpu), rmw_ror); break;
        case 0x76: exec_rmw(cpu, addr_zp_idx(cpu, cpu->X), rmw_ror); break;
        case 0x6E: exec_rmw(cpu, addr_abs(cpu), rmw_ror); break;
        case 0x7E: exec_rmw(cpu, addr_abs_idx(cpu, cpu->X), rmw_ror); break;

        case 0xC6: exec_rmw(cpu, addr_zp(cpu), rmw_dec); break;
        case 0xD6: exec_rmw(cpu, addr_zp_idx(cpu, cpu->X), rmw_dec); break;
        case 0xCE: exec_rmw(cpu, addr_abs(cpu), rmw_dec); break;
        case 0xDE: exec_rmw(cpu, addr_abs_idx(cpu, cpu->X), rmw_dec); break;

        case 0xE6: exec_rmw(cpu, addr_zp(cpu), rmw_inc); break;
        case 0xF6: exec_rmw(cpu, addr_zp_idx(cpu, cpu->X), rmw_inc); break;
        case 0xEE: exec_rmw(cpu, addr_abs(cpu), rmw_inc); break;
        case 0xFE: exec_rmw(cpu, addr_abs_idx(cpu, cpu->X), rmw_inc); break;

        case 0x03: exec_rmw(cpu, addr_ind_x(cpu), rmw_slo); break;
        case 0x07: exec_rmw(cpu, addr_zp(cpu),    rmw_slo); break;
        case 0x0F: exec_rmw(cpu, addr_abs(cpu),   rmw_slo); break;
        case 0x13: exec_rmw(cpu, addr_ind_y(cpu), rmw_slo); break;
        case 0x17: exec_rmw(cpu, addr_zp_idx(cpu, cpu->X),  rmw_slo); break;
        case 0x1B: exec_rmw(cpu, addr_abs_idx(cpu, cpu->Y), rmw_slo); break;
        case 0x1F: exec_rmw(cpu, addr_abs_idx(cpu, cpu->X), rmw_slo); break;

        case 0x23: exec_rmw(cpu, addr_ind_x(cpu), rmw_rla); break;
        case 0x27: exec_rmw(cpu, addr_zp(cpu),    rmw_rla); break;
        case 0x2F: exec_rmw(cpu, addr_abs(cpu),   rmw_rla); break;
        case 0x33: exec_rmw(cpu, addr_ind_y(cpu), rmw_rla); break;
        case 0x37: exec_rmw(cpu, addr_zp_idx(cpu, cpu->X),  rmw_rla); break;
        case 0x3B: exec_rmw(cpu, addr_abs_idx(cpu, cpu->Y), rmw_rla); break;
        case 0x3F: exec_rmw(cpu, addr_abs_idx(cpu, cpu->X), rmw_rla); break;

        case 0x43: exec_rmw(cpu, addr_ind_x(cpu), rmw_sre); break;
        case 0x47: exec_rmw(cpu, addr_zp(cpu),    rmw_sre); break;
        case 0x4F: exec_rmw(cpu, addr_abs(cpu),   rmw_sre); break;
        case 0x53: exec_rmw(cpu, addr_ind_y(cpu), rmw_sre); break;
        case 0x57: exec_rmw(cpu, addr_zp_idx(cpu, cpu->X),  rmw_sre); break;
        case 0x5B: exec_rmw(cpu, addr_abs_idx(cpu, cpu->Y), rmw_sre); break;
        case 0x5F: exec_rmw(cpu, addr_abs_idx(cpu, cpu->X), rmw_sre); break;

        case 0x63: exec_rmw(cpu, addr_ind_x(cpu), rmw_rra); break;
        case 0x67: exec_rmw(cpu, addr_zp(cpu),    rmw_rra); break;
        case 0x6F: exec_rmw(cpu, addr_abs(cpu),   rmw_rra); break;
        case 0x73: exec_rmw(cpu, addr_ind_y(cpu), rmw_rra); break;
        case 0x77: exec_rmw(cpu, addr_zp_idx(cpu, cpu->X),  rmw_rra); break;
        case 0x7B: exec_rmw(cpu, addr_abs_idx(cpu, cpu->Y), rmw_rra); break;
        case 0x7F: exec_rmw(cpu, addr_abs_idx(cpu, cpu->X), rmw_rra); break;

        case 0xC3: exec_rmw(cpu, addr_ind_x(cpu), rmw_dcp); break;
        case 0xC7: exec_rmw(cpu, addr_zp(cpu),    rmw_dcp); break;
        case 0xCF: exec_rmw(cpu, addr_abs(cpu),   rmw_dcp); break;
        case 0xD3: exec_rmw(cpu, addr_ind_y(cpu), rmw_dcp); break;
        case 0xD7: exec_rmw(cpu, addr_zp_idx(cpu, cpu->X),  rmw_dcp); break;
        case 0xDB: exec_rmw(cpu, addr_abs_idx(cpu, cpu->Y), rmw_dcp); break;
        case 0xDF: exec_rmw(cpu, addr_abs_idx(cpu, cpu->X), rmw_dcp); break;

        case 0xE3: exec_rmw(cpu, addr_ind_x(cpu), rmw_isc); break;
        case 0xE7: exec_rmw(cpu, addr_zp(cpu),    rmw_isc); break;
        case 0xEF: exec_rmw(cpu, addr_abs(cpu),   rmw_isc); break;
        case 0xF3: exec_rmw(cpu, addr_ind_y(cpu), rmw_isc); break;
        case 0xF7: exec_rmw(cpu, addr_zp_idx(cpu, cpu->X),  rmw_isc); break;
        case 0xFB: exec_rmw(cpu, addr_abs_idx(cpu, cpu->Y), rmw_isc); break;
        case 0xFF: exec_rmw(cpu, addr_abs_idx(cpu, cpu->X), rmw_isc); break;

        case 0x83: exec_write(cpu, addr_ind_x(cpu), (uint8_t)(cpu->A & cpu->X)); break;
        case 0x87: exec_write(cpu, addr_zp(cpu),    (uint8_t)(cpu->A & cpu->X)); break;
        case 0x8F: exec_write(cpu, addr_abs(cpu),   (uint8_t)(cpu->A & cpu->X)); break;
        case 0x97: exec_write(cpu, addr_zp_idx(cpu, cpu->Y), (uint8_t)(cpu->A & cpu->X)); break;

        case 0xA3: cpu->A = cpu->X = read_ind_x(cpu); set_nz(cpu, cpu->A); break;
        case 0xA7: cpu->A = cpu->X = read_zp(cpu);    set_nz(cpu, cpu->A); break;
        case 0xAF: cpu->A = cpu->X = read_abs(cpu);   set_nz(cpu, cpu->A); break;
        case 0xB3: cpu->A = cpu->X = read_ind_y(cpu); set_nz(cpu, cpu->A); break;
        case 0xB7: cpu->A = cpu->X = read_zp_idx(cpu, cpu->Y);  set_nz(cpu, cpu->A); break;
        case 0xBF: cpu->A = cpu->X = read_abs_idx(cpu, cpu->Y); set_nz(cpu, cpu->A); break;

        case 0x0B:
        case 0x2B:
            cpu->A &= read_imm(cpu);
            set_nz(cpu, cpu->A);
            set_flag(cpu, RICOH_2A03_FLAG_C, (cpu->A & 0x80u) != 0u);
            break;

        case 0x4B:
            cpu->A &= read_imm(cpu);
            set_flag(cpu, RICOH_2A03_FLAG_C, (cpu->A & 0x01u) != 0u);
            cpu->A >>= 1;
            set_nz(cpu, cpu->A);
            break;

        case 0x6B: {
            cpu->A &= read_imm(cpu);
            uint8_t c_in = (cpu->P & RICOH_2A03_FLAG_C) ? 0x80u : 0x00u;
            cpu->A = (uint8_t)((cpu->A >> 1) | c_in);
            set_nz(cpu, cpu->A);
            set_flag(cpu, RICOH_2A03_FLAG_C, (cpu->A & 0x40u) != 0u);
            set_flag(cpu, RICOH_2A03_FLAG_V, (((cpu->A >> 6) ^ (cpu->A >> 5)) & 0x01u) != 0u);
            break;
        }

        case 0x8B: {
            uint8_t val = read_imm(cpu);
            cpu->A = (uint8_t)((cpu->A | RICOH_2A03_UNSTABLE_CONST) & cpu->X & val);
            set_nz(cpu, cpu->A);
            break;
        }

        case 0xAB: {
            uint8_t val = read_imm(cpu);
            cpu->A = (uint8_t)((cpu->A | RICOH_2A03_LXA_CONST) & val);
            cpu->X = cpu->A;
            set_nz(cpu, cpu->A);
            break;
        }

        case 0xCB: {
            uint8_t val = read_imm(cpu);
            uint8_t ax = (uint8_t)(cpu->A & cpu->X);
            set_flag(cpu, RICOH_2A03_FLAG_C, ax >= val);
            cpu->X = (uint8_t)(ax - val);
            set_nz(cpu, cpu->X);
            break;
        }

        case 0xBB: {
            uint8_t res = (uint8_t)(read_abs_idx(cpu, cpu->Y) & cpu->SP);
            cpu->A = res;
            cpu->X = res;
            cpu->SP = res;
            set_nz(cpu, res);
            break;
        }

        case 0x93: {
            uint8_t zp = cpu_read(cpu, cpu->PC++);
            uint8_t lo = cpu_read(cpu, zp);
            uint8_t hi = cpu_read(cpu, (uint8_t)(zp + 1u));
            uint16_t base = (uint16_t)lo | ((uint16_t)hi << 8);
            uint16_t addr = (uint16_t)(base + cpu->Y);
            (void)cpu_read(cpu, (uint16_t)((base & 0xFF00u) | (addr & 0x00FFu)));
            uint8_t addr_hi = (uint8_t)(hi + 1u);
            uint8_t h = (cpu->last_read_stalls > 0u) ? 0xFFu : addr_hi;
            uint8_t val = (uint8_t)(cpu->A & cpu->X & h);
            if ((base & 0xFF00u) != (addr & 0xFF00u)) {
                addr = (uint16_t)(((uint16_t)(addr_hi & cpu->X) << 8) | (addr & 0x00FFu));
            }
            exec_write(cpu, addr, val);
            break;
        }

        case 0x9F: {
            uint8_t lo = cpu_read(cpu, cpu->PC++);
            uint8_t hi = cpu_read(cpu, cpu->PC++);
            uint16_t base = (uint16_t)lo | ((uint16_t)hi << 8);
            uint16_t addr = (uint16_t)(base + cpu->Y);
            (void)cpu_read(cpu, (uint16_t)((base & 0xFF00u) | (addr & 0x00FFu)));
            uint8_t addr_hi = (uint8_t)(hi + 1u);
            uint8_t h = (cpu->last_read_stalls > 0u) ? 0xFFu : addr_hi;
            uint8_t val = (uint8_t)(cpu->A & cpu->X & h);
            if ((base & 0xFF00u) != (addr & 0xFF00u)) {
                addr = (uint16_t)(((uint16_t)(addr_hi & cpu->X) << 8) | (addr & 0x00FFu));
            }
            exec_write(cpu, addr, val);
            break;
        }

        case 0x9B: {
            uint8_t lo = cpu_read(cpu, cpu->PC++);
            uint8_t hi = cpu_read(cpu, cpu->PC++);
            uint16_t base = (uint16_t)lo | ((uint16_t)hi << 8);
            uint16_t addr = (uint16_t)(base + cpu->Y);
            (void)cpu_read(cpu, (uint16_t)((base & 0xFF00u) | (addr & 0x00FFu)));
            cpu->SP = (uint8_t)(cpu->A & cpu->X);
            uint8_t addr_hi = (uint8_t)(hi + 1u);
            uint8_t h = (cpu->last_read_stalls > 0u) ? 0xFFu : addr_hi;
            uint8_t val = (uint8_t)(cpu->SP & h);
            if ((base & 0xFF00u) != (addr & 0xFF00u)) {
                addr = (uint16_t)(((uint16_t)(addr_hi & cpu->X) << 8) | (addr & 0x00FFu));
            }
            exec_write(cpu, addr, val);
            break;
        }

        case 0x9C: {
            uint8_t lo = cpu_read(cpu, cpu->PC++);
            uint8_t hi = cpu_read(cpu, cpu->PC++);
            uint16_t base = (uint16_t)lo | ((uint16_t)hi << 8);
            uint16_t addr = (uint16_t)(base + cpu->X);
            (void)cpu_read(cpu, (uint16_t)((base & 0xFF00u) | (addr & 0x00FFu)));
            uint8_t addr_hi = (uint8_t)(hi + 1u);
            uint8_t h = (cpu->last_read_stalls > 0u) ? 0xFFu : addr_hi;
            uint8_t val = (uint8_t)(cpu->Y & h);
            if ((base & 0xFF00u) != (addr & 0xFF00u)) {
                addr = (uint16_t)(((uint16_t)(addr_hi & cpu->Y) << 8) | (addr & 0x00FFu));
            }
            exec_write(cpu, addr, val);
            break;
        }

        case 0x9E: {
            uint8_t lo = cpu_read(cpu, cpu->PC++);
            uint8_t hi = cpu_read(cpu, cpu->PC++);
            uint16_t base = (uint16_t)lo | ((uint16_t)hi << 8);
            uint16_t addr = (uint16_t)(base + cpu->Y);
            (void)cpu_read(cpu, (uint16_t)((base & 0xFF00u) | (addr & 0x00FFu)));
            uint8_t addr_hi = (uint8_t)(hi + 1u);
            uint8_t h = (cpu->last_read_stalls > 0u) ? 0xFFu : addr_hi;
            uint8_t val = (uint8_t)(cpu->X & h);
            if ((base & 0xFF00u) != (addr & 0xFF00u)) {
                addr = (uint16_t)(((uint16_t)(addr_hi & cpu->X) << 8) | (addr & 0x00FFu));
            }
            exec_write(cpu, addr, val);
            break;
        }

        case 0x1A: case 0x3A: case 0x5A: case 0x7A:
        case 0xDA: case 0xEA: case 0xFA:
            poll_interrupts(cpu);
            (void)cpu_read(cpu, cpu->PC);
            break;

        case 0x80: case 0x82: case 0x89: case 0xC2: case 0xE2:
            (void)read_imm(cpu);
            break;

        case 0x04: case 0x44: case 0x64:
            (void)read_zp(cpu);
            break;

        case 0x14: case 0x34: case 0x54: case 0x74: case 0xD4: case 0xF4:
            (void)read_zp_idx(cpu, cpu->X);
            break;

        case 0x0C:
            (void)read_abs(cpu);
            break;

        case 0x1C: case 0x3C: case 0x5C: case 0x7C: case 0xDC: case 0xFC:
            (void)read_abs_idx(cpu, cpu->X);
            break;

        case 0x02: case 0x12: case 0x22: case 0x32:
        case 0x42: case 0x52: case 0x62: case 0x72:
        case 0x92: case 0xB2: case 0xD2: case 0xF2:
            cpu->halted = true;
            break;
    }

    return (uint32_t)(cpu->cycles - start_cycles);
}