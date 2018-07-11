/*
 * Copyright 2017, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * Copyright 2018, DornerWorks
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(DATA61_DORNERWORKS_GPL)
 */

#include <config.h>

#ifdef CONFIG_ARM_HYPERVISOR_SUPPORT

#include <machine/assembler.h>
#include <arch/api/syscall.h>
#include <arch/machine/hardware.h>
#include <arch/machine/registerset.h>

.macro lsp_i _tmp
    mrs     \_tmp, TPIDR_EL2
    mov     sp, \_tmp
.endm

.macro ventry label
.align 7
    b       \label
.endm

.section .vectors

BEGIN_FUNC(arm_vector_table)
    ventry  invalid_vector_entry       // Synchronous EL2t
    ventry  invalid_vector_entry       // IRQ EL2t
    ventry  invalid_vector_entry       // FIQ EL2t
    ventry  invalid_vector_entry       // SError EL2t

    ventry  hyp_sync                   // Synchronous EL2h
    ventry  hyp_irq                    // IRQ EL2h
    ventry  invalid_vector_entry       // FIQ EL2h
    ventry  invalid_vector_entry       // Error EL2h

    ventry  guest_sync                 // Synchronous 64-bit EL0/EL1
    ventry  guest_irq                  // IRQ 64-bit EL0/EL1
    ventry  invalid_vector_entry       // FIQ 64-bit EL0/EL1
    ventry  invalid_vector_entry       // Error 64-bit EL0/EL1

    ventry  invalid_vector_entry       // Synchronous 32-bit EL0/EL1
    ventry  invalid_vector_entry       // IRQ 32-bit EL0/EL1
    ventry  invalid_vector_entry       // FIQ 32-bit EL0/EL1
    ventry  invalid_vector_entry       // Error 32-bit EL0/EL1
END_FUNC(arm_vector_table)

.section .vectors.text

.macro kernel_enter
    /* Storing thread's stack frame */
    stp     x0,  x1,  [sp, #16 * 0]
    stp     x2,  x3,  [sp, #16 * 1]
    stp     x4,  x5,  [sp, #16 * 2]
    stp     x6,  x7,  [sp, #16 * 3]
    stp     x8,  x9,  [sp, #16 * 4]
    stp     x10, x11, [sp, #16 * 5]
    stp     x12, x13, [sp, #16 * 6]
    stp     x14, x15, [sp, #16 * 7]
    stp     x16, x17, [sp, #16 * 8]
    stp     x18, x19, [sp, #16 * 9]
    stp     x20, x21, [sp, #16 * 10]
    stp     x22, x23, [sp, #16 * 11]
    stp     x24, x25, [sp, #16 * 12]
    stp     x26, x27, [sp, #16 * 13]
    stp     x28, x29, [sp, #16 * 14]

    /* Store thread's SPSR, LR, and SP */
    mrs     x21, SP_EL1
    mrs     x22, ELR_EL2
    mrs     x23, SPSR_EL2
    stp     x30, x21, [sp, #PT_LR]
    stp     x22, x23, [sp, #PT_ELR_EL1]

.endm

BEGIN_FUNC(invalid_vector_entry)
    b       halt
END_FUNC(invalid_vector_entry)

BEGIN_FUNC(hyp_sync)
    /* Read esr_el2 and branch to respective labels*/
    mrs     x25, ESR_EL2
    lsr     x24, x25, #ESR_EC_SHIFT
    cmp     x24, #ESR_EC_DABT_SAME
    b.eq    hyp_da
    cmp     x24, #ESR_EC_IABT_SAME
    b.eq    hyp_ia
    b       hyp_inv

hyp_da:
#ifdef CONFIG_DEBUG_BUILD
    mrs     x0, ELR_EL2
    lsp_i   x19
    bl      kernelDataAbort
#endif /* CONFIG_DEBUG_BUILD */
    b       halt

hyp_ia:
#ifdef CONFIG_DEBUG_BUILD
    mrs     x0, ELR_EL2
    lsp_i   x19
    bl      kernelPrefetchAbort
#endif /* CONFIG_DEBUG_BUILD */
    b       halt

hyp_inv:
    b       invalid_vector_entry
END_FUNC(hyp_sync)

/*
 * This is only called if ksCurThread is idle thread.
 *
 * No need to store the state of idle thread and simply call c_handle_interrupt to
 * activate ksCurThread when returning from interrupt as long as idle thread is stateless.
 */
BEGIN_FUNC(hyp_irq)
    lsp_i   x19
    b       c_handle_interrupt
END_FUNC(hyp_irq)

BEGIN_FUNC(guest_sync)
    kernel_enter

    /* Read esr_el2 and branch to respective labels*/
    mrs     x25, ESR_EL2
    lsr     x24, x25, #ESR_EC_SHIFT
    cmp     x24, #ESR_EC_DABT_LOW
    b.eq    guest_da
    cmp     x24, #ESR_EC_IABT_LOW
    b.eq    guest_ia
    cmp     x24, #ESR_EC_HVC64
    b.eq    guest_svc
    cmp     x24, #ESR_EC_SYSREG
    b.eq    guest_vcpu
    cmp     x24, #ESR_EC_ENFP
    b.eq    guest_enfp
    b       guest_user

guest_da:
    mrs     x20, ELR_EL2
    str     x20, [sp, #PT_FaultInstruction]

    lsp_i   x19
    b       c_handle_data_fault

guest_ia:
    mrs     x20, ELR_EL2
    str     x20, [sp, #PT_FaultInstruction]

    lsp_i   x19
    b       c_handle_instruction_fault

guest_svc:
    mrs     x20, ELR_EL2
    sub     x20, x20, #4
    str     x20, [sp, #PT_FaultInstruction]

    lsp_i   x19
    mov     x2, x7
    b       c_handle_syscall

guest_vcpu:
    mrs     x20, ELR_EL2
    add     x20, x20, #4
    str     x20, [sp, #PT_FaultInstruction]

    lsp_i   x19
    b       c_handle_vcpu_fault

guest_enfp:
#ifdef CONFIG_HAVE_FPU
    lsp_i   x19
    b       c_handle_enfp
#endif /* CONFIG_HAVE_FPU */

guest_user:
    mrs     x20, ELR_EL2
    str     x20, [sp, #PT_FaultInstruction]

    lsp_i   x19
    b       c_handle_undefined_instruction
END_FUNC(guest_sync)

BEGIN_FUNC(guest_irq)
    kernel_enter

    mrs     x20, ELR_EL2
    str     x20, [sp, #PT_FaultInstruction]

    lsp_i   x19
    b       c_handle_interrupt
END_FUNC(guest_irq)

#endif /* CONFIG_ARM_HYP */
