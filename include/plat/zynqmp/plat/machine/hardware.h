/*
 * Copyright 2017, DornerWorks
 * Copyright 2017, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(DATA61_DORNERWORKS_GPL)
 */
/*
 * This data was produced by DornerWorks, Ltd. of Grand Rapids, MI, USA under
 * a DARPA SBIR, Contract Number D16PC00107.
 *
 * Approved for Public Release, Distribution Unlimited.
 */

#ifndef __PLAT_MACHINE_HARDWARE_H
#define __PLAT_MACHINE_HARDWARE_H
#include <config.h>
#include <types.h>
#include <basic_types.h>
#include <arch/linker.h>
#include <arch/object/vcpu.h>
#include <plat/machine.h>
#include <plat/machine/devices.h>
#include <plat_mode/machine/hardware.h>
#include <arch/machine/smmu.h>
#include <machine/io.h>

#define physBase          0x00000000

static const kernel_frame_t BOOT_RODATA kernel_devices[] = {
    {
        /* GIC */
        ACPU_GIC_CONTROLLER_PADDR,
        GIC_PL390_CONTROLLER_PPTR,
        1,
        true  /* armExecuteNever */
    },
    {
        ACPU_GIC_DISTRIBUTOR_PADDR,
        GIC_PL390_DISTRIBUTOR_PPTR,
        1,
        true  /* armExecuteNever */
#ifdef CONFIG_ARM_HYPERVISOR_SUPPORT
    },
    {
        ACPU_GIC_HYPCONTROL_PADDR,
        GIC_VCPUCTRL_PPTR,
        1,
        false
#endif /* CONFIG_ARM_HYPERVISOR */
#ifdef CONFIG_ARM_SMMU
    },
    {
        SMMU_GPV_PADDR,
        SMMU_PPTR,
        (SMMU_SIZE >> PAGE_BITS),
        false
#endif /* CONFIG_ARM_SMMU */
#ifdef CONFIG_PRINTING
    },
    {
        /*  UART */
        UART_PADDR,
        UART_PPTR,
        1,
        true  /* armExecuteNever */
#endif /* CONFIG_PRINTING */
    }
};


/* Available physical memory regions on platform (RAM) */
/* NOTE: Regions are not allowed to be adjacent! */
const p_region_t BOOT_RODATA avail_p_regs[] = {
    /* 2 GiB */
    { /* .start = */ 0x00000000, /* .end = */ 0x80000000 },
    { /* .start = */ 0x800000000, /* .end = */ 0x880000000 }
};

const p_region_t BOOT_RODATA dev_p_regs[] = {
    { /* .start = */ UART0_PADDR             , /* .end = */ UART0_PADDR              + ( 1 << PAGE_BITS)},
    { /* .start = */ UART1_PADDR             , /* .end = */ UART1_PADDR              + ( 1 << PAGE_BITS)},
    { /* .start = */ TTC0_PADDR              , /* .end = */ TTC0_PADDR               + ( 1 << PAGE_BITS)},
    { /* .start = */ TTC1_PADDR              , /* .end = */ TTC1_PADDR               + ( 1 << PAGE_BITS)},
    { /* .start = */ TTC2_PADDR              , /* .end = */ TTC2_PADDR               + ( 1 << PAGE_BITS)},
    { /* .start = */ TTC3_PADDR              , /* .end = */ TTC3_PADDR               + ( 1 << PAGE_BITS)},
    { /* .start = */ ACPU_GIC_PADDR          , /* .end = */ ACPU_GIC_PADDR           + ( 0x71000)},
    { /* .start = */ I2C0_PADDR              , /* .end = */ I2C0_PADDR               + ( 1 << PAGE_BITS)},
    { /* .start = */ I2C1_PADDR              , /* .end = */ I2C1_PADDR               + ( 1 << PAGE_BITS)},
    { /* .start = */ GPIO_PADDR              , /* .end = */ GPIO_PADDR               + ( 1 << PAGE_BITS)},
    { /* .start = */ IOU_SLCR_PADDR          , /* .end = */ IOU_SLCR_PADDR           + ( 1 << PAGE_BITS)},
    { /* .start = */ DDRC_PADDR              , /* .end = */ DDRC_PADDR               + ( 1 << PAGE_BITS)},
    { /* .start = */ RTC_PADDR               , /* .end = */ RTC_PADDR                + ( 1 << PAGE_BITS)},
    { /* .start = */ GEM3_PADDR              , /* .end = */ GEM3_PADDR               + ( 1 << PAGE_BITS)},
#ifndef CONFIG_ARM_SMMU
    { /* .start = */ SMMU_GPV_PADDR          , /* .end = */ SMMU_GPV_PADDR           + SMMU_SIZE},
#endif
};

/* Handle a platform-reserved IRQ. */
static inline void
handleReservedIRQ(irq_t irq)
{
   if ((config_set(CONFIG_ARM_HYPERVISOR_SUPPORT)) && (irq == INTERRUPT_VGIC_MAINTENANCE)) {
      VGICMaintenance();
      return;
   }

   if (config_set(CONFIG_ARM_SMMU) && (irq == INTERRUPT_SMMU)) {
      plat_smmu_handle_interrupt();
      return;
   }
}

#endif /* !__PLAT_MACHINE_HARDWARE_H */
