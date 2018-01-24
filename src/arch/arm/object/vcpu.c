/*
 * Copyright 2014, General Dynamics C4 Systems
 *
 * Copyright 2018, DornerWorks
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(GD_DORNERWORKS_GPL)
 */

#include <config.h>

#ifdef CONFIG_ARM_HYPERVISOR_SUPPORT

#include <arch/object/vcpu.h>
#include <plat/machine/devices.h>
#include <arch/machine/debug.h>
#include <arch/machine/debug_conf.h>
#include <arch/machine/fpu.h>

#define HCR_RW       BIT(31)     /* Execution state control        */
#define HCR_TRVM     BIT(30)     /* trap reads of VM controls      */
#define HCR_HCD      BIT(29)     /* Disable HVC                    */
#define HCR_TDZ      BIT(28)     /* trap DC ZVA AArch64 only       */
#define HCR_TGE      BIT(27)     /* Trap general exceptions        */
#define HCR_TVM      BIT(26)     /* Trap MMU access                */
#define HCR_TTLB     BIT(25)     /* Trap TLB operations            */
#define HCR_TPU      BIT(24)     /* Trap cache maintenance         */
#define HCR_TPC      BIT(23)     /* Trap cache maintenance PoC     */
#define HCR_TSW      BIT(22)     /* Trap cache maintenance set/way */
#define HCR_TCACHE   (HCR_TPU | HCR_TPC | HCR_TSW)
#define HCR_TAC      BIT(21)     /* Trap ACTLR access              */
#define HCR_TIDCP    BIT(20)     /* Trap lockdown                  */
#define HCR_TSC      BIT(19)     /* Trap SMC instructions          */
#define HCR_TID3     BIT(18)     /* Trap ID register 3             */
#define HCR_TID2     BIT(17)     /* Trap ID register 2             */
#define HCR_TID1     BIT(16)     /* Trap ID register 1             */
#define HCR_TID0     BIT(15)     /* Trap ID register 0             */
#define HCR_TID      (HCR_TID0 | HCR_TID1 | HCR_TID2 | HCR_TID3)
#define HCR_TWE      BIT(14)     /* Trap WFE                       */
#define HCR_TWI      BIT(13)     /* Trap WFI                       */
#define HCR_DC       BIT(12)     /* Default cacheable              */
#define HCR_BSU(x)   ((x) << 10) /* Barrier sharability upgrade    */
#define HCR_FB       BIT( 9)     /* Force broadcast                */
#define HCR_VA       BIT( 8)     /* Virtual async abort            */
#define HCR_VI       BIT( 7)     /* Virtual IRQ                    */
#define HCR_VF       BIT( 6)     /* Virtual FIRQ                   */
#define HCR_AMO      BIT( 5)     /* CPSR.A override enable         */
#define HCR_IMO      BIT( 4)     /* CPSR.I override enable         */
#define HCR_FMO      BIT( 3)     /* CPSR.F override enable         */
#define HCR_PTW      BIT( 2)     /* Protected table walk           */
#define HCR_SWIO     BIT( 1)     /* set/way invalidate override    */
#define HCR_VM       BIT( 0)     /* Virtualization MMU enable      */

#define SCTLR_RES1 BIT(11) | BIT(22) | BIT(23) | BIT(28) | BIT(29)
#define SCTLR_WFE  BIT(18)
#define SCTLR_WFI  BIT(16)
#define SCTLR_I    BIT(12)

#define VGIC_HCR_EOI_INVALID_COUNT(hcr) (((hcr) >> 27) & 0x1f)
#define VGIC_HCR_VGRP1DIE               (1U << 7)
#define VGIC_HCR_VGRP1EIE               (1U << 6)
#define VGIC_HCR_VGRP0DIE               (1U << 5)
#define VGIC_HCR_VGRP0EIE               (1U << 4)
#define VGIC_HCR_NPIE                   (1U << 3)
#define VGIC_HCR_LRENPIE                (1U << 2)
#define VGIC_HCR_UIE                    (1U << 1)
#define VGIC_HCR_EN                     (1U << 0)
#define VGIC_MISR_VGRP1D                VGIC_HCR_VGRP1DIE
#define VGIC_MISR_VGRP1E                VGIC_HCR_VGRP1EIE
#define VGIC_MISR_VGRP0D                VGIC_HCR_VGRP0DIE
#define VGIC_MISR_VGRP0E                VGIC_HCR_VGRP0EIE
#define VGIC_MISR_NP                    VGIC_HCR_NPIE
#define VGIC_MISR_LRENP                 VGIC_HCR_LRENPIE
#define VGIC_MISR_U                     VGIC_HCR_UIE
#define VGIC_MISR_EOI                   VGIC_HCR_EN
#define VGIC_VTR_NLISTREGS(vtr)         ((((vtr) >>  0) & 0x3f) + 1)
#define VGIC_VTR_NPRIOBITS(vtr)         ((((vtr) >> 29) & 0x07) + 1)
#define VGIC_VTR_NPREBITS(vtr)          ((((vtr) >> 26) & 0x07) + 1)

struct gich_vcpu_ctrl_map {
    uint32_t hcr;    /* 0x000 RW 0x00000000 Hypervisor Control Register */
    uint32_t vtr;    /* 0x004 RO IMPLEMENTATION DEFINED VGIC Type Register */
    /* Save restore on VCPU switch */
    uint32_t vmcr;   /* 0x008 RW IMPLEMENTATION DEFINED Virtual Machine Control Register */
    uint32_t res1[1];
    /* IRQ pending flags */
    uint32_t misr;   /* 0x010 RO 0x00000000 Maintenance Interrupt Status Register */
    uint32_t res2[3];
    /* Bitfield of list registers that have EOI */
    uint32_t eisr0;  /* 0x020 RO 0x00000000 End of Interrupt Status Registers 0 and 1, see EISRn */
    uint32_t eisr1;  /* 0x024 RO 0x00000000 */
    uint32_t res3[2];
    /* Bitfield of list registers that are empty */
    uint32_t elsr0;  /* 0x030 RO IMPLEMENTATION DEFINED a */
    uint32_t elsr1;  /* 0x034 RO IMPLEMENTATION DEFINED a Empty List Register Status Registers 0 and 1, see ELRSRn */
    uint32_t res4[46];
    /* Active priority: bitfield of active priorities */
    uint32_t apr;    /* 0x0F0 RW 0x00000000 Active Priorities Register */
    uint32_t res5[3];
    uint32_t lr[64]; /* 0x100 RW 0x00000000 List Registers 0-63, see LRn */
};

#ifndef GIC_PL400_VCPUCTRL_PPTR
#error GIC_PL400_VCPUCTRL_PPTR must be defined for virtual memory access to the gic virtual cpu interface control
#else  /* GIC_PL400_GICVCPUCTRL_PPTR */
static volatile struct gich_vcpu_ctrl_map *gic_vcpu_ctrl =
    (volatile struct gich_vcpu_ctrl_map*)(GIC_PL400_VCPUCTRL_PPTR);
#endif /* GIC_PL400_GICVCPUCTRL_PPTR */

static inline uint32_t
get_gic_vcpu_ctrl_hcr(void)
{
    return gic_vcpu_ctrl->hcr;
}

static inline void
set_gic_vcpu_ctrl_hcr(uint32_t hcr)
{
    gic_vcpu_ctrl->hcr = hcr;
}

static inline uint32_t
get_gic_vcpu_ctrl_vmcr(void)
{
    return gic_vcpu_ctrl->vmcr;
}

static inline void
set_gic_vcpu_ctrl_vmcr(uint32_t vmcr)
{
    gic_vcpu_ctrl->vmcr = vmcr;
}

static inline uint32_t
get_gic_vcpu_ctrl_apr(void)
{
    return gic_vcpu_ctrl->apr;
}

static inline void
set_gic_vcpu_ctrl_apr(uint32_t apr)
{
    gic_vcpu_ctrl->apr = apr;
}

static inline uint32_t
get_gic_vcpu_ctrl_vtr(void)
{
    return gic_vcpu_ctrl->vtr;
}

static inline uint32_t
get_gic_vcpu_ctrl_eisr0(void)
{
    return gic_vcpu_ctrl->eisr0;
}

static inline uint32_t
get_gic_vcpu_ctrl_eisr1(void)
{
    return gic_vcpu_ctrl->eisr1;
}

static inline uint32_t
get_gic_vcpu_ctrl_misr(void)
{
    return gic_vcpu_ctrl->misr;
}

static inline virq_t
get_gic_vcpu_ctrl_lr(int num)
{
    virq_t virq;
    virq.words[0] = gic_vcpu_ctrl->lr[num];
    return virq;
}

static inline void
set_gic_vcpu_ctrl_lr(int num, virq_t lr)
{
    gic_vcpu_ctrl->lr[num] = lr.words[0];
}

static unsigned int gic_vcpu_num_list_regs;

word_t vcpu_read_reg(vcpu_t *vcpu, word_t reg);
void vcpu_write_reg(vcpu_t *vcpu, word_t reg, word_t value);
void vcpu_save_reg(vcpu_t *vcpu, word_t reg);
void vcpu_save_reg_range(vcpu_t *vcpu, word_t start, word_t end);
void vcpu_restore_reg(vcpu_t *vcpu, word_t reg);
void vcpu_restore_reg_range(vcpu_t *vcpu, word_t start, word_t end);

#ifdef CONFIG_ARCH_AARCH64

/* Virtual Translation Control Register */
#define VTCR_RES1  BIT(31)       /* Res1 */
#define VTCR_DS    BIT(22)       /* Hardware Management of Dirty State */
#define VTCR_HA    BIT(21)       /* Hardware Access Flag */
#define VTCR_VS    BIT(19)       /* VMID Size */
#define VTCR_4G    (0 << 16)
#define VTCR_64G   (1 << 16)
#define VTCR_1T    (2 << 16)
#define VTCR_4T    (3 << 16)
#define VTCR_16T   (4 << 16)
#define VTCR_256T  (5 << 16)
#define VTCR_4P    (6 << 16)
#define VTCR_4KB   (0 << 14)     /* Granule Size 4kB */
#define VTCR_64KB  (1 << 14)     /* Granule Size 64kB */
#define VTCR_16KB  (2 << 14)     /* Granule Size 16kB */
#define VTCR_NS    (0 << 12)     /* Non Shareable */
#define VTCR_OS    (2 << 12)     /* Outer Shareable */
#define VTCR_IS    (3 << 12)     /* Inner Shareable */
#define VTCR_ON    (0 << 10)
#define VTCR_OWBWA (1 << 10)
#define VTCR_OWTRA (2 << 10)
#define VTCR_OWBRA (3 << 10)
#define VTCR_IN    (0 << 8)
#define VTCR_IWBWA (1 << 8)
#define VTCR_IWTRA (2 << 8)
#define VTCR_IWBRA (3 << 8)
#define VTCR_SL2   (0 << 6)     /* If TG0 is 0, Start at L2 */
#define VTCR_SL1   (1 << 6)     /* If TG0 is 0, Start at L1 */
#define VTCR_SL0   (2 << 6)     /* If TG0 is 0, Start at L0 */

#define VTCR_T0SZ(x) (x & 0x3f) /* Mask 6 Bits */

/* note that the HCR_DC for ARMv8 disables S1 translation if enabled */
#define HCR_COMMON ( HCR_TWE | HCR_VM | HCR_AMO | HCR_IMO | HCR_FMO | \
                     HCR_RW  | HCR_BSU(0x1) | HCR_PTW | HCR_FB | HCR_SWIO)

#define HCR_NATIVE ( HCR_COMMON | HCR_TVM | HCR_TTLB | HCR_TAC | HCR_TSC | HCR_DC)
#define HCR_VCPU   HCR_COMMON

#define SCTLR_EL1_NATIVE  SCTLR_RES1 | SCTLR_WFE | SCTLR_WFI | SCTLR_I
#define SCTLR_EL1_VM      SCTLR_EL1_NATIVE
#define SCTLR_DEFAULT     SCTLR_EL1_NATIVE

#ifdef CONFIG_ARM_CORTEX_A53
/* VTCR Limited to 40-bit PA space on Cortex A53 */
#define VTCR_EL2_NATIVE  VTCR_RES1 | VTCR_1T | VTCR_IS | VTCR_OWBWA | VTCR_IWBWA | VTCR_SL1 | VTCR_T0SZ(25)
#endif

#define FPU_FAULT  0x2000000
#define WFI_FAULT  0x7e00000
#define WFE_FAULT  0x7e00001

static inline word_t
readTTBR0(void)
{
    word_t reg;
    MRS("ttbr0_el1", reg);
    return reg;
}

static inline void
writeTTBR0(word_t reg)
{
    MSR("ttbr0_el1", reg);
}

static inline word_t
readTTBR1(void)
{
    word_t reg;
    MRS("ttbr1_el1", reg);
    return reg;
}

static inline void
writeTTBR1(word_t reg)
{
    MSR("ttbr1_el1", reg);
}

static inline word_t
readTCR(void)
{
    word_t reg;
    MRS("tcr_el1", reg);
    return reg;
}

static inline void
writeTCR(word_t reg)
{
    MSR("tcr_el1", reg);
}

static inline word_t
readMAIR(void)
{
    word_t reg;
    MRS("mair_el1", reg);
    return reg;
}

static inline void
writeMAIR(word_t reg)
{
    MSR("mair_el1", reg);
}

static inline word_t
readAMAIR(void)
{
    word_t reg;
    MRS("amair_el1", reg);
    return reg;
}

static inline void
writeAMAIR(word_t reg)
{
    MSR("amair_el1", reg);
}

static inline word_t
readCIDR(void)
{
    uint32_t reg;
    MRS("contextidr_el1", reg);
    return (word_t)reg;
}

static inline void
writeCIDR(word_t reg)
{
    MSR("contextidr_el1", (uint32_t)reg);
}

static inline word_t
readACTLR(void)
{
    word_t reg;
    MRS("actlr_el1", reg);
    return reg;
}

static inline void
writeACTLR(word_t reg)
{
    MSR("actlr_el1", reg);
}

static inline word_t
readAFSR0(void)
{
    uint32_t reg;
    MRS("afsr0_el1", reg);
    return (word_t)reg;
}

static inline void
writeAFSR0(word_t reg)
{
    MSR("afsr0_el1", (uint32_t)reg);
}

static inline word_t
readAFSR1(void)
{
    uint32_t reg;
    MRS("afsr1_el1", reg);
    return (word_t)reg;
}

static inline void
writeAFSR1(word_t reg)
{
    MSR("afsr1_el1", (uint32_t)reg);
}

static inline word_t
readESR(void)
{
    uint32_t reg;
    MRS("esr_el1", reg);
    return (word_t)reg;
}

static inline void
writeESR(word_t reg)
{
    MSR("esr_el1", (uint32_t)reg);
}

static inline word_t
readFAR(void)
{
    word_t reg;
    MRS("far_el1", reg);
    return reg;
}

static inline void
writeFAR(word_t reg)
{
    MSR("far_el1", reg);
}

/* ISR is read-only */
static inline word_t
readISR(void)
{
    uint32_t reg;
    MRS("isr_el1", reg);
    return (word_t)reg;
}

static inline word_t
readVBAR(void)
{
    word_t vbar;
    MRS("vbar_el1", vbar);
    return vbar;
}

static inline void
writeVBAR(word_t reg)
{
    MSR("vbar_el1", reg);
}

static inline word_t
readTPIDR_EL0(void)
{
    word_t tpidr;
    MRS("tpidr_el0", tpidr);
    return tpidr;
}

static inline void
writeTPIDR_EL0(word_t reg)
{
    MSR("tpidr_el0", reg);
}

static inline word_t
readTPIDR_EL1(void)
{
    word_t tpidr;
    MRS("tpidr_el1", tpidr);
    return tpidr;
}

static inline void
writeTPIDR_EL1(word_t reg)
{
    MSR("tpidr_el1", reg);
}

static inline word_t
readTPIDRRO_EL0(void)
{
    word_t tpidrro;
    MRS("tpidrro_el0", tpidrro);
    return tpidrro;
}

static inline void
writeTPIDRRO_EL0(word_t reg)
{
    MSR("tpidrro_el0", reg);
}

static inline word_t
readSP_EL1(void)
{
    word_t sp;
    MRS("sp_el1", sp);
    return sp;
}

static inline void
writeSP_EL1(word_t reg)
{
    MSR("sp_el1", reg);
}

static inline word_t
readSP_EL0(void)
{
    word_t sp;
    MRS("sp_el0", sp);
    return sp;
}

static inline void
writeSP_EL0(word_t reg)
{
    MSR("sp_el0", reg);
}

static inline word_t
readELR_EL1(void)
{
    word_t elr;
    MRS("elr_el1", elr);
    return elr;
}

static inline void
writeELR_EL1(word_t reg)
{
    MSR("elr_el1", reg);
}

static inline word_t
readSPSR_EL1(void)
{
    word_t spsr;
    MRS("spsr_el1", spsr);
    return spsr;
}

static inline void
writeSPSR_EL1(word_t reg)
{
    MSR("spsr_el1", reg);
}

static inline void
writeCNTV_CTL(word_t reg)
{
    isb();
    MSR("cntv_ctl_el0", reg);
    isb();
}

static inline word_t
readCNTV_CTL(void)
{
    word_t ctl;
    MRS("cntv_ctl_el0", ctl);
    return ctl;
}

static inline void
writeCNTV_CVAL(word_t reg)
{
    MSR("cntv_cval_el0", reg);
    isb();
}

static inline word_t
readCNTV_CVAL(void)
{
    word_t cval;
    MRS("cntv_cval_el0", cval);
    return cval;
}

static inline void
writeCNTV_TVAL(word_t reg)
{
    MSR("cntv_tval_el0", reg);
    isb();
}

static inline word_t
readCNTV_TVAL(void)
{
    word_t tval;
    MRS("cntv_tval_el0", tval);
    return tval;
}

static inline void
writeHCR(word_t reg)
{
    MSR("hcr_el2", reg);
    isb();
}

static inline void
writeVTCR(word_t reg)
{
    MSR("vtcr_el2", reg);
    isb();
}

static inline word_t
getSCTLR(void)
{
    return readSystemControlRegister();
}

static inline void
setSCTLR(word_t sctlr)
{
    writeSystemControlRegister(sctlr);
}

static unsigned int gic_vcpu_num_list_regs;

BOOT_CODE void
vcpu_boot_init(void)
{
    /* set up the stage-2 translation control register */
    writeVTCR(VTCR_EL2_NATIVE);
    writeHCR(HCR_NATIVE);

    /* set the SCTLR_EL1 for running native seL4 threads */
    setSCTLR(SCTLR_EL1_NATIVE);

    gic_vcpu_num_list_regs = VGIC_VTR_NLISTREGS(get_gic_vcpu_ctrl_vtr());
    if (gic_vcpu_num_list_regs > GIC_VCPU_MAX_NUM_LR) {
        printf("Warning: VGIC is reporting more list registers than we support. Truncating\n");
        gic_vcpu_num_list_regs = GIC_VCPU_MAX_NUM_LR;
    }

    armHSCurVCPU = NULL;
    armHSVCPUActive = false;
}

void
handleVCPUFault(word_t hsr)
{
    current_fault = seL4_Fault_VCPUFault_new(hsr);
    handleFault(ksCurThread);
    schedule();
    activateThread();
}

static word_t
hw_read_reg(word_t reg_index)
{
    if (reg_index >= seL4_VCPUReg_Num) fail("ARM/HYP: Invalid register index");
    word_t reg = 0;
    switch (reg_index) {
        case seL4_VCPUReg_SCTLR:
            return getSCTLR();
        case seL4_VCPUReg_TTBR0:
            return readTTBR0();
        case seL4_VCPUReg_TTBR1:
            return readTTBR1();
        case seL4_VCPUReg_TCR:
            return readTCR();
        case seL4_VCPUReg_MAIR:
            return readMAIR();
        case seL4_VCPUReg_AMAIR:
            return readAMAIR();
        case seL4_VCPUReg_CIDR:
            return readCIDR();
        case seL4_VCPUReg_ACTLR:
            return readACTLR();
        case seL4_VCPUReg_CPACR:
            /* skip CPACR for the moment */
            return 0;
        case seL4_VCPUReg_AFSR0:
            return readAFSR0();
        case seL4_VCPUReg_AFSR1:
            return readAFSR1();
        case seL4_VCPUReg_ESR:
            return readESR();
        case seL4_VCPUReg_FAR:
            return readFAR();
        case seL4_VCPUReg_ISR:
            return readISR();
        case seL4_VCPUReg_VBAR:
            return readVBAR();
        case seL4_VCPUReg_TPIDR_EL0:
            return readTPIDR_EL0();
        case seL4_VCPUReg_TPIDR_EL1:
            return readTPIDR_EL1();
        case seL4_VCPUReg_TPIDRRO_EL0:
            return readTPIDRRO_EL0();
        case seL4_VCPUReg_CNTV_TVAL:
            return readCNTV_TVAL();
        case seL4_VCPUReg_CNTV_CTL:
            return readCNTV_CTL();
        case seL4_VCPUReg_CNTV_CVAL:
            return readCNTV_CVAL();
        case seL4_VCPUReg_SP_EL0:
            return readSP_EL0();
        case seL4_VCPUReg_SP_EL1:
            return readSP_EL1();
        case seL4_VCPUReg_ELR_EL1:
            return readELR_EL1();
        case seL4_VCPUReg_SPSR_EL1:
            return readSPSR_EL1();
        default:
            fail("ARM/HYP: Invalid register index");
    }

    return reg;
}

static void
hw_write_reg(word_t reg_index, word_t reg)
{
    if (reg_index >= seL4_VCPUReg_Num) fail("ARM/HYP: Invalid register index");
    switch (reg_index) {
        case seL4_VCPUReg_SCTLR:
            return setSCTLR(reg);
        case seL4_VCPUReg_TTBR0:
            return writeTTBR0(reg);
        case seL4_VCPUReg_TTBR1:
            return writeTTBR1(reg);
        case seL4_VCPUReg_TCR:
            return writeTCR(reg);
        case seL4_VCPUReg_MAIR:
            return writeMAIR(reg);
        case seL4_VCPUReg_AMAIR:
            return writeAMAIR(reg);
        case seL4_VCPUReg_CIDR:
            return writeCIDR(reg);
        case seL4_VCPUReg_ACTLR:
            return writeACTLR(reg);
        case seL4_VCPUReg_CPACR:
            return;
        case seL4_VCPUReg_AFSR0:
            return writeAFSR0(reg);
        case seL4_VCPUReg_AFSR1:
            return writeAFSR1(reg);
        case seL4_VCPUReg_ESR:
            return writeESR(reg);
        case seL4_VCPUReg_FAR:
            return writeFAR(reg);
        case seL4_VCPUReg_ISR:
            return;
        case seL4_VCPUReg_VBAR:
            return writeVBAR(reg);
        case seL4_VCPUReg_TPIDR_EL0:
            return writeTPIDR_EL0(reg);
        case seL4_VCPUReg_TPIDR_EL1:
            return writeTPIDR_EL1(reg);
        case seL4_VCPUReg_TPIDRRO_EL0:
            return writeTPIDRRO_EL0(reg);
        case seL4_VCPUReg_CNTV_TVAL:
            return writeCNTV_TVAL(reg);
        case seL4_VCPUReg_CNTV_CTL:
            return writeCNTV_CTL(reg);
        case seL4_VCPUReg_CNTV_CVAL:
            return writeCNTV_CVAL(reg);
        case seL4_VCPUReg_SP_EL0:
            return writeSP_EL0(reg);
        case seL4_VCPUReg_SP_EL1:
            return writeSP_EL1(reg);
        case seL4_VCPUReg_ELR_EL1:
            return writeELR_EL1(reg);
        case seL4_VCPUReg_SPSR_EL1:
            return writeSPSR_EL1(reg);
        default:
            fail("ARM/HYP: Invalid register index");
    }

    return;
}

static void
vcpu_enable(vcpu_t *vcpu)
{
    writeHCR(HCR_VCPU);
    vcpu_restore_reg(vcpu, seL4_VCPUReg_SCTLR);
    isb();

    set_gic_vcpu_ctrl_hcr(vcpu->vgic.hcr);
}

static void
vcpu_disable(vcpu_t *vcpu)
{
    uint32_t hcr;
    dsb();
    if (likely(vcpu)) {
        hcr = get_gic_vcpu_ctrl_hcr();
        vcpu->vgic.hcr = hcr;
        vcpu_save_reg(vcpu, seL4_VCPUReg_SCTLR);
        isb();
    }
    /* Turn off the VGIC */
    set_gic_vcpu_ctrl_hcr(0);
    isb();

    /* Stage 1 MMU off */
    setSCTLR(SCTLR_DEFAULT);
    isb();
    writeHCR(HCR_NATIVE);
    isb();
}

#else  /* CONFIG_ARCH_AARCH32 */

/* Trap WFI/WFE/SMC and override CPSR.AIF */
#define HCR_COMMON ( HCR_TSC | HCR_TWE | HCR_TWI | HCR_AMO | HCR_IMO \
                   | HCR_FMO | HCR_DC  | HCR_VM)
/* Allow native tasks to run at PL1, but restrict access */
#define HCR_NATIVE ( HCR_COMMON | HCR_TGE | HCR_TVM | HCR_TTLB | HCR_TCACHE \
                   | HCR_TAC | HCR_SWIO)
#define HCR_VCPU   (HCR_COMMON)

/* Amongst other things we set the caches to enabled by default. This
 * may cause problems when booting guests that expect caches to be
 * disabled */
#define SCTLR_DEFAULT 0xc5187c
#define ACTLR_DEFAULT 0x40

static inline word_t
get_lr_svc(void)
{
    word_t ret;
    asm ("mrs %[ret], lr_svc" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_lr_svc(word_t val)
{
    asm ("msr lr_svc, %[val]" :: [val]"r"(val));
}

static inline word_t
get_sp_svc(void)
{
    word_t ret;
    asm ("mrs %[ret], sp_svc" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_sp_svc(word_t val)
{
    asm ("msr sp_svc, %[val]" :: [val]"r"(val));
}

static inline word_t
get_spsr_svc(void)
{
    word_t ret;
    asm ("mrs %[ret], spsr_svc" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_spsr_svc(word_t val)
{
    asm ("msr spsr_svc, %[val]" :: [val]"r"(val));
}

static inline word_t
get_lr_abt(void)
{
    word_t ret;
    asm ("mrs %[ret], lr_abt" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_lr_abt(word_t val)
{
    asm ("msr lr_abt, %[val]" :: [val]"r"(val));
}

static inline word_t
get_sp_abt(void)
{
    word_t ret;
    asm ("mrs %[ret], sp_abt" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_sp_abt(word_t val)
{
    asm ("msr sp_abt, %[val]" :: [val]"r"(val));
}

static inline word_t
get_spsr_abt(void)
{
    word_t ret;
    asm ("mrs %[ret], spsr_abt" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_spsr_abt(word_t val)
{
    asm ("msr spsr_abt, %[val]" :: [val]"r"(val));
}

static inline word_t
get_lr_und(void)
{
    word_t ret;
    asm ("mrs %[ret], lr_und" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_lr_und(word_t val)
{
    asm ("msr lr_und, %[val]" :: [val]"r"(val));
}

static inline word_t
get_sp_und(void)
{
    word_t ret;
    asm ("mrs %[ret], sp_und" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_sp_und(word_t val)
{
    asm ("msr sp_und, %[val]" :: [val]"r"(val));
}

static inline word_t
get_spsr_und(void)
{
    word_t ret;
    asm ("mrs %[ret], spsr_und" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_spsr_und(word_t val)
{
    asm ("msr spsr_und, %[val]" :: [val]"r"(val));
}

static inline word_t
get_lr_irq(void)
{
    word_t ret;
    asm ("mrs %[ret], lr_irq" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_lr_irq(word_t val)
{
    asm ("msr lr_irq, %[val]" :: [val]"r"(val));
}

static inline word_t
get_sp_irq(void)
{
    word_t ret;
    asm ("mrs %[ret], sp_irq" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_sp_irq(word_t val)
{
    asm ("msr sp_irq, %[val]" :: [val]"r"(val));
}

static inline word_t
get_spsr_irq(void)
{
    word_t ret;
    asm ("mrs %[ret], spsr_irq" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_spsr_irq(word_t val)
{
    asm ("msr spsr_irq, %[val]" :: [val]"r"(val));
}

static inline word_t
get_lr_fiq(void)
{
    word_t ret;
    asm ("mrs %[ret], lr_fiq" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_lr_fiq(word_t val)
{
    asm ("msr lr_fiq, %[val]" :: [val]"r"(val));
}

static inline word_t
get_sp_fiq(void)
{
    word_t ret;
    asm ("mrs %[ret], sp_fiq" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_sp_fiq(word_t val)
{
    asm ("msr sp_fiq, %[val]" :: [val]"r"(val));
}

static inline word_t
get_spsr_fiq(void)
{
    word_t ret;
    asm ("mrs %[ret], spsr_fiq" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_spsr_fiq(word_t val)
{
    asm ("msr spsr_fiq, %[val]" :: [val]"r"(val));
}

static inline word_t
get_r8_fiq(void)
{
    word_t ret;
    asm ("mrs %[ret], r8_fiq" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_r8_fiq(word_t val)
{
    asm ("msr r8_fiq, %[val]" :: [val]"r"(val));
}

static inline word_t
get_r9_fiq(void)
{
    word_t ret;
    asm ("mrs %[ret], r9_fiq" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_r9_fiq(word_t val)
{
    asm ("msr r9_fiq, %[val]" :: [val]"r"(val));
}

static inline word_t
get_r10_fiq(void)
{
    word_t ret;
    asm ("mrs %[ret], r10_fiq" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_r10_fiq(word_t val)
{
    asm ("msr r10_fiq, %[val]" :: [val]"r"(val));
}

static inline word_t
get_r11_fiq(void)
{
    word_t ret;
    asm ("mrs %[ret], r11_fiq" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_r11_fiq(word_t val)
{
    asm ("msr r11_fiq, %[val]" :: [val]"r"(val));
}

static inline word_t
get_r12_fiq(void)
{
    word_t ret;
    asm ("mrs %[ret], r12_fiq" : [ret]"=r"(ret));
    return ret;
}

static inline void
set_r12_fiq(word_t val)
{
    asm ("msr r12_fiq, %[val]" :: [val]"r"(val));
}

static word_t
hw_read_reg(word_t reg_index)
{
    if (reg_index >= seL4_VCPUReg_Num) fail("ARM/HYP: Invalid register index");
    word_t reg = 0;
    switch (reg_index) {
        case seL4_VCPUReg_SCTLR:
            return getSCTLR();
        case seL4_VCPUReg_ACTLR:
            return getACTLR();
        case seL4_VCPUReg_TTBRC:
            return readTTBRC();
        case seL4_VCPUReg_TTBR0:
            return readTTBR0();
        case seL4_VCPUReg_TTBR1:
            return readTTBR1();
        case seL4_VCPUReg_DACR:
            return readDACR();
        case seL4_VCPUReg_DFSR:
            return getDFSR();
        case seL4_VCPUReg_IFSR:
            return getIFSR();
        case seL4_VCPUReg_ADFSR:
            return getADFSR();
        case seL4_VCPUReg_AIFSR:
            return getAIFSR();
        case seL4_VCPUReg_DFAR:
            return getDFAR();
        case seL4_VCPUReg_IFAR:
            return getIFAR();
        case seL4_VCPUReg_PRRR:
            return getPRRR();
        case seL4_VCPUReg_NMRR:
            return getNMRR();
        case seL4_VCPUReg_CIDR:
            return getCIDR();
        case seL4_VCPUReg_TPIDRPRW:
            return readTPIDRPRW();
        case seL4_VCPUReg_TPIDRURO:
            return readTPIDRURO();
        case seL4_VCPUReg_TPIDRURW:
            return readTPIDRURW();
        case seL4_VCPUReg_FPEXC:
            return reg;
        case seL4_VCPUReg_CNTV_TVAL:
            MRC(CNTV_TVAL, reg);
            return reg;
        case seL4_VCPUReg_CNTV_CTL:
            MRC(CNTV_CTL, reg);
            return reg;
        case seL4_VCPUReg_CNTV_CVAL:
            return reg;
        case seL4_VCPUReg_LRsvc:
            return get_lr_svc();
        case seL4_VCPUReg_SPsvc:
            return get_sp_svc();
        case seL4_VCPUReg_LRabt:
            return get_lr_abt();
        case seL4_VCPUReg_SPabt:
            return get_sp_abt();
        case seL4_VCPUReg_LRund:
            return get_lr_und();
        case seL4_VCPUReg_SPund:
            return get_sp_und();
        case seL4_VCPUReg_LRirq:
            return get_lr_irq();
        case seL4_VCPUReg_SPirq:
            return get_sp_irq();
        case seL4_VCPUReg_LRfiq:
            return get_lr_fiq();
        case seL4_VCPUReg_SPfiq:
            return get_sp_fiq();
        case seL4_VCPUReg_R8fiq:
            return get_r8_fiq();
        case seL4_VCPUReg_R9fiq:
            return get_r9_fiq();
        case seL4_VCPUReg_R10fiq:
            return get_r10_fiq();
        case seL4_VCPUReg_R11fiq:
            return get_r11_fiq();
        case seL4_VCPUReg_R12fiq:
            return get_r12_fiq();
        case seL4_VCPUReg_SPSRsvc:
            return get_spsr_svc();
        case seL4_VCPUReg_SPSRabt:
            return get_spsr_abt();
        case seL4_VCPUReg_SPSRund:
            return get_spsr_und();
        case seL4_VCPUReg_SPSRirq:
            return get_spsr_irq();
        case seL4_VCPUReg_SPSRfiq:
            return get_spsr_fiq();
        default:
            fail("ARM/HYP: Invalid register index");
    }
}

static void
hw_write_reg(word_t reg_index, word_t reg)
{
    if (reg_index >= seL4_VCPUReg_Num) return;

    switch (reg_index) {
        case seL4_VCPUReg_SCTLR:
            return setSCTLR(reg);
        case seL4_VCPUReg_ACTLR:
            return setACTLR(reg);
        case seL4_VCPUReg_TTBRC:
            return writeTTBRC(reg);
        case seL4_VCPUReg_TTBR0:
            return writeTTBR0Raw(reg);
        case seL4_VCPUReg_TTBR1:
            return writeTTBR1Raw(reg);
        case seL4_VCPUReg_DACR:
            return writeDACR(reg);
        case seL4_VCPUReg_DFSR:
            return setDFSR(reg);
        case seL4_VCPUReg_IFSR:
            return setIFSR(reg);
        case seL4_VCPUReg_ADFSR:
            return setADFSR(reg);
        case seL4_VCPUReg_AIFSR:
            return setAIFSR(reg);
        case seL4_VCPUReg_DFAR:
            return setDFAR(reg);
        case seL4_VCPUReg_IFAR:
            return setIFAR(reg);
        case seL4_VCPUReg_PRRR:
            return setPRRR(reg);
        case seL4_VCPUReg_NMRR:
            return setNMRR(reg);
        case seL4_VCPUReg_CIDR:
            return setCIDR(reg);
        case seL4_VCPUReg_TPIDRPRW:
            return writeTPIDRPRW(reg);
        case seL4_VCPUReg_TPIDRURO:
            return writeTPIDRURO(reg);
        case seL4_VCPUReg_TPIDRURW:
            return writeTPIDRURW(reg);
        case seL4_VCPUReg_FPEXC:
            return;
        case seL4_VCPUReg_CNTV_TVAL:
            MCR(CNTV_TVAL, reg);
            return;
        case seL4_VCPUReg_CNTV_CTL:
            MCR(CNTV_CTL, reg);
            return;
        case seL4_VCPUReg_CNTV_CVAL:
            return;
        case seL4_VCPUReg_LRsvc:
            return set_lr_svc(reg);
        case seL4_VCPUReg_SPsvc:
            return set_sp_svc(reg);
        case seL4_VCPUReg_LRabt:
            return set_lr_abt(reg);
        case seL4_VCPUReg_SPabt:
            return set_sp_abt(reg);
        case seL4_VCPUReg_LRund:
            return set_lr_und(reg);
        case seL4_VCPUReg_SPund:
            return set_sp_und(reg);
        case seL4_VCPUReg_LRirq:
            return set_lr_irq(reg);
        case seL4_VCPUReg_SPirq:
            return set_sp_irq(reg);
        case seL4_VCPUReg_LRfiq:
            return set_lr_fiq(reg);
        case seL4_VCPUReg_SPfiq:
            return set_sp_fiq(reg);
        case seL4_VCPUReg_R8fiq:
            return set_r8_fiq(reg);
        case seL4_VCPUReg_R9fiq:
            return set_r9_fiq(reg);
        case seL4_VCPUReg_R10fiq:
            return set_r10_fiq(reg);
        case seL4_VCPUReg_R11fiq:
            return set_r11_fiq(reg);
        case seL4_VCPUReg_R12fiq:
            return set_r12_fiq(reg);
        case seL4_VCPUReg_SPSRsvc:
            return set_spsr_svc(reg);
        case seL4_VCPUReg_SPSRabt:
            return set_spsr_abt(reg);
        case seL4_VCPUReg_SPSRund:
            return set_spsr_und(reg);
        case seL4_VCPUReg_SPSRirq:
            return set_spsr_irq(reg);
        case seL4_VCPUReg_SPSRfiq:
            return set_spsr_fiq(reg);
        default:
            fail("ARM/HYP: Invalid register index");
    }
}


static void
vcpu_enable(vcpu_t *vcpu)
{
    vcpu_restore_reg(vcpu, seL4_VCPUReg_SCTLR);
    setHCR(HCR_VCPU);
    isb();

    /* Turn on the VGIC */
    set_gic_vcpu_ctrl_hcr(vcpu->vgic.hcr);

#if !defined(ARM_CP14_SAVE_AND_RESTORE_NATIVE_THREADS) && defined(ARM_HYP_CP14_SAVE_AND_RESTORE_VCPU_THREADS)
    /* This is guarded by an #ifNdef (negation) ARM_CP14_SAVE_AND_RESTORE_NATIVE_THREADS
     * because if it wasn't, we'd be calling restore_user_debug_context twice
     * on a debug-API build; recall that restore_user_debug_context is called
     * in restore_user_context.
     *
     * We call restore_user_debug_context here, because vcpu_restore calls this
     * function (vcpu_enable). It's better to embed the
     * restore_user_debug_context call in here than to call it in the outer
     * level caller (vcpu_switch), because if the structure of this VCPU code
     * changes later on, it will be less likely that the person who changes
     * the code will be able to omit the debug register context restore, if
     * it's done here.
     */
    restore_user_debug_context(vcpu->vcpuTCB);
#endif
#if defined(ARM_HYP_TRAP_CP14_IN_NATIVE_USER_THREADS)
    /* Disable debug exception trapping and let the PL1 Guest VM handle all
     * of its own debug faults.
     */
    setHDCRTrapDebugExceptionState(false);
#endif
#ifdef CONFIG_HAVE_FPU
    /* We need to restore the FPEXC value early for the following reason:
     *
     * 1: When an application inside a VM is trying to execute an FPU
     * instruction and the EN bit of FPEXC is disabled, an undefined
     * instruction exception is sent to the guest Linux kernel instead of
     * the seL4. Until the Linux kernel examines the EN bit of the FPEXC
     * to determine if the exception FPU related, a VCPU trap is sent to
     * the seL4 kernel. However, it can be too late to restore the value
     * of saved FPEXC in the VCPU trap handler: if the EN bit of the saved
     * FPEXC is enabled, the Linux kernel thinks the FPU is enabled and
     * thus refuses to handle the exception. The result is the application
     * is killed with the cause of illegal instruction.
     *
     * Note that we restore the FPEXC here, but the current FPU owner
     * can be a different thread. Thus, it seems that we are modifying
     * another thread's FPEXC. However, the modification is OK.
     *
     * 1: If the other thread is a native thread, even if the EN bit of
     * the FPEXC is enabled, a trap th HYP mode will be triggered when
     * the thread tries to use the FPU.
     *
     * 2: If the other thread has a VCPU, the FPEXC is already saved
     * in the VCPU's vcpu->fpexc when the VCPU is saved or disabled.
     *
     * We also overwrite the fpuState.fpexc with the value saved in
     * vcpu->fpexc. Since the following scenario can happen:
     *
     * VM0 (the FPU owner) -> VM1 (update the FPEXC in vcpu_enable) ->
     * switchLocalFpuOwner (save VM0 with modified FPEXC) ->
     * VM1 (the new FPU owner)
     *
     * In the case above, the fpuState.fpexc of VM0 saves the value written
     * by the VM1, but the vcpu->fpexc of VM0 still contains the correct
     * value when VM0 is disabed (vcpu_disable) or saved (vcpu_save).
     *
     *
     */

    vcpu->vcpuTCB->tcbArch.tcbContext.fpuState.fpexc = vcpu_read_reg(vcpu, seL4_VCPUReg_FPEXC);
    access_fpexc(vcpu, true);
#endif
}

static void
vcpu_disable(vcpu_t *vcpu)
{
    uint32_t hcr;
    dsb();
    if (likely(vcpu)) {
        hcr = get_gic_vcpu_ctrl_hcr();
        vcpu->vgic.hcr = hcr;
        vcpu_save_reg(vcpu, seL4_VCPUReg_SCTLR);
        isb();
#ifdef CONFIG_HAVE_FPU
        if (nativeThreadUsingFPU(vcpu->vcpuTCB)) {
            access_fpexc(vcpu, false);
        }
#endif
    }
    /* Turn off the VGIC */
    set_gic_vcpu_ctrl_hcr(0);
    isb();

    /* Stage 1 MMU off */
    setSCTLR(SCTLR_DEFAULT);
    setHCR(HCR_NATIVE);

#if defined(ARM_HYP_CP14_SAVE_AND_RESTORE_VCPU_THREADS)
    /* Disable all breakpoint registers from triggering their
     * respective events, so that when we switch from a guest VM
     * to a native thread, the native thread won't trigger events
     * that were caused by things the guest VM did.
     */
    loadAllDisabledBreakpointState();
#endif
#if defined(ARM_HYP_TRAP_CP14_IN_NATIVE_USER_THREADS)
    /* Enable debug exception trapping and let seL4 trap all PL0 (user) native
     * seL4 threads' debug exceptions, so it can deliver them as fault messages.
     */
    setHDCRTrapDebugExceptionState(true);
#endif
    isb();
}


BOOT_CODE void
vcpu_boot_init(void)
{
    gic_vcpu_num_list_regs = VGIC_VTR_NLISTREGS(get_gic_vcpu_ctrl_vtr());
    if (gic_vcpu_num_list_regs > GIC_VCPU_MAX_NUM_LR) {
        printf("Warning: VGIC is reporting more list registers than we support. Truncating\n");
        gic_vcpu_num_list_regs = GIC_VCPU_MAX_NUM_LR;
    }
    vcpu_disable(NULL);
    armHSCurVCPU = NULL;
    armHSVCPUActive = false;

#if defined(ARM_HYP_TRAP_CP14_IN_VCPU_THREADS) || defined(ARM_HYP_TRAP_CP14_IN_NATIVE_USER_THREADS)
    /* On the verified build, we have implemented a workaround that ensures
     * that we don't need to save and restore the debug coprocessor's state
     * (and therefore don't have to expose the CP14 registers to verification).
     *
     * This workaround is simple: we just trap and intercept all Guest VM
     * accesses to the debug coprocessor, and deliver them as VMFault
     * messages to the VM Monitor. To that end, the VM Monitor can then
     * choose to either kill the Guest VM, or it can also choose to silently
     * step over the Guest VM's accesses to the debug coprocessor, thereby
     * silently eliminating the communication channel between the Guest VMs
     * (because the debug coprocessor acted as a communication channel
     * unless we saved/restored its state between VM switches).
     *
     * This workaround delegates the communication channel responsibility
     * from the kernel to the VM Monitor, essentially.
     */
    initHDCR();
#endif
}

#define HSR_FPU_FAULT   (0x1fe0000a)
#define HSR_TASE_FAULT  (0x1fe00020)

void
handleVCPUFault(word_t hsr)
{
#ifdef CONFIG_HAVE_FPU
    if (hsr == HSR_FPU_FAULT || hsr == HSR_TASE_FAULT) {
        assert(!isFpuEnable());
        handleFPUFault();
        setNextPC(NODE_STATE(ksCurThread), getRestartPC(NODE_STATE(ksCurThread)));
        return;
    }
#endif
    current_fault = seL4_Fault_VCPUFault_new(hsr);
    handleFault(ksCurThread);
    schedule();
    activateThread();
}

#ifdef CONFIG_HAVE_FPU
static inline void
access_fpexc(vcpu_t *vcpu, bool_t write)
{
    /* save a copy of the current status since
     * the enableFpuHyp modifies the armHSFPUEnabled
     */
    bool_t flag = armHSFPUEnabled;
    if (!flag) {
        enableFpuInstInHyp();
    }
    if (write) {
        MCR(FPEXC, vcpu_read_reg(vcpu, seL4_VCPUReg_FPEXC));
    } else {
        word_t fpexc;
        MRC(FPEXC, fpexc);
        vcpu_write_reg(vcpu, seL4_VCPUReg_FPEXC, fpexc);
    }
    /* restore the status */
    if (!flag) {
        trapFpuInstToHyp();
    }
}
#endif

#endif  /* CONFIG_ARCH_AARCH32 */

void
vcpu_save_reg(vcpu_t *vcpu, word_t reg)
{
    if (reg >= seL4_VCPUReg_Num || vcpu == NULL) return;
    vcpu->regs[reg] = hw_read_reg(reg);
}

void
vcpu_save_reg_range(vcpu_t *vcpu, word_t start, word_t end)
{
    if (start >= seL4_VCPUReg_Num || vcpu == NULL) return;
    for (word_t i = start; i <= end; i++) {
        vcpu_save_reg(vcpu, i);
    }
}

void
vcpu_restore_reg(vcpu_t *vcpu, word_t reg)
{
    if (reg >= seL4_VCPUReg_Num || vcpu == NULL) return;
    hw_write_reg(reg, vcpu->regs[reg]);
}

void
vcpu_restore_reg_range(vcpu_t *vcpu, word_t start, word_t end)
{
    if (start >= seL4_VCPUReg_Num || vcpu == NULL) return;
    for (word_t i = start; i <= end; i++) {
        vcpu_restore_reg(vcpu, i);
    }
}

word_t
vcpu_read_reg(vcpu_t *vcpu, word_t reg)
{
    if (reg >= seL4_VCPUReg_Num || vcpu == NULL) return 0;
    return vcpu->regs[reg];
}

void
vcpu_write_reg(vcpu_t *vcpu, word_t reg, word_t value)
{
    if (reg >= seL4_VCPUReg_Num || vcpu == NULL) return;
    vcpu->regs[reg] = value;
}

static void
vcpu_save(vcpu_t *vcpu, bool_t active)
{
    word_t i;
    unsigned int lr_num;

    assert(vcpu);
    dsb();
    /* If we aren't active then this state already got stored when
     * we were disabled */
    if (active) {
        vcpu_save_reg(vcpu, seL4_VCPUReg_SCTLR);
        vcpu->vgic.hcr = get_gic_vcpu_ctrl_hcr();
    }
    /* Store GIC VCPU control state */
    vcpu->vgic.vmcr = get_gic_vcpu_ctrl_vmcr();
    vcpu->vgic.apr = get_gic_vcpu_ctrl_apr();
    lr_num = gic_vcpu_num_list_regs;
    for (i = 0; i < lr_num; i++) {
        vcpu->vgic.lr[i] = get_gic_vcpu_ctrl_lr(i);
    }

    /* save registers */
#ifdef CONFIG_ARCH_AARCH64
    vcpu_save_reg_range(vcpu, seL4_VCPUReg_TTBR0, seL4_VCPUReg_SPSR_EL1);
#else
    vcpu_save_reg_range(vcpu, seL4_VCPUReg_ACTLR, seL4_VCPUReg_SPSRfiq);
#endif

#ifdef ARM_HYP_CP14_SAVE_AND_RESTORE_VCPU_THREADS
    /* This is done when we are asked to save and restore the CP14 debug context
     * of VCPU threads; the register context is saved into the underlying TCB.
     */
    saveAllBreakpointState(vcpu->vcpuTCB);
#endif
    isb();
#ifdef CONFIG_HAVE_FPU
    /* Other FPU registers are still lazily saved and restored when
     * handleFPUFault is called. See the comments in vcpu_enable
     * for more information.
     */
#ifdef CONFIG_ARCH_AARCH64
#else
    if (active && nativeThreadUsingFPU(vcpu->vcpuTCB)) {
        access_fpexc(vcpu, false);
    }
#endif
#endif
}


static uint32_t
readVCPUReg(vcpu_t *vcpu, uint32_t field)
{
    if (likely(armHSCurVCPU == vcpu)) {
        switch (field) {
        case seL4_VCPUReg_SCTLR:
            /* The SCTLR value is switched to/from hardware when we enable/disable
             * the vcpu, not when we switch vcpus */
            if (armHSVCPUActive) {
                return getSCTLR();
            } else {
                return vcpu->regs[seL4_VCPUReg_SCTLR];
            }
        default:
            return hw_read_reg(field);
        }
    } else {
        return vcpu_read_reg(vcpu, field);
    }
}

static void
writeVCPUReg(vcpu_t *vcpu, uint32_t field, uint32_t value)
{
    if (likely(armHSCurVCPU == vcpu)) {
        switch (field) {
        case seL4_VCPUReg_SCTLR:
            if (armHSVCPUActive) {
                setSCTLR(value);
            } else {
                hw_write_reg(field, value);
            }
            break;
        default:
            hw_write_reg(field, value);
        }
    } else {
       vcpu_write_reg(vcpu, field, value);
    }
}

void
vcpu_restore(vcpu_t *vcpu)
{
    assert(vcpu);
    word_t i;
    unsigned int lr_num;
    /* Turn off the VGIC */
    set_gic_vcpu_ctrl_hcr(0);
    isb();

    /* Restore GIC VCPU control state */
    set_gic_vcpu_ctrl_vmcr(vcpu->vgic.vmcr);
    set_gic_vcpu_ctrl_apr(vcpu->vgic.apr);
    lr_num = gic_vcpu_num_list_regs;
    for (i = 0; i < lr_num; i++) {
        set_gic_vcpu_ctrl_lr(i, vcpu->vgic.lr[i]);
    }

    /* restore registers */
#ifdef CONFIG_ARCH_AARCH64
    vcpu_restore_reg_range(vcpu, seL4_VCPUReg_TTBR0, seL4_VCPUReg_SPSR_EL1);
#else
    vcpu_restore_reg_range(vcpu, seL4_VCPUReg_ACTLR, seL4_VCPUReg_SPSRfiq);
#endif
    vcpu_enable(vcpu);
}

void
VGICMaintenance(void)
{
    uint32_t eisr0, eisr1;
    uint32_t flags;

    /* The current thread must be runnable at this point as we can only get
     * a VGIC maintenance whilst we are actively running a thread with an
     * associated VCPU. For the moment for the proof we leave a redundant
     * check in here that this is indeed not happening */
    if (!isRunnable(ksCurThread)) {
        printf("Received VGIC maintenance on non-runnable thread!\n");
        return;
    }

    eisr0 = get_gic_vcpu_ctrl_eisr0();
    eisr1 = get_gic_vcpu_ctrl_eisr1();
    flags = get_gic_vcpu_ctrl_misr();

    if (flags & VGIC_MISR_EOI) {
        int irq_idx;
        if (eisr0) {
            irq_idx = ctzl(eisr0);
        } else if (eisr1) {
            irq_idx = ctzl(eisr1) + 32;
        } else {
            irq_idx = -1;
        }

        /* the hardware should never give us an invalid index, but we don't
         * want to trust it that far */
        if (irq_idx == -1  || irq_idx >= gic_vcpu_num_list_regs) {
            current_fault = seL4_Fault_VGICMaintenance_new(0, 0);
        } else {
            virq_t virq = get_gic_vcpu_ctrl_lr(irq_idx);
            switch (virq_get_virqType(virq)) {
            case virq_virq_active:
                virq = virq_virq_active_set_virqEOIIRQEN(virq, 0);
                break;
            case virq_virq_pending:
                virq = virq_virq_pending_set_virqEOIIRQEN(virq, 0);
                break;
            case virq_virq_invalid:
                virq = virq_virq_invalid_set_virqEOIIRQEN(virq, 0);
                break;
            }
            set_gic_vcpu_ctrl_lr(irq_idx, virq);
            /* decodeVCPUInjectIRQ below checks the vgic.lr register,
             * so we should also sync the shadow data structure as well */
            armHSCurVCPU->vgic.lr[irq_idx] = virq;
            current_fault = seL4_Fault_VGICMaintenance_new(irq_idx, 1);
        }

    } else {
        /* Assume that it was an EOI for a LR that was not present */
        current_fault = seL4_Fault_VGICMaintenance_new(0, 0);
    }

    handleFault(ksCurThread);
}

void
vcpu_init(vcpu_t *vcpu)
{
#ifdef CONFIG_ARCH_AARCH64
    vcpu_write_reg(vcpu, seL4_VCPUReg_SCTLR, SCTLR_EL1_VM);
#else
    vcpu_write_reg(vcpu, seL4_VCPUReg_SCTLR, SCTLR_DEFAULT);
    vcpu_write_reg(vcpu, seL4_VCPUReg_ACTLR, ACTLR_DEFAULT);
#endif
    /* GICH VCPU interface control */
    vcpu->vgic.hcr = VGIC_HCR_EN;
}

void
vcpu_switch(vcpu_t *new)
{
    if (likely(armHSCurVCPU != new)) {
        if (unlikely(new != NULL)) {
            if (unlikely(armHSCurVCPU != NULL)) {
                vcpu_save(armHSCurVCPU, armHSVCPUActive);
            }
            vcpu_restore(new);
            armHSCurVCPU = new;
            armHSVCPUActive = true;
        } else if (unlikely(armHSVCPUActive)) {
            /* leave the current VCPU state loaded, but disable vgic and mmu */
#ifdef ARM_HYP_CP14_SAVE_AND_RESTORE_VCPU_THREADS
            saveAllBreakpointState(armHSCurVCPU->vcpuTCB);
#endif
            vcpu_disable(armHSCurVCPU);
            armHSVCPUActive = false;
        }
    } else if (likely(!armHSVCPUActive && new != NULL)) {
        isb();
        vcpu_enable(new);
        armHSVCPUActive = true;
    }
}


static void
vcpu_invalidate_active(void)
{
    if (armHSVCPUActive) {
        vcpu_disable(NULL);
        armHSVCPUActive = false;
    }
    armHSCurVCPU = NULL;
}

void
vcpu_finalise(vcpu_t *vcpu)
{
    if (vcpu->vcpuTCB) {
        dissociateVCPUTCB(vcpu, vcpu->vcpuTCB);
    }
}

void
associateVCPUTCB(vcpu_t *vcpu, tcb_t *tcb)
{
    if (tcb->tcbArch.tcbVCPU) {
        dissociateVCPUTCB(tcb->tcbArch.tcbVCPU, tcb);
    }
    if (vcpu->vcpuTCB) {
        dissociateVCPUTCB(vcpu, vcpu->vcpuTCB);
    }
    tcb->tcbArch.tcbVCPU = vcpu;
    vcpu->vcpuTCB = tcb;
}

void
dissociateVCPUTCB(vcpu_t *vcpu, tcb_t *tcb)
{
    if (tcb->tcbArch.tcbVCPU != vcpu || vcpu->vcpuTCB != tcb) {
        fail("TCB and VCPU not associated.");
    }
    if (vcpu == armHSCurVCPU) {
        vcpu_invalidate_active();
    }
    tcb->tcbArch.tcbVCPU = NULL;
    vcpu->vcpuTCB = NULL;
#ifdef ARM_HYP_CP14_SAVE_AND_RESTORE_VCPU_THREADS
    Arch_debugDissociateVCPUTCB(tcb);
#endif
    /* sanitize the CPSR as without a VCPU a thread should only be in user mode */
#ifdef CONFIG_ARCH_AARCH64
#else
    setRegister(tcb, CPSR, sanitiseRegister(CPSR, getRegister(tcb, CPSR), false));
#endif

}


exception_t
invokeVCPUWriteReg(vcpu_t *vcpu, uint32_t field, uint32_t value)
{
    writeVCPUReg(vcpu, field, value);
    return EXCEPTION_NONE;
}

exception_t
decodeVCPUWriteReg(cap_t cap, unsigned int length, word_t* buffer)
{
    uint32_t field;
    uint32_t value;
    if (length < 2) {
        userError("VCPUWriteReg: Truncated message.");
        current_syscall_error.type = seL4_TruncatedMessage;
        return EXCEPTION_SYSCALL_ERROR;
    }
    field = getSyscallArg(0, buffer);
    value = getSyscallArg(1, buffer);
    if (field >= seL4_VCPUReg_Num) {
        userError("VCPUWriteReg: Invalid field 0x%lx.", (long)field);
        current_syscall_error.type = seL4_InvalidArgument;
        current_syscall_error.invalidArgumentNumber = 1;
        return EXCEPTION_SYSCALL_ERROR;
    }
    setThreadState(ksCurThread, ThreadState_Restart);
    return invokeVCPUWriteReg(VCPU_PTR(cap_vcpu_cap_get_capVCPUPtr(cap)), field, value);
}

exception_t
invokeVCPUReadReg(vcpu_t *vcpu, uint32_t field, bool_t call)
{
    tcb_t *thread;
    thread = ksCurThread;
    uint32_t value = readVCPUReg(vcpu, field);
    if (call) {
        word_t *ipcBuffer = lookupIPCBuffer(true, thread);
        setRegister(thread, badgeRegister, 0);
        unsigned int length = setMR(thread, ipcBuffer, 0, value);
        setRegister(thread, msgInfoRegister, wordFromMessageInfo(
                        seL4_MessageInfo_new(0, 0, 0, length)));
    }
    setThreadState(ksCurThread, ThreadState_Running);
    return EXCEPTION_NONE;
}

exception_t
decodeVCPUReadReg(cap_t cap, unsigned int length, bool_t call, word_t* buffer)
{
    uint32_t field;
    if (length < 1) {
        userError("VCPUReadReg: Truncated message.");
        current_syscall_error.type = seL4_TruncatedMessage;
        return EXCEPTION_SYSCALL_ERROR;
    }

    field = getSyscallArg(0, buffer);

    if (field >= seL4_VCPUReg_Num) {
        userError("VCPUReadReg: Invalid field 0x%lx.", (long)field);
        current_syscall_error.type = seL4_InvalidArgument;
        current_syscall_error.invalidArgumentNumber = 1;
        return EXCEPTION_SYSCALL_ERROR;
    }

    setThreadState(ksCurThread, ThreadState_Restart);
    return invokeVCPUReadReg(VCPU_PTR(cap_vcpu_cap_get_capVCPUPtr(cap)), field, call);
}

exception_t
invokeVCPUInjectIRQ(vcpu_t* vcpu, unsigned long index, virq_t virq)
{
    if (likely(armHSCurVCPU == vcpu)) {
        set_gic_vcpu_ctrl_lr(index, virq);
    } else {
        vcpu->vgic.lr[index] = virq;
    }

    return EXCEPTION_NONE;
}

exception_t
decodeVCPUInjectIRQ(cap_t cap, unsigned int length, word_t* buffer)
{
    word_t vid, priority, group, index;
    vcpu_t *vcpu;

#ifdef CONFIG_ARCH_AARCH64
    word_t mr0;
    if (length < 1) {
        current_syscall_error.type = seL4_TruncatedMessage;
        return EXCEPTION_SYSCALL_ERROR;
    }

    mr0 = getSyscallArg(0, buffer);
    vid = mr0 & 0xffff;
    priority = (mr0 >> 16) & 0xff;
    group = (mr0 >> 24) & 0xff;
    index = (mr0 >> 32) & 0xff;
#else
    word_t mr0, mr1;
    if (length < 2) {
        current_syscall_error.type = seL4_TruncatedMessage;
        return EXCEPTION_SYSCALL_ERROR;
    }

    mr0 = getSyscallArg(0, buffer);
    mr1 = getSyscallArg(1, buffer);
    vid = mr0 & 0xffff;
    priority = (mr0 >> 16) & 0xff;
    group = (mr0 >> 24) & 0xff;
    index = mr1 & 0xff;
#endif

    vcpu = VCPU_PTR(cap_vcpu_cap_get_capVCPUPtr(cap));
    /* Check IRQ parameters */
    if (vid > (1U << 10) - 1) {
        current_syscall_error.type = seL4_RangeError;
        current_syscall_error.rangeErrorMin = 0;
        current_syscall_error.rangeErrorMax = (1U << 10) - 1;
        current_syscall_error.invalidArgumentNumber = 1;
        current_syscall_error.type = seL4_RangeError;
        return EXCEPTION_SYSCALL_ERROR;
    }
    if (priority > 31) {
        current_syscall_error.type = seL4_RangeError;
        current_syscall_error.rangeErrorMin = 0;
        current_syscall_error.rangeErrorMax = 31;
        current_syscall_error.invalidArgumentNumber = 2;
        current_syscall_error.type = seL4_RangeError;
        return EXCEPTION_SYSCALL_ERROR;
    }
    if (group > 1) {
        current_syscall_error.type = seL4_RangeError;
        current_syscall_error.rangeErrorMin = 0;
        current_syscall_error.rangeErrorMax = 1;
        current_syscall_error.invalidArgumentNumber = 3;
        current_syscall_error.type = seL4_RangeError;
        return EXCEPTION_SYSCALL_ERROR;
    }
    /* LR index out of range */
    if (index >= gic_vcpu_num_list_regs) {
        current_syscall_error.type = seL4_RangeError;
        current_syscall_error.rangeErrorMin = 0;
        current_syscall_error.rangeErrorMax = gic_vcpu_num_list_regs - 1;
        current_syscall_error.invalidArgumentNumber = 4;
        current_syscall_error.type = seL4_RangeError;
        return EXCEPTION_SYSCALL_ERROR;
    }
    /* LR index is in use */
    if (virq_get_virqType(vcpu->vgic.lr[index]) == virq_virq_active) {
        userError("VGIC List register in use.");
        current_syscall_error.type = seL4_DeleteFirst;
        return EXCEPTION_SYSCALL_ERROR;
    }
    virq_t virq = virq_virq_pending_new(group, priority, 1, vid);

    setThreadState(ksCurThread, ThreadState_Restart);
    return invokeVCPUInjectIRQ(vcpu, index, virq);
}

exception_t
decodeVCPUSetTCB(cap_t cap, extra_caps_t extraCaps)
{
    cap_t tcbCap;
    if ( extraCaps.excaprefs[0] == NULL) {
        userError("VCPU SetTCB: Truncated message.");
        current_syscall_error.type = seL4_TruncatedMessage;
        return EXCEPTION_SYSCALL_ERROR;
    }
    tcbCap  = extraCaps.excaprefs[0]->cap;

    if (cap_get_capType(tcbCap) != cap_thread_cap) {
        userError("TCB cap is not a TCB cap.");
        current_syscall_error.type = seL4_IllegalOperation;
        return EXCEPTION_SYSCALL_ERROR;
    }

    setThreadState(ksCurThread, ThreadState_Restart);
    return invokeVCPUSetTCB(VCPU_PTR(cap_vcpu_cap_get_capVCPUPtr(cap)), TCB_PTR(cap_thread_cap_get_capTCBPtr(tcbCap)));
}

exception_t
invokeVCPUSetTCB(vcpu_t *vcpu, tcb_t *tcb)
{
    associateVCPUTCB(vcpu, tcb);

    return EXCEPTION_NONE;
}

exception_t decodeARMVCPUInvocation(
    word_t label,
    unsigned int length,
    cptr_t cptr,
    cte_t* slot,
    cap_t cap,
    extra_caps_t extraCaps,
    bool_t call,
    word_t* buffer
)
{
    switch (label) {
    case ARMVCPUSetTCB:
        return decodeVCPUSetTCB(cap, extraCaps);
    case ARMVCPUReadReg:
        return decodeVCPUReadReg(cap, length, call, buffer);
    case ARMVCPUWriteReg:
        return decodeVCPUWriteReg(cap, length, buffer);
    case ARMVCPUInjectIRQ:
        return decodeVCPUInjectIRQ(cap, length, buffer);
    default:
        userError("VCPU: Illegal operation.");
        current_syscall_error.type = seL4_IllegalOperation;
        return EXCEPTION_SYSCALL_ERROR;
    }
}

#endif /* end of CONFIG_ARM_HYPERVISOR_SUPPORT */
