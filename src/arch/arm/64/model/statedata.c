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
#include <util.h>
#include <api/types.h>
#include <arch/types.h>
#include <arch/model/statedata.h>
#include <arch/object/structures.h>
#include <linker.h>
#include <plat/machine/hardware.h>

asid_pool_t *armKSASIDTable[BIT(asidHighBits)];

pgde_t armKSGlobalUserPGD[BIT(PGD_INDEX_BITS)] ALIGN_BSS(BIT(seL4_PGDBits));
pgde_t armKSGlobalKernelPGD[BIT(PGD_INDEX_BITS)] ALIGN_BSS(BIT(seL4_PGDBits));

pude_t armKSGlobalKernelPUD[BIT(PUD_INDEX_BITS)] ALIGN_BSS(BIT(seL4_PUDBits));
pde_t armKSGlobalKernelPDs[BIT(PUD_INDEX_BITS)][BIT(PD_INDEX_BITS)] ALIGN_BSS(BIT(seL4_PageDirBits));
pte_t armKSGlobalKernelPT[BIT(PD_INDEX_BITS)][BIT(PT_INDEX_BITS)] ALIGN_BSS(BIT(seL4_PageTableBits));

#ifdef CONFIG_ARM_HYPERVISOR_SUPPORT

/* Current VCPU */
vcpu_t *armHSCurVCPU;
/* Whether the current loaded VCPU is enabled in the hardware or not */
bool_t armHSVCPUActive;

#endif
