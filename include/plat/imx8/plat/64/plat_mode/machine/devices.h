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

#ifndef __PLAT_MODE_MACHINE_DEVICES_H
#define __PLAT_MODE_MACHINE_DEVICES_H

#ifndef CONFIG_ARM_HYPERVISOR_SUPPORT
#define UART_PPTR                     0xffffffffffff0000
#define GIC_DISTRIBUTOR_PPTR          0xfffffffffff00000
#define GIC_REDIST_PPTR               0xfffffffffff10000
#else
#define UART_PPTR                     0xffffffff0000
#define GIC_DISTRIBUTOR_PPTR          0xfffffff00000
#define GIC_REDIST_PPTR               0xfffffff10000
#endif

#endif /* __PLAT_MODE_MACHINE_DEVICES_H */
