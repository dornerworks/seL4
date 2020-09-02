#
# Copyright 2020, DornerWorks
#
# This software may be distributed and modified according to the terms of
# the GNU General Public License version 2. Note that NO WARRANTY is provided.
# See "LICENSE_GPLv2.txt" for details.
#
# @TAG(DORNERWORKS_GPL)
#

cmake_minimum_required(VERSION 3.7.2)

declare_platform(polarfire KernelPlatformPolarfire PLAT_POLARFIRE KernelSel4ArchRiscV64)

if(KernelPlatformPolarfire)
    declare_seL4_arch(riscv64)
    config_set(KernelRiscVPlatform RISCV_PLAT "polarfire")
    config_set(KernelPlatformFirstHartID FIRST_HART_ID 1)
    list(APPEND KernelDTSList "tools/dts/mpfs_icicle.dts")
    list(APPEND KernelDTSList "src/plat/polarfire/overlay-polarfire.dts")
    declare_default_headers(
        TIMER_FREQUENCY 10000000llu
        PLIC_MAX_NUM_INT 93
        INTERRUPT_CONTROLLER drivers/irq/riscv_plic0.h
    )
else()
    unset(KernelPlatformFirstHartID CACHE)
endif()
