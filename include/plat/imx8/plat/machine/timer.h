/*
 * Copyright 2017, DornerWorks
 * Copyright 2017, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(DATA61_DORNERWORKS_BSD)
 */

#pragma once

/*
   This is set by uboot, specifically in
   include/configs/imx8qm_arm2.h
   with the COUNTER_FREQUENCY define
   It can also be found by reading the cntfrq_el0 system register
*/
#define TIMER_CLOCK_HZ        8000000llu
#include <arch/machine/generic_timer.h>
