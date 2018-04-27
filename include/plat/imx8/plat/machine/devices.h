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

#ifndef __PLAT_MACHINE_DEVICES_H
#define __PLAT_MACHINE_DEVICES_H

#include <plat_mode/machine/devices.h>

#define ARM_PLAT_NUM_CB            32

// Virtual Addresses _________________________________________________
#define GIC_500_DISTRIBUTOR_PPTR   GIC_DISTRIBUTOR_PPTR
#define GIC_500_REDIST_PPTR        GIC_REDIST_PPTR
#define L2CC_L2C310_PPTR           (L2CC_PL310_PPTR      )
#define ARM_MP_PRIV_TIMER_PPTR     (ARM_MP_PPTR1 + 0x600 )
#define ARM_MP_GLOBAL_TIMER_PPTR   (ARM_MP_PPTR1 + 0x200 )

// Device Physical Addresses _________________________________________
#define UART_PADDR                  DMA_LPUART0_PADDR
#define Dblog_GIC_DISTRIBUTOR_PADDR Dblog_GIC_PADDR
#define Dblog_GIC_REDIST_PADDR      (Dblog_GIC_PADDR + 0x100000)

// All Device Physical Addresses _____________________________________
#define FLEX_SPI1_PADDR     0x400000000
// #define RESERVED_PADDR       0x100000000
#define DDR_PADDR           0x80000000
#define PCIe1_PADDR         0x70000000
#define PCIe0_PADDR         0x60000000
#define HSIO_PADDR          0x5F000000
// #define RESERVED_PADDR       0x5E000000
#define LSIO_PADDR          0x5D000000
#define Db_DRAM_SLAVE_PADDR 0x5C000000
#define CONN_PADDR          0x5B000000
#define DMA_PADDR           0x5A000000
#define AUDIO_PADDR         0x59000000
#define IMAGING_PADDR       0x58000000
#define DC1_PADDR           0x57000000
#define DC0_PADDR           0x56000000
#define VPU_PERIPH_PADDR    0x55000000
#define GPU1_PADDR          0x54000000
#define GPU0_PADDR          0x53000000
#define CCI_PADDR           0x52000000
#define SMMU_PADDR          0x51400000
#define SMMU_SIZE           0x00040000
#define DBLOG_PADDR         0x51000000
// #define RESERVED_PADDR       0x42000000
// #define RESERVED_PADDR       0x40000000
// #define RESERVED_PADDR       0x3C000000
#define CM4_1_PADDR         0x38000000
#define CM4_0_PADDR         0x34000000
#define SCU_PADDR           0x30000000
#define VPU_PADDR           0x2C000000
// #define RESERVED_PADDR       0x28000000
// #define RESERVED_PADDR       0x20000000
// #define RESERVED_PADDR       0x1C000000
#define FLEX_SPI0_PADDR     0x08000000
// #define RESERVED_PADDR       0x02000000
// #define RESERVED_PADDR       0x00140000
#define SYSTEM_RAM_PADDR    0x00100000
// #define RESERVED_PADDR       0x00018000
#define BOOT_ROM_PADDR      0x00000000

/* Device: HSIO (29) ________________________________________________________________________________ */
#define HSIO_PCIeb_PADDR                                        0x70000000      /* 256MB */
#define HSIO_PCIea_PADDR                                        0x60000000      /* 256MB */
// #define RESERVED_PADDR  0x5F1B0000
#define HSIO_PHYx1_PADDR                                        0x5F1A0000      /* 64KB */
#define HSIO_PHYx2lane1_PADDR                                   0x5F190000      /* 64KB */
#define HSIO_PHYx2lane0_PADDR                                   0x5F180000      /* 64KB */
#define HSIO_GPIO_PADDR                                         0x5F170000      /* 64KB */
#define HSIO_SYS_CSRRegsMISC_PADDR                              0x5F160000      /* 64KB */
#define HSIO_SYS_CSRRegsSATA0_PADDR                             0x5F150000      /* 64KB */
#define HSIO_SYS_CSRRegsPCIeb_PADDR                             0x5F140000      /* 64KB */
#define HSIO_SYS_CSRRegsPCIea_PADDR                             0x5F130000      /* 64KB */
#define HSIO_SYS_CSRRegsPHYx1_PADDR                             0x5F120000      /* 64KB */
#define HSIO_SYS_CSRRegsPHYx2_PADDR                             0x5F110000      /* 64KB */
#define HSIO_LPCGRegsGPIO_PADDR                                 0x5F100000      /* 64KB */
#define HSIO_LPCGRegsCRR_5_PADDR                                0x5F0F0000      /* 64KB */
#define HSIO_LPCGRegsCRR_4_PADDR                                0x5F0E0000      /* 64KB */
#define HSIO_LPCGRegsCRR_3_PADDR                                0x5F0D0000      /* 64KB */
#define HSIO_LPCGRegsCRR_2_PADDR                                0x5F0C0000      /* 64KB */
#define HSIO_LPCGRegsCRR_1_PADDR                                0x5F0B0000      /* 64KB */
#define HSIO_LPCGRegsCRR_0_PADDR                                0x5F0A0000      /* 64KB */
#define HSIO_LPCGRegsPHYx1_PADDR                                0x5F090000      /* 64KB */
#define HSIO_LPCGRegsPHYx2_PADDR                                0x5F080000      /* 64KB */
#define HSIO_LPCGRegsSATA0_PADDR                                0x5F070000      /* 64KB */
#define HSIO_LPCGRegsPCIeb_PADDR                                0x5F060000      /* 64KB */
#define HSIO_LPCGRegsPCIea_PADDR                                0x5F050000      /* 64KB */
// #define RESERVED_PADDR  0x5F030000
#define HSIO_SATA0SATAAHCI_PADDR                                0x5F020000      /* 64KB */
#define HSIO_PCIebPCIeConfig_PADDR                              0x5F010000      /* 64KB */
#define HSIO_PCIeaPCIeConfig_PADDR                              0x5F000000      /* 64KB */
/* Device: LSIO (109) _______________________________________________________________________________ */
// #define RESERVED_PADDR  0x5E800000
// #define RESERVED_PADDR  0x5E720000
#define LSIO_LPCG_MU13_BClkCtrl_PADDR                           0x5E710000      /* 64KB */
#define LSIO_LPCG_MU12_BClkCtrl_PADDR                           0x5E700000      /* 64KB */
#define LSIO_LPCG_MU11_BClkCtrl_PADDR                           0x5E6F0000      /* 64KB */
#define LSIO_LPCG_MU10_BClkCtrl_PADDR                           0x5E6E0000      /* 64KB */
#define LSIO_LPCG_MU9_BClkCtrl_PADDR                            0x5E6D0000      /* 64KB */
#define LSIO_LPCG_MU8_BClkCtrl_PADDR                            0x5E6C0000      /* 64KB */
#define LSIO_LPCG_MU7_BClkCtrl_PADDR                            0x5E6B0000      /* 64KB */
#define LSIO_LPCG_MU6_BClkCtrl_PADDR                            0x5E6A0000      /* 64KB */
#define LSIO_LPCG_MU5_BClkCtrl_PADDR                            0x5E690000      /* 64KB */
#define LSIO_LPCG_MU13_AClkCtrl_PADDR                           0x5E680000      /* 64KB */
#define LSIO_LPCG_MU12_AClkCtrl_PADDR                           0x5E670000      /* 64KB */
#define LSIO_LPCG_MU11_AClkCtrl_PADDR                           0x5E660000      /* 64KB */
#define LSIO_LPCG_MU10_AClkCtrl_PADDR                           0x5E650000      /* 64KB */
#define LSIO_LPCG_MU9_AClkCtrl_PADDR                            0x5E640000      /* 64KB */
#define LSIO_LPCG_MU8_AClkCtrl_PADDR                            0x5E630000      /* 64KB */
#define LSIO_LPCG_MU7_AClkCtrl_PADDR                            0x5E620000      /* 64KB */
#define LSIO_LPCG_MU6_AClkCtrl_PADDR                            0x5E610000      /* 64KB */
#define LSIO_LPCG_MU5_AClkCtrl_PADDR                            0x5E600000      /* 64KB */
// #define RESERVED_PADDR  0x5E5B0000
#define LSIO_LPCG_KPPClkCtrl_PADDR                              0x5E5A0000      /* 64KB */
#define LSIO_LPCG_OCRAMClkCtrl_PADDR                            0x5E590000      /* 64KB */
#define LSIO_LPCG_GPT4ClkCtrl_PADDR                             0x5E580000      /* 64KB */
#define LSIO_LPCG_GPT3ClkCtrl_PADDR                             0x5E570000      /* 64KB */
#define LSIO_LPCG_GPT2ClkCtrl_PADDR                             0x5E560000      /* 64KB */
#define LSIO_LPCG_GPT1ClkCtrl_PADDR                             0x5E550000      /* 64KB */
#define LSIO_LPCG_GPT0ClkCtrl_PADDR                             0x5E540000      /* 64KB */
#define LSIO_LPCG_FlexSPI1ClkCtrl_PADDR                         0x5E530000      /* 64KB */
#define LSIO_LPCG_FlexSPI0ClkCtrl_PADDR                         0x5E520000      /* 64KB */
// #define RESERVED_PADDR  0x5E510000
#define LSIO_LPCG_ROMCPClkCtrl_PADDR                            0x5E500000      /* 64KB */
#define LSIO_LPCG_GPIO7ClkCtrl_PADDR                            0x5E4F0000      /* 64KB */
#define LSIO_LPCG_GPIO6ClkCtrl_PADDR                            0x5E4E0000      /* 64KB */
#define LSIO_LPCG_GPIO5ClkCtrl_PADDR                            0x5E4D0000      /* 64KB */
#define LSIO_LPCG_GPIO4ClkCtrl_PADDR                            0x5E4C0000      /* 64KB */
#define LSIO_LPCG_GPIO3ClkCtrl_PADDR                            0x5E4B0000      /* 64KB */
#define LSIO_LPCG_GPIO2ClkCtrl_PADDR                            0x5E4A0000      /* 64KB */
#define LSIO_LPCG_GPIO1ClkCtrl_PADDR                            0x5E490000      /* 64KB */
#define LSIO_LPCG_GPIO0ClkCtrl_PADDR                            0x5E480000      /* 64KB */
#define LSIO_LPCG_PWM7ClkCtrl_PADDR                             0x5E470000      /* 64KB */
#define LSIO_LPCG_PWM6ClkCtrl_PADDR                             0x5E460000      /* 64KB */
#define LSIO_LPCG_PWM5ClkCtrl_PADDR                             0x5E450000      /* 64KB */
#define LSIO_LPCG_PWM4ClkCtrl_PADDR                             0x5E440000      /* 64KB */
#define LSIO_LPCG_PWM3ClkCtrl_PADDR                             0x5E430000      /* 64KB */
#define LSIO_LPCG_PWM2ClkCtrl_PADDR                             0x5E420000      /* 64KB */
#define LSIO_LPCG_PWM1ClkCtrl_PADDR                             0x5E410000      /* 64KB */
#define LSIO_LPCG_PWM0ClkCtrl_PADDR                             0x5E400000      /* 64KB */
// #define RESERVED_PADDR  0x5E3C0000
#define LSIO_IEEtestdatabuffer_PADDR                            0x5E3B0000      /* 64KB */
#define LSIO_IEEregion7_PADDR                                   0x5E3A0000      /* 64KB */
#define LSIO_IEEregion6_PADDR                                   0x5E390000      /* 64KB */
#define LSIO_IEEregion5_PADDR                                   0x5E380000      /* 64KB */
#define LSIO_IEEregion4_PADDR                                   0x5E370000      /* 64KB */
#define LSIO_IEEregion3_PADDR                                   0x5E360000      /* 64KB */
#define LSIO_IEEregion2_PADDR                                   0x5E350000      /* 64KB */
#define LSIO_IEEregion1_PADDR                                   0x5E340000      /* 64KB */
#define LSIO_IEEregion0_PADDR                                   0x5E330000      /* 64KB */
#define LSIO_IEEbase_PADDR                                      0x5E320000      /* 64KB */
#define LSIO_MU13_B_PADDR                                       0x5E310000      /* 64KB */
#define LSIO_MU12_B_PADDR                                       0x5E300000      /* 64KB */
#define LSIO_MU11_B_PADDR                                       0x5E2F0000      /* 64KB */
#define LSIO_MU10_B_PADDR                                       0x5E2E0000      /* 64KB */
#define LSIO_MU9_B_PADDR                                        0x5E2D0000      /* 64KB */
#define LSIO_MU8_B_PADDR                                        0x5E2C0000      /* 64KB */
#define LSIO_MU7_B_PADDR                                        0x5E2B0000      /* 64KB */
#define LSIO_MU6_B_PADDR                                        0x5E2A0000      /* 64KB */
#define LSIO_MU5_B_PADDR                                        0x5E290000      /* 64KB */
#define LSIO_MU13_A_PADDR                                       0x5E280000      /* 64KB */
#define LSIO_MU12_A_PADDR                                       0x5E270000      /* 64KB */
#define LSIO_MU11_A_PADDR                                       0x5E260000      /* 64KB */
#define LSIO_MU10_A_PADDR                                       0x5E250000      /* 64KB */
#define LSIO_MU9_A_PADDR                                        0x5E240000      /* 64KB */
#define LSIO_MU8_A_PADDR                                        0x5E230000      /* 64KB */
#define LSIO_MU7_A_PADDR                                        0x5E220000      /* 64KB */
#define LSIO_MU6_A_PADDR                                        0x5E210000      /* 64KB */
#define LSIO_MU5_A_PADDR                                        0x5E200000      /* 64KB */
#define LSIO_MU4_A_PADDR                                        0x5E1F0000      /* 64KB */
#define LSIO_MU3_A_PADDR                                        0x5E1E0000      /* 64KB */
#define LSIO_MU2_A_PADDR                                        0x5E1D0000      /* 64KB */
#define LSIO_MU1_A_PADDR                                        0x5E1C0000      /* 64KB */
#define LSIO_MU0_A_PADDR                                        0x5E1B0000      /* 64KB */
#define LSIO_KPP_PADDR                                          0x5E1A0000      /* 64KB */
// #define RESERVED_PADDR  0x5E190000
#define LSIO_GPT4_PADDR                                         0x5E180000      /* 64KB */
#define LSIO_GPT3_PADDR                                         0x5E170000      /* 64KB */
#define LSIO_GPT2_PADDR                                         0x5E160000      /* 64KB */
#define LSIO_GPT1_PADDR                                         0x5E150000      /* 64KB */
#define LSIO_GPT0_PADDR                                         0x5E140000      /* 64KB */
#define LSIO_FlexSPI1_PADDR                                     0x5E130000      /* 64KB */
#define LSIO_FlexSPI0_PADDR                                     0x5E120000      /* 64KB */
// #define RESERVED_PADDR  0x5E110000
#define LSIO_ROMCP_PADDR                                        0x5E100000      /* 64KB */
#define LSIO_GPIO7_PADDR                                        0x5E0F0000      /* 64KB */
#define LSIO_GPIO6_PADDR                                        0x5E0E0000      /* 64KB */
#define LSIO_GPIO5_PADDR                                        0x5E0D0000      /* 64KB */
#define LSIO_GPIO4_PADDR                                        0x5E0C0000      /* 64KB */
#define LSIO_GPIO3_PADDR                                        0x5E0B0000      /* 64KB */
#define LSIO_GPIO2_PADDR                                        0x5E0A0000      /* 64KB */
#define LSIO_GPIO1_PADDR                                        0x5E090000      /* 64KB */
#define LSIO_GPIO0_PADDR                                        0x5E080000      /* 64KB */
#define LSIO_PWM7_PADDR                                         0x5E070000      /* 64KB */
#define LSIO_PWM6_PADDR                                         0x5E060000      /* 64KB */
#define LSIO_PWM5_PADDR                                         0x5E050000      /* 64KB */
#define LSIO_PWM4_PADDR                                         0x5E040000      /* 64KB */
#define LSIO_PWM3_PADDR                                         0x5E030000      /* 64KB */
#define LSIO_PWM2_PADDR                                         0x5E020000      /* 64KB */
#define LSIO_PWM1_PADDR                                         0x5E010000      /* 64KB */
#define LSIO_PWM0_PADDR                                         0x5E000000      /* 64KB */
#define MU3_BASE_PADDR                                          0x5D1E0000
#define MU2_BASE_PADDR                                          0x5D1D0000
#define MU1_BASE_PADDR                                          0x5D1C0000
#define MU_BASE_PADDR                                           0x5D1B0000
// GPT0 is the only one really listed in the Linux device tree
#define GPT4_PADDR                                              0x5D180000      /* 64KB */
#define GPT3_PADDR                                              0x5D170000      /* 64KB */
#define GPT2_PADDR                                              0x5D160000      /* 64KB */
#define GPT1_PADDR                                              0x5D150000      /* 64KB */
#define GPT0_PADDR                                              0x5D140000      /* 64KB */
/* Device: Connectivity (36) ________________________________________________________________________ */
// #define RESERVED_PADDR  0x5B820000
#define Connectivity_APBHDMA_PADDR                              0x5B810000      /* 64KB */
#define Connectivity_DTCP_PADDR                                 0x5B800000      /* 64KB */
// #define RESERVED_PADDR  0x5B2B0000
#define Connectivity_LPCGeDMA_PADDR                             0x5B2A0000      /* 64KB */
#define Connectivity_LPCGNAND_PADDR                             0x5B290000      /* 64KB */
#define Connectivity_LPCGUSB3_PADDR                             0x5B280000      /* 64KB */
#define Connectivity_LPCGUSB2_PADDR                             0x5B270000      /* 64KB */
#define Connectivity_LPCGMLB_PADDR                              0x5B260000      /* 64KB */
#define Connectivity_LPCGDTCP_PADDR                             0x5B250000      /* 64KB */
#define Connectivity_LPCGENET2_PADDR                            0x5B240000      /* 64KB */
#define Connectivity_LPCGENET1_PADDR                            0x5B230000      /* 64KB */
#define Connectivity_LPCGUSDHC3_PADDR                           0x5B220000      /* 64KB */
#define Connectivity_LPCGUSDHC2_PADDR                           0x5B210000      /* 64KB */
#define Connectivity_LPCGUSDHC1_PADDR                           0x5B200000      /* 64KB */
// #define RESERVED_PADDR  0x5B1A0000
#define Connectivity_USB3_PHY3P0_PADDR                          0x5B160000      /* 256KB */
#define Connectivity_USB3_PADDR                                 0x5B120000      /* 256KB */
#define Connectivity_USB3_CTRL_PADDR                            0x5B110000      /* 64KB */
#define Connectivity_USBOH_PHY_PADDR                            0x5B100000      /* 64KB */
#define Connectivity_USBOH_PL301_PADDR                          0x5B0F0000      /* 64KB */
#define Connectivity_USBOH_HSIC_PADDR                           0x5B0E0000      /* 64KB */
#define Connectivity_USBOH_OTG_PADDR                            0x5B0D0000      /* 64KB */
#define Connectivity_eDMA_CH4_PADDR                             0x5B0C0000      /* 64KB */
#define Connectivity_eDMA_CH3_PADDR                             0x5B0B0000      /* 64KB */
#define Connectivity_eDMA_CH2_PADDR                             0x5B0A0000      /* 64KB */
#define Connectivity_eDMA_CH1_PADDR                             0x5B090000      /* 64KB */
#define Connectivity_eDMA_CH0_PADDR                             0x5B080000      /* 64KB */
#define Connectivity_eDMA_MP_PADDR                              0x5B070000      /* 64KB */
#define Connectivity_MLB150_PADDR                               0x5B060000      /* 64KB */
#define Connectivity_ENET_AVB2_PADDR                            0x5B050000      /* 64KB */
#define Connectivity_ENET_AVB1_PADDR                            0x5B040000      /* 64KB */
#define Connectivity_USDHC3_PADDR                               0x5B030000      /* 64KB */
#define Connectivity_USDHC2_PADDR                               0x5B020000      /* 64KB */
#define Connectivity_USDHC1_PADDR                               0x5B010000      /* 64KB */
// #define RESERVED_PADDR  0x5B000000
/* Device: DMA (127) ________________________________________________________________________________ */
// #define RESERVED_PADDR  0x5AD00000
#define DMA_LPCG1_FlexCAN2_PADDR                                0x5ACF0000      /* 64KB */
#define DMA_LPCG1_FlexCAN1_PADDR                                0x5ACE0000      /* 64KB */
#define DMA_LPCG1_FlexCAN0_PADDR                                0x5ACD0000      /* 64KB */
// #define RESERVED_PADDR  0x5ACC0000
#define DMA_LPCG1_FTM1_PADDR                                    0x5ACB0000      /* 64KB */
#define DMA_LPCG1_FTM0_PADDR                                    0x5ACA0000      /* 64KB */
#define DMA_LPCG1_ADC1_PADDR                                    0x5AC90000      /* 64KB */
#define DMA_LPCG1_ADC0_PADDR                                    0x5AC80000      /* 64KB */
// #define RESERVED_PADDR  0x5AC50000
#define DMA_LPCG1_LPI2C4_PADDR                                  0x5AC40000      /* 64KB */
#define DMA_LPCG1_LPI2C3_PADDR                                  0x5AC30000      /* 64KB */
#define DMA_LPCG1_LPI2C2_PADDR                                  0x5AC20000      /* 64KB */
#define DMA_LPCG1_LPI2C1_PADDR                                  0x5AC10000      /* 64KB */
#define DMA_LPCG1_LPI2C0_PADDR                                  0x5AC00000      /* 64KB */
#define DMA_eDMA1_Channel31_PADDR                               0x5ABF0000      /* 64KB */
#define DMA_eDMA1_Channel30_PADDR                               0x5ABE0000      /* 64KB */
#define DMA_eDMA1_Channel29_PADDR                               0x5ABD0000      /* 64KB */
#define DMA_eDMA1_Channel28_PADDR                               0x5ABC0000      /* 64KB */
#define DMA_eDMA1_Channel27_PADDR                               0x5ABB0000      /* 64KB */
#define DMA_eDMA1_Channel26_PADDR                               0x5ABA0000      /* 64KB */
#define DMA_eDMA1_Channel25_PADDR                               0x5AB90000      /* 64KB */
#define DMA_eDMA1_Channel24_PADDR                               0x5AB80000      /* 64KB */
#define DMA_eDMA1_Channel23_PADDR                               0x5AB70000      /* 64KB */
#define DMA_eDMA1_Channel22_PADDR                               0x5AB60000      /* 64KB */
#define DMA_eDMA1_Channel21_PADDR                               0x5AB50000      /* 64KB */
#define DMA_eDMA1_Channel20_PADDR                               0x5AB40000      /* 64KB */
#define DMA_eDMA1_Channel19_PADDR                               0x5AB30000      /* 64KB */
#define DMA_eDMA1_Channel18_PADDR                               0x5AB20000      /* 64KB */
#define DMA_eDMA1_Channel17_PADDR                               0x5AB10000      /* 64KB */
#define DMA_eDMA1_Channel16_PADDR                               0x5AB00000      /* 64KB */
#define DMA_eDMA1_Channel15_PADDR                               0x5AAF0000      /* 64KB */
#define DMA_eDMA1_Channel14_PADDR                               0x5AAE0000      /* 64KB */
#define DMA_eDMA1_Channel13_PADDR                               0x5AAD0000      /* 64KB */
#define DMA_eDMA1_Channel12_PADDR                               0x5AAC0000      /* 64KB */
#define DMA_eDMA1_Channel11_PADDR                               0x5AAB0000      /* 64KB */
#define DMA_eDMA1_Channel10_PADDR                               0x5AAA0000      /* 64KB */
#define DMA_eDMA1_Channel9_PADDR                                0x5AA90000      /* 64KB */
#define DMA_eDMA1_Channel8_PADDR                                0x5AA80000      /* 64KB */
#define DMA_eDMA1_Channel7_PADDR                                0x5AA70000      /* 64KB */
#define DMA_eDMA1_Channel6_PADDR                                0x5AA60000      /* 64KB */
#define DMA_eDMA1_Channel5_PADDR                                0x5AA50000      /* 64KB */
#define DMA_eDMA1_Channel4_PADDR                                0x5AA40000      /* 64KB */
#define DMA_eDMA1_Channel3_PADDR                                0x5AA30000      /* 64KB */
#define DMA_eDMA1_Channel2_PADDR                                0x5AA20000      /* 64KB */
#define DMA_eDMA1_Channel1_PADDR                                0x5AA10000      /* 64KB */
#define DMA_eDMA1_Channel0_PADDR                                0x5AA00000      /* 64KB */
#define DMA_eDMA1_Control_PADDR                                 0x5A9F0000      /* 64KB */
// #define RESERVED_PADDR  0x5A900000
#define DMA_FlexCAN2_PADDR                                      0x5A8F0000      /* 64KB */
#define DMA_FlexCAN1_PADDR                                      0x5A8E0000      /* 64KB */
#define DMA_FlexCAN0_PADDR                                      0x5A8D0000      /* 64KB */
// #define RESERVED_PADDR  0x5A8C0000
#define DMA_FTM1_PADDR                                          0x5A8B0000      /* 64KB */
#define DMA_FTM0_PADDR                                          0x5A8A0000      /* 64KB */
#define DMA_ADC1_PADDR                                          0x5A890000      /* 64KB */
#define DMA_ADC0_PADDR                                          0x5A880000      /* 64KB */
// #define RESERVED_PADDR  0x5A870000
// #define RESERVED_PADDR  0x5A860000
// #define RESERVED_PADDR  0x5A850000
#define DMA_LPI2C4_PADDR                                        0x5A840000      /* 64KB */
#define DMA_LPI2C3_PADDR                                        0x5A830000      /* 64KB */
#define DMA_LPI2C2_PADDR                                        0x5A820000      /* 64KB */
#define DMA_LPI2C1_PADDR                                        0x5A810000      /* 64KB */
#define DMA_LPI2C0_PADDR                                        0x5A800000      /* 64KB */
// #define RESERVED_PADDR  0x5A5F0000
// #define RESERVED_PADDR  0x5A4F0000
#define DMA_LPCG0_EMVSIM1_PADDR                                 0x5A4E0000      /* 64KB */
#define DMA_LPCG0_EMVSIM0_PADDR                                 0x5A4D0000      /* 64KB */
// #define RESERVED_PADDR  0x5A4B0000
#define DMA_LPCG0_LPUART4_PADDR                                 0x5A4A0000      /* 64KB */
#define DMA_LPCG0_LPUART3_PADDR                                 0x5A490000      /* 64KB */
#define DMA_LPCG0_LPUART2_PADDR                                 0x5A480000      /* 64KB */
#define DMA_LPCG0_LPUART1_PADDR                                 0x5A470000      /* 64KB */
#define DMA_LPCG0_LPUART0_PADDR                                 0x5A460000      /* 64KB */
// #define RESERVED_PADDR  0x5A440000
#define DMA_LPCG0_LPSPI3_PADDR                                  0x5A430000      /* 64KB */
#define DMA_LPCG0_LPSPI2_PADDR                                  0x5A420000      /* 64KB */
#define DMA_LPCG0_LPSPI1_PADDR                                  0x5A410000      /* 64KB */
#define DMA_LPCG0_LPSPI0_PADDR                                  0x5A400000      /* 64KB */
#define DMA_eDMA0_Channel31_PADDR                               0x5A3F0000      /* 64KB */
#define DMA_eDMA0_Channel30_PADDR                               0x5A3E0000      /* 64KB */
#define DMA_eDMA0_Channel29_PADDR                               0x5A3D0000      /* 64KB */
#define DMA_eDMA0_Channel28_PADDR                               0x5A3C0000      /* 64KB */
#define DMA_eDMA0_Channel27_PADDR                               0x5A3B0000      /* 64KB */
#define DMA_eDMA0_Channel26_PADDR                               0x5A3A0000      /* 64KB */
#define DMA_eDMA0_Channel25_PADDR                               0x5A390000      /* 64KB */
#define DMA_eDMA0_Channel24_PADDR                               0x5A380000      /* 64KB */
#define DMA_eDMA0_Channel23_PADDR                               0x5A370000      /* 64KB */
#define DMA_eDMA0_Channel22_PADDR                               0x5A360000      /* 64KB */
#define DMA_eDMA0_Channel21_PADDR                               0x5A350000      /* 64KB */
#define DMA_eDMA0_Channel20_PADDR                               0x5A340000      /* 64KB */
#define DMA_eDMA0_Channel19_PADDR                               0x5A330000      /* 64KB */
#define DMA_eDMA0_Channel18_PADDR                               0x5A320000      /* 64KB */
#define DMA_eDMA0_Channel17_PADDR                               0x5A310000      /* 64KB */
#define DMA_eDMA0_Channel16_PADDR                               0x5A300000      /* 64KB */
#define DMA_eDMA0_Channel15_PADDR                               0x5A2F0000      /* 64KB */
#define DMA_eDMA0_Channel14_PADDR                               0x5A2E0000      /* 64KB */
#define DMA_eDMA0_Channel13_PADDR                               0x5A2D0000      /* 64KB */
#define DMA_eDMA0_Channel12_PADDR                               0x5A2C0000      /* 64KB */
#define DMA_eDMA0_Channel11_PADDR                               0x5A2B0000      /* 64KB */
#define DMA_eDMA0_Channel10_PADDR                               0x5A2A0000      /* 64KB */
#define DMA_eDMA0_Channel9_PADDR                                0x5A290000      /* 64KB */
#define DMA_eDMA0_Channel8_PADDR                                0x5A280000      /* 64KB */
#define DMA_eDMA0_Channel7_PADDR                                0x5A270000      /* 64KB */
#define DMA_eDMA0_Channel6_PADDR                                0x5A260000      /* 64KB */
#define DMA_eDMA0_Channel5_PADDR                                0x5A250000      /* 64KB */
#define DMA_eDMA0_Channel4_PADDR                                0x5A240000      /* 64KB */
#define DMA_eDMA0_Channel3_PADDR                                0x5A230000      /* 64KB */
#define DMA_eDMA0_Channel2_PADDR                                0x5A220000      /* 64KB */
#define DMA_eDMA0_Channel1_PADDR                                0x5A210000      /* 64KB */
#define DMA_eDMA0_Channel0_PADDR                                0x5A200000      /* 64KB */
#define DMA_eDMA0_Control_PADDR                                 0x5A1F0000      /* 64KB */
// #define RESERVED_PADDR  0x5A0F0000
#define DMA_EMVSIM1_PADDR                                       0x5A0E0000      /* 64KB */
#define DMA_EMVSIM0_PADDR                                       0x5A0D0000      /* 64KB */
// #define RESERVED_PADDR  0x5A0B0000
#define DMA_LPUART4_PADDR                                       0x5A0A0000      /* 64KB */
#define DMA_LPUART3_PADDR                                       0x5A090000      /* 64KB */
#define DMA_LPUART2_PADDR                                       0x5A080000      /* 64KB */
#define DMA_LPUART1_PADDR                                       0x5A070000      /* 64KB */
#define DMA_LPUART0_PADDR                                       0x5A060000      /* 64KB */
// #define RESERVED_PADDR  0x5A040000
#define DMA_LPSPI3_PADDR                                        0x5A030000      /* 64KB */
#define DMA_LPSPI2_PADDR                                        0x5A020000      /* 64KB */
#define DMA_LPSPI1_PADDR                                        0x5A010000      /* 64KB */
#define DMA_LPSPI0_PADDR                                        0x5A000000      /* 64KB */
/* Device: Audio (131) ______________________________________________________________________________ */
// #define RESERVED_PADDR  0x59FE0000
// #define RESERVED_PADDR  0x59FD0000
#define Audio_ACM_PADDR                                         0x59E00000      /* 1.8MB */
#define Audio_LPCG_eDMA1_PADDR                                  0x59DF0000      /* 64KB */
// #define RESERVED_PADDR  0x59D70000
#define Audio_LPCG_MCLKOUT1_PADDR                               0x59D60000      /* 64KB */
#define Audio_LPCG_MCLKOUT0_PADDR                               0x59D50000      /* 64KB */
// #define RESERVED_PADDR  0x59D40000
#define Audio_AUD_PLL_DIV_CLK1_PADDR                            0x59D30000      /* 64KB */
#define Audio_AUD_PLL_DIV_CLK0_PADDR                            0x59D20000      /* 64KB */
#define Audio_LPCG_AUD_REC_CLK1_PADDR                           0x59D10000      /* 64KB */
#define Audio_LPCG_AUD_REC_CLK0_PADDR                           0x59D00000      /* 64KB */
// #define RESERVED_PADDR  0x59C70000
#define Audio_LPCG_ACM_PADDR                                    0x59C60000      /* 64KB */
#define Audio_LPCG_MQS_PADDR                                    0x59C50000      /* 64KB */
#define Audio_LPCG_AMIX_PADDR                                   0x59C40000      /* 64KB */
#define Audio_LPCG_SAI7_PADDR                                   0x59C30000      /* 64KB */
#define Audio_LPCG_SAI6_PADDR                                   0x59C20000      /* 64KB */
#define Audio_LPCG_ESAI1_PADDR                                  0x59C10000      /* 64KB */
#define Audio_LPCG_ASRC1_PADDR                                  0x59C00000      /* 64KB */
#define Audio_eDMA1_Channel31_PADDR                             0x59BF0000      /* 64KB */
#define Audio_eDMA1_Channel30_PADDR                             0x59BE0000      /* 64KB */
#define Audio_eDMA1_Channel29_PADDR                             0x59BD0000      /* 64KB */
#define Audio_eDMA1_Channel28_PADDR                             0x59BC0000      /* 64KB */
#define Audio_eDMA1_Channel27_PADDR                             0x59BB0000      /* 64KB */
#define Audio_eDMA1_Channel26_PADDR                             0x59BA0000      /* 64KB */
#define Audio_eDMA1_Channel25_PADDR                             0x59B90000      /* 64KB */
#define Audio_eDMA1_Channel24_PADDR                             0x59B80000      /* 64KB */
#define Audio_eDMA1_Channel23_PADDR                             0x59B70000      /* 64KB */
#define Audio_eDMA1_Channel22_PADDR                             0x59B60000      /* 64KB */
#define Audio_eDMA1_Channel21_PADDR                             0x59B50000      /* 64KB */
#define Audio_eDMA1_Channel20_PADDR                             0x59B40000      /* 64KB */
#define Audio_eDMA1_Channel19_PADDR                             0x59B30000      /* 64KB */
#define Audio_eDMA1_Channel18_PADDR                             0x59B20000      /* 64KB */
#define Audio_eDMA1_Channel17_PADDR                             0x59B10000      /* 64KB */
#define Audio_eDMA1_Channel16_PADDR                             0x59B00000      /* 64KB */
#define Audio_eDMA1_Channel15_PADDR                             0x59AF0000      /* 64KB */
#define Audio_eDMA1_Channel14_PADDR                             0x59AE0000      /* 64KB */
#define Audio_eDMA1_Channel13_PADDR                             0x59AD0000      /* 64KB */
#define Audio_eDMA1_Channel12_PADDR                             0x59AC0000      /* 64KB */
#define Audio_eDMA1_Channel11_PADDR                             0x59AB0000      /* 64KB */
#define Audio_eDMA1_Channel10_PADDR                             0x59AA0000      /* 64KB */
#define Audio_eDMA1_Channel9_PADDR                              0x59A90000      /* 64KB */
#define Audio_eDMA1_Channel8_PADDR                              0x59A80000      /* 64KB */
#define Audio_eDMA1_Channel7_PADDR                              0x59A70000      /* 64KB */
#define Audio_eDMA1_Channel6_PADDR                              0x59A60000      /* 64KB */
#define Audio_eDMA1_Channel5_PADDR                              0x59A50000      /* 64KB */
#define Audio_eDMA1_Channel4_PADDR                              0x59A40000      /* 64KB */
#define Audio_eDMA1_Channel3_PADDR                              0x59A30000      /* 64KB */
#define Audio_eDMA1_Channel2_PADDR                              0x59A20000      /* 64KB */
#define Audio_eDMA1_Channel1_PADDR                              0x59A10000      /* 64KB */
#define Audio_eDMA1_Channel0_PADDR                              0x59A00000      /* 64KB */
#define Audio_eDMA1_Control_PADDR                               0x599F0000      /* 64KB */
// #define RESERVED_PADDR  0x59860000
#define Audio_MQS_PADDR                                         0x59850000      /* 64KB */
#define Audio_AMIX_PADDR                                        0x59840000      /* 64KB */
#define Audio_SAI7_PADDR                                        0x59830000      /* 64KB */
#define Audio_SAI6_PADDR                                        0x59820000      /* 64KB */
#define Audio_ESAI1_PADDR                                       0x59810000      /* 64KB */
#define Audio_ASRC1_PADDR                                       0x59800000      /* 64KB */
// #define RESERVED_PADDR  0x59600000
#define Audio_LPCG_eDMA0_PADDR                                  0x595F0000      /* 64KB */
// #define RESERVED_PADDR  0x59510000
#define Audio_LPCG_GPT5_PADDR                                   0x59500000      /* 64KB */
#define Audio_LPCG_GPT4_PADDR                                   0x594F0000      /* 64KB */
#define Audio_LPCG_GPT3_PADDR                                   0x594E0000      /* 64KB */
#define Audio_LPCG_GPT2_PADDR                                   0x594D0000      /* 64KB */
#define Audio_LPCG_GPT1_PADDR                                   0x594C0000      /* 64KB */
#define Audio_LPCG_GPT0_PADDR                                   0x594B0000      /* 64KB */
// #define RESERVED_PADDR  0x594A0000
#define Audio_LPCG_HDMI_TX_SAI0_PADDR                           0x59490000      /* 64KB */
#define Audio_LPCG_HDMI_RX_SAI0_PADDR                           0x59480000      /* 64KB */
#define Audio_LPCG_SAI3_PADDR                                   0x59470000      /* 64KB */
#define Audio_LPCG_SAI2_PADDR                                   0x59460000      /* 64KB */
#define Audio_LPCG_SAI1_PADDR                                   0x59450000      /* 64KB */
#define Audio_LPCG_SAI0_PADDR                                   0x59440000      /* 64KB */
#define Audio_LPCG_SPDIF1_PADDR                                 0x59430000      /* 64KB */
#define Audio_LPCG_SPDIF0_PADDR                                 0x59420000      /* 64KB */
#define Audio_LPCG_ESAI0_PADDR                                  0x59410000      /* 64KB */
#define Audio_LPCG_ASRC0_PADDR                                  0x59400000      /* 64KB */
#define Audio_eDMA0_Channel31_PADDR                             0x593F0000      /* 64KB */
#define Audio_eDMA0_Channel30_PADDR                             0x593E0000      /* 64KB */
#define Audio_eDMA0_Channel29_PADDR                             0x593D0000      /* 64KB */
#define Audio_eDMA0_Channel28_PADDR                             0x593C0000      /* 64KB */
#define Audio_eDMA0_Channel27_PADDR                             0x593B0000      /* 64KB */
#define Audio_eDMA0_Channel26_PADDR                             0x593A0000      /* 64KB */
#define Audio_eDMA0_Channel25_PADDR                             0x59390000      /* 64KB */
#define Audio_eDMA0_Channel24_PADDR                             0x59380000      /* 64KB */
#define Audio_eDMA0_Channel23_PADDR                             0x59370000      /* 64KB */
#define Audio_eDMA0_Channel22_PADDR                             0x59360000      /* 64KB */
#define Audio_eDMA0_Channel21_PADDR                             0x59350000      /* 64KB */
#define Audio_eDMA0_Channel20_PADDR                             0x59340000      /* 64KB */
#define Audio_eDMA0_Channel19_PADDR                             0x59330000      /* 64KB */
#define Audio_eDMA0_Channel18_PADDR                             0x59320000      /* 64KB */
#define Audio_eDMA0_Channel17_PADDR                             0x59310000      /* 64KB */
#define Audio_eDMA0_Channel16_PADDR                             0x59300000      /* 64KB */
#define Audio_eDMA0_Channel15_PADDR                             0x592F0000      /* 64KB */
#define Audio_eDMA0_Channel14_PADDR                             0x592E0000      /* 64KB */
#define Audio_eDMA0_Channel13_PADDR                             0x592D0000      /* 64KB */
#define Audio_eDMA0_Channel12_PADDR                             0x592C0000      /* 64KB */
#define Audio_eDMA0_Channel11_PADDR                             0x592B0000      /* 64KB */
#define Audio_eDMA0_Channel10_PADDR                             0x592A0000      /* 64KB */
#define Audio_eDMA0_Channel9_PADDR                              0x59290000      /* 64KB */
#define Audio_eDMA0_Channel8_PADDR                              0x59280000      /* 64KB */
#define Audio_eDMA0_Channel7_PADDR                              0x59270000      /* 64KB */
#define Audio_eDMA0_Channel6_PADDR                              0x59260000      /* 64KB */
#define Audio_eDMA0_Channel5_PADDR                              0x59250000      /* 64KB */
#define Audio_eDMA0_Channel4_PADDR                              0x59240000      /* 64KB */
#define Audio_eDMA0_Channel3_PADDR                              0x59230000      /* 64KB */
#define Audio_eDMA0_Channel2_PADDR                              0x59220000      /* 64KB */
#define Audio_eDMA0_Channel1_PADDR                              0x59210000      /* 64KB */
#define Audio_eDMA0_Channel0_PADDR                              0x59200000      /* 64KB */
#define Audio_eDMA0_Control_PADDR                               0x591F0000      /* 64KB */
// #define RESERVED_PADDR  0x59110000
#define Audio_GPT5_PADDR                                        0x59100000      /* 64KB */
#define Audio_GPT4_PADDR                                        0x590F0000      /* 64KB */
#define Audio_GPT3_PADDR                                        0x590E0000      /* 64KB */
#define Audio_GPT2_PADDR                                        0x590D0000      /* 64KB */
#define Audio_GPT1_PADDR                                        0x590C0000      /* 64KB */
#define Audio_GPT0_PADDR                                        0x590B0000      /* 64KB */
// #define RESERVED_PADDR  0x590A0000
#define Audio_HDMI_TX_SAI0_PADDR                                0x59090000      /* 64KB */
#define Audio_HDMI_RX_SAI0_PADDR                                0x59080000      /* 64KB */
#define Audio_SAI3_PADDR                                        0x59070000      /* 64KB */
#define Audio_SAI2_PADDR                                        0x59060000      /* 64KB */
#define Audio_SAI1_PADDR                                        0x59050000      /* 64KB */
#define Audio_SAI0_PADDR                                        0x59040000      /* 64KB */
#define Audio_SPDIF1_PADDR                                      0x59030000      /* 64KB */
#define Audio_SPDIF0_PADDR                                      0x59020000      /* 64KB */
#define Audio_ESAI0_PADDR                                       0x59010000      /* 64KB */
#define Audio_ASRC0_PADDR                                       0x59000000      /* 64KB */
/* Device: VPU (15) _________________________________________________________________________________ */
// #define RESERVED_PADDR  0x2CB00000
#define VPU_me_PADDR                                            0x2CA00000      /* 1MB */
#define VPU_Venc_PADDR                                          0x2C800000      /* 1MB */
// #define RESERVED_PADDR  0x2C720000
#define VPU_XUVI_PADDR                                          0x2C700000      /* 128KB */
// #define RESERVED_PADDR  0x2C500000
#define VPU_VBIB_PADDR                                          0x2C400000      /* 1MB */
#define VPU_FWPU_PADDR                                          0x2C230000      /* 1.8MB */
#define VPU_MU_DSP_PADDR                                        0x2C220000      /* 64KB */
#define VPU_MU_MCU_PADDR                                        0x2C210000      /* 64KB */
#define VPU_UART_PADDR                                          0x2C200000      /* 64KB */
// #define RESERVED_PADDR  0x2C1C0000
#define VPU_MFD_PADDR                                           0x2C180000      /* 256KB */
// #define RESERVED_PADDR  0x2C100000
#define VPU_SCB_PADDR                                           0x2C000000      /* 1MB */
/* Device: GPU (6) __________________________________________________________________________________ */
// #define RESERVED_PADDR  0x54140000
#define GPU_GPU_1_GC7K_XSVX_PADDR                               0x54100000      /* 256KB */
// #define RESERVED_PADDR  0x54000000
// #define RESERVED_PADDR  0x53140000
#define GPU_GPU_0_GC7K_XSVX_PADDR                               0x53100000      /* 256KB */
// #define RESERVED_PADDR  0x53000000
/* Device: Display Controller 0 (25) ________________________________________________________________ */
// #define RESERVED_PADDR  0x56300000
#define Display_Controller_0_HDP_PADDR                          0x56200000      /* 1MB */
// #define RESERVED_PADDR  0x561C0000
#define Display_Controller_0_DC_SEERIS_PADDR                    0x56180000      /* 256KB */
// #define RESERVED_PADDR  0x56140000
// #define RESERVED_PADDR  0x56130000
#define Display_Controller_0_DPR1_CH2_PADDR                     0x56120000      /* 64KB */
#define Display_Controller_0_DPR1_CH1_PADDR                     0x56110000      /* 64KB */
#define Display_Controller_0_DPR1_CH0_PADDR                     0x56100000      /* 64KB */
#define Display_Controller_0_DPR0_CH2_PADDR                     0x560F0000      /* 64KB */
#define Display_Controller_0_DPR0_CH1_PADDR                     0x560E0000      /* 64KB */
#define Display_Controller_0_DPR0_CH0_PADDR                     0x560D0000      /* 64KB */
#define Display_Controller_0_PRG8_PADDR                         0x560C0000      /* 64KB */
#define Display_Controller_0_PRG7_PADDR                         0x560B0000      /* 64KB */
#define Display_Controller_0_PRG6_PADDR                         0x560A0000      /* 64KB */
#define Display_Controller_0_PRG5_PADDR                         0x56090000      /* 64KB */
#define Display_Controller_0_PRG4_PADDR                         0x56080000      /* 64KB */
#define Display_Controller_0_PRG3_PADDR                         0x56070000      /* 64KB */
#define Display_Controller_0_PRG2_PADDR                         0x56060000      /* 64KB */
#define Display_Controller_0_PRG1_PADDR                         0x56050000      /* 64KB */
#define Display_Controller_0_PRG0_PADDR                         0x56040000      /* 64KB */
#define Display_Controller_0_LTS_PADDR                          0x56030000      /* 64KB */
#define Display_Controller_0_Pixelcombiner_PADDR                0x56020000      /* 64KB */
#define Display_Controller_0_LPCG_PADDR                         0x56010000      /* 64KB */
#define Display_Controller_0_INT_Steer_PADDR                    0x56000000      /* 64KB */
/* Device: Display Controller 1 (69) ________________________________________________________________ */
// #define RESERVED_PADDR  0x57300000
#define Display_Controller_1_HDP_PADDR                          0x57200000      /* 1MB */
// #define RESERVED_PADDR  0x571C0000
#define Display_Controller_1_DC_SEERIS_PADDR                    0x57180000      /* 256KB */
// #define RESERVED_PADDR  0x57140000
// #define RESERVED_PADDR  0x57130000
#define Display_Controller_1_DPR1_CH2_PADDR                     0x57120000      /* 64KB */
#define Display_Controller_1_DPR1_CH1_PADDR                     0x57110000      /* 64KB */
#define Display_Controller_1_DPR1_CH0_PADDR                     0x57100000      /* 64KB */
#define Display_Controller_1_DPR0_CH2_PADDR                     0x570F0000      /* 64KB */
#define Display_Controller_1_DPR0_CH1_PADDR                     0x570E0000      /* 64KB */
#define Display_Controller_1_DPR0_CH0_PADDR                     0x570D0000      /* 64KB */
#define Display_Controller_1_PRG8_PADDR                         0x570C0000      /* 64KB */
#define Display_Controller_1_PRG7_PADDR                         0x570B0000      /* 64KB */
#define Display_Controller_1_PRG6_PADDR                         0x570A0000      /* 64KB */
#define Display_Controller_1_PRG5_PADDR                         0x57090000      /* 64KB */
#define Display_Controller_1_PRG4_PADDR                         0x57080000      /* 64KB */
#define Display_Controller_1_PRG3_PADDR                         0x57070000      /* 64KB */
#define Display_Controller_1_PRG2_PADDR                         0x57060000      /* 64KB */
#define Display_Controller_1_PRG1_PADDR                         0x57050000      /* 64KB */
#define Display_Controller_1_PRG0_PADDR                         0x57040000      /* 64KB */
#define Display_Controller_1_LTS_PADDR                          0x57030000      /* 64KB */
#define Display_Controller_1_Pixelcombiner_PADDR                0x57020000      /* 64KB */
#define Display_Controller_1_LPCG_PADDR                         0x57010000      /* 64KB */
#define Display_Controller_1_INT_Steer_PADDR                    0x57000000      /* 64KB */
// #define RESERVED_PADDR  0x56400000
// #define RESERVED_PADDR  0x56320000
// #define RESERVED_PADDR  0x56310000
// #define RESERVED_PADDR  0x56300000
// #define RESERVED_PADDR  0x562C0000
// #define RESERVED_PADDR  0x56290000
// #define RESERVED_PADDR  0x56280000
// #define RESERVED_PADDR  0x56270000
// #define RESERVED_PADDR  0x5626F000
// #define RESERVED_PADDR  0x5626A000
#define Display_Controller_1_HD_TX_CTRL1_PADDR                  0x56269000      /* 4KB */
#define Display_Controller_1_HD_TX_CTRL0_PADDR                  0x56268000      /* 4KB */
// #define RESERVED_PADDR  0x56267000
#define Display_Controller_1_I2C_PADDR                          0x56266000      /* 4KB */
// #define RESERVED_PADDR  0x56265000
#define Display_Controller_1_PWM_PADDR                          0x56264000      /* 4KB */
#define Display_Controller_1_LPCG_2_PADDR                       0x56263000      /* 4KB */
#define Display_Controller_1_GPIO_PADDR                         0x56262000      /* 4KB */
#define Display_Controller_1_CSR_PADDR                          0x56261000      /* 4KB */
#define Display_Controller_1_INT_PADDR                          0x56260000      /* 4KB */
// #define RESERVED_PADDR  0x56250000
// #define RESERVED_PADDR  0x56249000
// #define RESERVED_PADDR  0x56248000
#define Display_Controller_1_I2C1_PADDR                         0x56247000      /* 4KB */
#define Display_Controller_1_I2C0_PADDR                         0x56246000      /* 4KB */
// #define RESERVED_PADDR  0x56245000
#define Display_Controller_1_PWM_2_PADDR                        0x56244000      /* 4KB */
#define Display_Controller_1_LPCG_3_PADDR                       0x56243000      /* 4KB */
#define Display_Controller_1_GPIO_2_PADDR                       0x56242000      /* 4KB */
#define Display_Controller_1_CSR_2_PADDR                        0x56241000      /* 4KB */
#define Display_Controller_1_INT_2_PADDR                        0x56240000      /* 4KB */
// #define RESERVED_PADDR  0x56230000
// #define RESERVED_PADDR  0x5622A000
#define Display_Controller_1_MIPITXPacketInterface_PADDR        0x56229000      /* 4KB */
#define Display_Controller_1_MIPIDSIController_PADDR            0x56228000      /* 4KB */
#define Display_Controller_1_ISC1_PADDR                         0x56227000      /* 4KB */
#define Display_Controller_1_I2C0_2_PADDR                       0x56226000      /* 4KB */
// #define RESERVED_PADDR  0x56225000
#define Display_Controller_1_PWM_3_PADDR                        0x56224000      /* 4KB */
#define Display_Controller_1_LPCG_4_PADDR                       0x56223000      /* 4KB */
#define Display_Controller_1_GPIO_3_PADDR                       0x56222000      /* 4KB */
#define Display_Controller_1_CSR_3_PADDR                        0x56221000      /* 4KB */
#define Display_Controller_1_INT_3_PADDR                        0x56220000      /* 4KB */
#define Display_Controller_1_DCmapped_PADDR                     0x56000000      /* 2MB */
/* Device: Display Interface 1 (31) _________________________________________________________________ */
// #define RESERVED_PADDR  0x57400000
// #define RESERVED_PADDR  0x57320000
// #define RESERVED_PADDR  0x57310000
// #define RESERVED_PADDR  0x57300000
// #define RESERVED_PADDR  0x572C0000
// #define RESERVED_PADDR  0x57290000
// #define RESERVED_PADDR  0x57260000
// #define RESERVED_PADDR  0x57250000
// #define RESERVED_PADDR  0x57248000
#define Display_Interface_1_I2C1_PADDR                          0x57247000      /* 4KB */
#define Display_Interface_1_I2C0_PADDR                          0x57246000      /* 4KB */
// #define RESERVED_PADDR  0x57245000
#define Display_Interface_1_PWM_PADDR                           0x57244000      /* 4KB */
#define Display_Interface_1_LPCG_PADDR                          0x57243000      /* 4KB */
#define Display_Interface_1_GPIO_PADDR                          0x57242000      /* 4KB */
#define Display_Interface_1_CSR_PADDR                           0x57241000      /* 4KB */
#define Display_Interface_1_Int_PADDR                           0x57240000      /* 4KB */
// #define RESERVED_PADDR  0x57230000
// #define RESERVED_PADDR  0x5722A000
#define Display_Interface_1_MIPITXPacketInterface_PADDR         0x57229000      /* 4KB */
#define Display_Interface_1_MIPIDSIController_PADDR             0x57228000      /* 4KB */
#define Display_Interface_1_I2C1_2_PADDR                        0x57227000      /* 4KB */
#define Display_Interface_1_I2C0_2_PADDR                        0x57226000      /* 4KB */
// #define RESERVED_PADDR  0x57225000
#define Display_Interface_1_PWM_2_PADDR                         0x57224000      /* 4KB */
#define Display_Interface_1_LPCG_2_PADDR                        0x57223000      /* 4KB */
#define Display_Interface_1_GPIO_2_PADDR                        0x57222000      /* 4KB */
#define Display_Interface_1_CSR_2_PADDR                         0x57221000      /* 4KB */
#define Display_Interface_1_Int_2_PADDR                         0x57220000      /* 4KB */
#define Display_Interface_1_DCmapped_PADDR                      0x57210000      /* 192KB */
#define Display_Interface_1_DCmapped_2_PADDR                    0x57000000      /* 2MB */
/* Device: Imaging (66) _____________________________________________________________________________ */
// #define RESERVED_PADDR  0x586E0000
// #define RESERVED_PADDR  0x58610000
// #define RESERVED_PADDR  0x58600000
#define Imaging_MJPEG_Common_Enc_EN_PADDR                       0x585F0000      /* 64KB */
// #define RESERVED_PADDR  0x585E0000
#define Imaging_MJPEG_Common_Dec_EN_PADDR                       0x585D0000      /* 64KB */
#define Imaging_PIXEL_SLAVE_DISP_1_PADDR                        0x585C0000      /* 64KB */
#define Imaging_PIXEL_SLAVE_DISP_0_PADDR                        0x585B0000      /* 64KB */
#define Imaging_PIXEL_SLAVE_HDMI_0_PADDR                        0x585A0000      /* 64KB */
#define Imaging_PIXEL_SLAVE_MIPI_CSI_1_PADDR                    0x58590000      /* 64KB */
#define Imaging_PIXEL_SLAVE_MIPI_CSI_0_PADDR                    0x58580000      /* 64KB */
#define Imaging_PixelDMAStream_7_PADDR                          0x58570000      /* 64KB */
#define Imaging_PixelDMAStream_6_PADDR                          0x58560000      /* 64KB */
#define Imaging_PixelDMAStream_5_PADDR                          0x58550000      /* 64KB */
#define Imaging_PixelDMAStream_4_PADDR                          0x58540000      /* 64KB */
#define Imaging_PixelDMAStream_3_PADDR                          0x58530000      /* 64KB */
#define Imaging_PixelDMAStream_2_PADDR                          0x58520000      /* 64KB */
#define Imaging_PixelDMAStream_1_PADDR                          0x58510000      /* 64KB */
#define Imaging_PixelDMAStream_0_PADDR                          0x58500000      /* 64KB */
// #define RESERVED_PADDR  0x584A0000
#define Imaging_MJPEGENCODER_BS_3_PADDR                         0x58490000      /* 64KB */
#define Imaging_MJPEGENCODER_BS_2_PADDR                         0x58480000      /* 64KB */
#define Imaging_MJPEGENCODER_BS_1_PADDR                         0x58470000      /* 64KB */
#define Imaging_MJPEGENCODER_BS_0_PADDR                         0x58460000      /* 64KB */
// #define RESERVED_PADDR  0x58450500
#define Imaging_StatusRegs_PADDR                                0x58450400      /* 0.25KB */
#define Imaging_CFG_MODEControlRegs_PADDR                       0x58450300      /* 0.25KB */
#define Imaging_ControlModeRegs_PADDR                           0x58450200      /* 0.25KB */
#define Imaging_ControlRegs_PADDR                               0x58450100      /* 0.25KB */
#define Imaging_MemoryWrapper_PADDR                             0x58450000      /* 0.25KB */
#define Imaging_MJPEGDECODER_BS_3_PADDR                         0x58440000      /* 64KB */
#define Imaging_MJPEGDECODER_BS_2_PADDR                         0x58430000      /* 64KB */
#define Imaging_MJPEGDECODER_BS_1_PADDR                         0x58420000      /* 64KB */
#define Imaging_MJPEGDECODER_BS_0_PADDR                         0x58410000      /* 64KB */
// #define RESERVED_PADDR  0x58400300
#define Imaging_StatusRegs_2_PADDR                              0x58400200      /* 0.25KB */
#define Imaging_ControlRegs_2_PADDR                             0x58400100      /* 0.25KB */
#define Imaging_MemoryWrapper_2_PADDR                           0x58400000      /* 0.25KB */
// #define RESERVED_PADDR  0x58300000
// #define RESERVED_PADDR  0x582E0000
// #define RESERVED_PADDR  0x582C0000
// #define RESERVED_PADDR  0x582A0000
// #define RESERVED_PADDR  0x58280000
#define Imaging_HDMI_0_PADDR                                    0x58260000      /* 128KB */
#define Imaging_MIPI_CSI_1_PADDR                                0x58240000      /* 128KB */
#define Imaging_MIPI_CSI_0_PADDR                                0x58220000      /* 128KB */
#define Imaging_MSIControl_PADDR                                0x58200000      /* 128KB */
// #define RESERVED_PADDR  0x58180000
#define Imaging_Stream_7_PADDR                                  0x58170000      /* 64KB */
#define Imaging_Stream_6_PADDR                                  0x58160000      /* 64KB */
#define Imaging_Stream_5_PADDR                                  0x58150000      /* 64KB */
#define Imaging_Stream_4_PADDR                                  0x58140000      /* 64KB */
#define Imaging_Stream_3_PADDR                                  0x58130000      /* 64KB */
#define Imaging_Stream_2_PADDR                                  0x58120000      /* 64KB */
#define Imaging_Stream_1_PADDR                                  0x58110000      /* 64KB */
#define Imaging_Stream_0_PADDR                                  0x58100000      /* 64KB */
// #define RESERVED_PADDR  0x580A0000
// #define RESERVED_PADDR  0x58090000
// #define RESERVED_PADDR  0x58080000
// #define RESERVED_PADDR  0x58070000
#define Imaging_DISPLAY_IN_1_PADDR                              0x58060000      /* 64KB */
#define Imaging_DISPLAY_IN_0_PADDR                              0x58050000      /* 64KB */
#define Imaging_HDMI_0_2_PADDR                                  0x58040000      /* 64KB */
#define Imaging_MIPI_CSI_1_2_PADDR                              0x58030000      /* 64KB */
#define Imaging_MIPI_CSI_0_2_PADDR                              0x58020000      /* 64KB */
#define Imaging_General_Purpose_GPR_PADDR                       0x58000000      /* 128KB */
/* Device: MIPI-CSI2 (19) ___________________________________________________________________________ */
// #define RESERVED_PADDR  0x008000
#define MIPI_CSI2_MIPICSI_2Control_PADDR                        0x007000        /* 4KB */
#define MIPI_CSI2_I2C_PADDR                                     0x006000        /* 4KB */
// #define RESERVED_PADDR  0x005000
#define MIPI_CSI2_PWM_PADDR                                     0x004000        /* 4KB */
#define MIPI_CSI2_LPCG_PADDR                                    0x003000        /* 4KB */
#define MIPI_CSI2_GPIO_PADDR                                    0x002000        /* 4KB */
#define MIPI_CSI2_CSR_PADDR                                     0x001000        /* 4KB */
#define MIPI_CSI2_LocalInterruptSteer_PADDR                     0x000000        /* 4KB */
#define MIPI_CSI2_HDMIRXControl_PADDR                           0x270000        /* 64KB */
// #define RESERVED_PADDR  0x268000
// #define RESERVED_PADDR  0x267000
#define MIPI_CSI2_I2C_2_PADDR                                   0x266000        /* 4KB */
// #define RESERVED_PADDR  0x265000
#define MIPI_CSI2_PWM_2_PADDR                                   0x264000        /* 4KB */
#define MIPI_CSI2_LPCG_2_PADDR                                  0x263000        /* 4KB */
#define MIPI_CSI2_GPIO_2_PADDR                                  0x262000        /* 4KB */
#define MIPI_CSI2_CSR_2_PADDR                                   0x261000        /* 4KB */
#define MIPI_CSI2_Int_PADDR                                     0x260000        /* 4KB */
/* Device: Db (51) __________________________________________________________________________________ */
// #define RESERVED_PADDR  0x5CB00000
#define Db_LPCG_PADDR                                           0x5CAF0000      /* 64KB */
// #define RESERVED_PADDR  0x5C800000
#define Db_LPCG_2_PADDR                                         0x5C7F0000      /* 64KB */
// #define RESERVED_PADDR  0x5C740000
#define Db_STC3_PADDR                                           0x5C730000      /* 64KB */
#define Db_STC2_PADDR                                           0x5C720000      /* 64KB */
#define Db_STC1_PADDR                                           0x5C710000      /* 64KB */
#define Db_STC0_PADDR                                           0x5C700000      /* 64KB */
#define Db_LPCG_3_PADDR                                         0x5C6F0000      /* 64KB */
// #define RESERVED_PADDR  0x5C640000
#define Db_STC3_2_PADDR                                         0x5C630000      /* 64KB */
#define Db_STC2_2_PADDR                                         0x5C620000      /* 64KB */
#define Db_STC1_2_PADDR                                         0x5C610000      /* 64KB */
#define Db_STC0_2_PADDR                                         0x5C600000      /* 64KB */
#define Db_LPCG_4_PADDR                                         0x5C5F0000      /* 64KB */
// #define RESERVED_PADDR  0x5C540000
#define Db_STC3_3_PADDR                                         0x5C530000      /* 64KB */
#define Db_STC2_3_PADDR                                         0x5C520000      /* 64KB */
#define Db_STC1_3_PADDR                                         0x5C510000      /* 64KB */
#define Db_STC0_3_PADDR                                         0x5C500000      /* 64KB */
#define Db_LPCG_5_PADDR                                         0x5C4F0000      /* 64KB */
// #define RESERVED_PADDR  0x5C440000
#define Db_STC3_4_PADDR                                         0x5C430000      /* 64KB */
#define Db_STC2_4_PADDR                                         0x5C420000      /* 64KB */
#define Db_STC1_4_PADDR                                         0x5C410000      /* 64KB */
#define Db_STC0_4_PADDR                                         0x5C400000      /* 64KB */
#define Db_LPCG2_PADDR                                          0x5C3F0000      /* 64KB */
#define Db_LPCG1_PADDR                                          0x5C3E0000      /* 64KB */
#define Db_LPCG0_PADDR                                          0x5C3D0000      /* 64KB */
// #define RESERVED_PADDR  0x5C320000
#define Db_PHY0_PADDR                                           0x5C310000      /* 64KB */
#define Db_DRC0_PADDR                                           0x5C300000      /* 64KB */
#define Db_LPCG2_2_PADDR                                        0x5C2F0000      /* 64KB */
#define Db_LPCG1_2_PADDR                                        0x5C2E0000      /* 64KB */
#define Db_LPCG0_2_PADDR                                        0x5C2D0000      /* 64KB */
// #define RESERVED_PADDR  0x5C220000
#define Db_PHY0_2_PADDR                                         0x5C210000      /* 64KB */
#define Db_DRC0_2_PADDR                                         0x5C200000      /* 64KB */
#define Db_LPCG2_3_PADDR                                        0x5C1F0000      /* 64KB */
#define Db_LPCG1_3_PADDR                                        0x5C1E0000      /* 64KB */
#define Db_LPCG0_3_PADDR                                        0x5C1D0000      /* 64KB */
// #define RESERVED_PADDR  0x5C120000
#define Db_PHY0_3_PADDR                                         0x5C110000      /* 64KB */
#define Db_DRC0_3_PADDR                                         0x5C100000      /* 64KB */
#define Db_LPCG2_4_PADDR                                        0x5C0F0000      /* 64KB */
#define Db_LPCG1_4_PADDR                                        0x5C0E0000      /* 64KB */
#define Db_LPCG0_4_PADDR                                        0x5C0D0000      /* 64KB */
// #define RESERVED_PADDR  0x5C020000
#define Db_PHY0_4_PADDR                                         0x5C010000      /* 64KB */
#define Db_DRC0_4_PADDR                                         0x5C000000      /* 64KB */
/* Device: Dblog (14) _______________________________________________________________________________ */
// #define RESERVED_PADDR  0x51C00000
#define Dblog_GIC_PADDR                                         0x51A00000      /* 2MB */
// #define RESERVED_PADDR  0x51800000
#define Dblog_SMMU_PADDR                                        0x51400000      /* 4MB */
// #define RESERVED_PADDR  0x51300000
#define Dblog_STM_PADDR                                         0x51100000      /* 2MB */
#define Dblog_LPCG_PADDR                                        0x510F0000      /* 64KB */
// #define RESERVED_PADDR  0x510B0000
#define Dblog_CTInotused_PADDR                                  0x510A0000      /* 64KB */
#define Dblog_IRQSTR_SCU2_PADDR                                 0x51090000      /* 64KB */
#define Dblog_IRQSTR_CM4_1_PADDR                                0x51080000      /* 64KB */
#define Dblog_IRQSTR_CM4_0_PADDR                                0x51070000      /* 64KB */
#define Dblog_IRQSTR_SCU_PADDR                                  0x51060000      /* 64KB */
// #define RESERVED_PADDR  0x51000000
/* Device: SCU (41) _________________________________________________________________________________ */
// #define RESERVED_PADDR  0x33640000
#define SCU_LPCG_LPI2C_PADDR                                    0x33630000      /* 64KB */
#define SCU_LPCG_LPUART_PADDR                                   0x33620000      /* 64KB */
#define SCU_LPCG_LPIT_PADDR                                     0x33610000      /* 64KB */
#define SCU_LPCG_TPM_PADDR                                      0x33600000      /* 64KB */
#define SCU_LPCG_MMCAU_HCLK_PADDR                               0x335F0000      /* 64KB */
#define SCU_LPCG_TCMC_HCLK_PADDR                                0x335E0000      /* 64KB */
#define SCU_LPCG_CPU_HCLK_PADDR                                 0x335D0000      /* 64KB */
// #define RESERVED_PADDR  0x33490000
#define SCU_MU1_A_PADDR                                         0x33480000      /* 64KB */
#define SCU_MU0_A3_PADDR                                        0x33470000      /* 64KB */
#define SCU_MU0_A2_PADDR                                        0x33460000      /* 64KB */
#define SCU_MU0_A1_PADDR                                        0x33450000      /* 64KB */
#define SCU_MU0_A0_PADDR                                        0x33440000      /* 64KB */
#define SCU_MU0_B_PADDR                                         0x33430000      /* 64KB */
#define SCU_WDOG_PADDR                                          0x33420000      /* 64KB */
#define SCU_BBS_SIM_PADDR                                       0x33410000      /* 64KB */
#define SCU_INTMUX_PADDR                                        0x33400000      /* 64KB */
// #define RESERVED_PADDR  0x33240000
#define SCU_LPI2C_PADDR                                         0x33230000      /* 64KB */
#define SCU_LPUART_PADDR                                        0x33220000      /* 64KB */
#define SCU_LPIT_PADDR                                          0x33210000      /* 64KB */
#define SCU_TPM_PADDR                                           0x33200000      /* 64KB */
// #define RESERVED_PADDR  0x331C0000
#define SCU_SEMA42_PADDR                                        0x331B0000      /* 64KB */
// #define RESERVED_PADDR  0x33100000
#define SCU_RGPIO_PADDR                                         0x330F0000      /* 64KB */
// #define RESERVED_PADDR  0x33000000
#define SCU_DebugBlock_PADDR                                    0x32400000      /* 12MB */
// #define RESERVED_PADDR  0x32070000
#define SCU_ROMCP_PADDR                                         0x32060000      /* 64KB */
// #define RESERVED_PADDR  0x32030000
#define SCU_SYSCNTCMP_PADDR                                     0x32020000      /* 64KB */
#define SCU_SYSCNTRD_PADDR                                      0x32010000      /* 64KB */
#define SCU_SYSCNTCTRL_PADDR                                    0x32000000      /* 64KB */
#define SCU_SecurityBlock_PADDR                                 0x31400000      /* 12MB */
// #define RESERVED_PADDR  0x31020000
#define SCU_TCMU_PADDR                                          0x31000000      /* 128KB */
#define SCU_TCML_PADDR                                          0x30FE0000      /* 128KB */
// #define RESERVED_PADDR  0x30C00000
// #define RESERVED_PADDR  0x30000000
/* Device: CM4_0 (35) _______________________________________________________________________________ */
// #define RESERVED_PADDR  0x37800000
// #define RESERVED_PADDR  0x37640000
#define CM4_0_LPCG_LPI2C_PADDR                                  0x37630000      /* 64KB */
#define CM4_0_LPCG_LPUART_PADDR                                 0x37620000      /* 64KB */
#define CM4_0_LPCG_LPIT_PADDR                                   0x37610000      /* 64KB */
#define CM4_0_LPCG_TPM_PADDR                                    0x37600000      /* 64KB */
#define CM4_0_LPCG_MMCAU_HCLK_PADDR                             0x375F0000      /* 64KB */
#define CM4_0_LPCG_TCMC_HCLK_PADDR                              0x375E0000      /* 64KB */
#define CM4_0_LPCG_CPU_HCLK_PADDR                               0x375D0000      /* 64KB */
// #define RESERVED_PADDR  0x37490000
#define CM4_0_MU1_A_PADDR                                       0x37480000      /* 64KB */
#define CM4_0_MU0_A3_PADDR                                      0x37470000      /* 64KB */
#define CM4_0_MU0_A2_PADDR                                      0x37460000      /* 64KB */
#define CM4_0_MU0_A1_PADDR                                      0x37450000      /* 64KB */
#define CM4_0_MU0_A0_PADDR                                      0x37440000      /* 64KB */
#define CM4_0_MU0_B_PADDR                                       0x37430000      /* 64KB */
#define CM4_0_WDOG_PADDR                                        0x37420000      /* 64KB */
#define CM4_0_BBS_SIM_PADDR                                     0x37410000      /* 64KB */
#define CM4_0_INTMUX_PADDR                                      0x37400000      /* 64KB */
// #define RESERVED_PADDR  0x37240000
#define CM4_0_LPI2C_PADDR                                       0x37230000      /* 64KB */
#define CM4_0_LPUART_PADDR                                      0x37220000      /* 64KB */
#define CM4_0_LPIT_PADDR                                        0x37210000      /* 64KB */
#define CM4_0_TPM_PADDR                                         0x37200000      /* 64KB */
// #define RESERVED_PADDR  0x371C0000
#define CM4_0_SEMA42_PADDR                                      0x371B0000      /* 64KB */
// #define RESERVED_PADDR  0x37100000
#define CM4_0_RGPIO_PADDR                                       0x370F0000      /* 64KB */
// #define RESERVED_PADDR  0x37000000
// #define RESERVED_PADDR  0x35400000
// #define RESERVED_PADDR  0x35020000
#define CM4_0_TCMU_PADDR                                        0x35000000      /* 128KB */
#define CM4_0_TCML_PADDR                                        0x34FE0000      /* 128KB */
// #define RESERVED_PADDR  0x34C00000
// #define RESERVED_PADDR  0x34000000






#endif /* __PLAT_MACHINE_DEVICES_H */
