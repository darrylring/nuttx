/****************************************************************************
 * arch/arm/src/stm32u3/hardware/stm32u3xx_adc.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32U3_HARDWARE_STM32U3XX_ADC_H
#define __ARCH_ARM_SRC_STM32U3_HARDWARE_STM32U3XX_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_ADC_ISR_OFFSET        0x0000  /* Interrupt and status */
#define STM32_ADC_IER_OFFSET        0x0004  /* Interrupt enable */
#define STM32_ADC_CR_OFFSET         0x0008  /* Control */
#define STM32_ADC_CFGR1_OFFSET      0x000c  /* Configuration 1 */
#define STM32_ADC_CFGR2_OFFSET      0x0010  /* Configuration 2 */
#define STM32_ADC_SMPR1_OFFSET      0x0014  /* Sample time 1 (channels 0-9) */
#define STM32_ADC_SMPR2_OFFSET      0x0018  /* Sample time 2 (channels 10-18) */
#define STM32_ADC_PCSEL_OFFSET      0x001c  /* Channel preselection */
#define STM32_ADC_SQR1_OFFSET       0x0030  /* Regular sequence 1 */
#define STM32_ADC_SQR2_OFFSET       0x0034  /* Regular sequence 2 */
#define STM32_ADC_SQR3_OFFSET       0x0038  /* Regular sequence 3 */
#define STM32_ADC_SQR4_OFFSET       0x003c  /* Regular sequence 4 */
#define STM32_ADC_DR_OFFSET         0x0040  /* Regular data */
#define STM32_ADC_JSQR_OFFSET       0x004c  /* Injected sequence */
#define STM32_ADC_OFCFGR1_OFFSET    0x0050  /* Offset 1 configuration */
#define STM32_ADC_OFCFGR2_OFFSET    0x0054  /* Offset 2 configuration */
#define STM32_ADC_OFCFGR3_OFFSET    0x0058  /* Offset 3 configuration */
#define STM32_ADC_OFCFGR4_OFFSET    0x005c  /* Offset 4 configuration */
#define STM32_ADC_OFR1_OFFSET       0x0060  /* Offset 1 */
#define STM32_ADC_OFR2_OFFSET       0x0064  /* Offset 2 */
#define STM32_ADC_OFR3_OFFSET       0x0068  /* Offset 3 */
#define STM32_ADC_OFR4_OFFSET       0x006c  /* Offset 4 */
#define STM32_ADC_GCOMP_OFFSET      0x0070  /* Gain compensation */
#define STM32_ADC_JDR1_OFFSET       0x0080  /* Injected data 1 */
#define STM32_ADC_JDR2_OFFSET       0x0084  /* Injected data 2 */
#define STM32_ADC_JDR3_OFFSET       0x0088  /* Injected data 3 */
#define STM32_ADC_JDR4_OFFSET       0x008c  /* Injected data 4 */
#define STM32_ADC_AWD2CR_OFFSET     0x00a0  /* Analog watchdog 2 channels */
#define STM32_ADC_AWD3CR_OFFSET     0x00a4  /* Analog watchdog 3 channels */
#define STM32_ADC_AWD1LTR_OFFSET    0x00a8  /* Analog watchdog 1 low threshold */
#define STM32_ADC_AWD1HTR_OFFSET    0x00ac  /* Analog watchdog 1 high threshold */
#define STM32_ADC_AWD2LTR_OFFSET    0x00b0  /* Analog watchdog 2 low threshold */
#define STM32_ADC_AWD2HTR_OFFSET    0x00b4  /* Analog watchdog 2 high threshold */
#define STM32_ADC_AWD3LTR_OFFSET    0x00b8  /* Analog watchdog 3 low threshold */
#define STM32_ADC_AWD3HTR_OFFSET    0x00bc  /* Analog watchdog 3 high threshold */
#define STM32_ADC_CALFACT_OFFSET    0x00c4  /* Calibration factor */
#define STM32_ADC_OR_OFFSET         0x00d0  /* Option */

/* Common registers, relative to STM32_ADC12_COMMON_BASE */

#define STM32_ADC_CSR_OFFSET        0x0000  /* Common status */
#define STM32_ADC_CCR_OFFSET        0x0008  /* Common control */
#define STM32_ADC_CDR_OFFSET        0x000c  /* Common data, dual mode */
#define STM32_ADC_CDR2_OFFSET       0x0010  /* Common data 2, dual mode */

/* Register Addresses *******************************************************/

#define STM32_ADC1_ISR              (STM32_ADC1_BASE + STM32_ADC_ISR_OFFSET)
#define STM32_ADC1_IER              (STM32_ADC1_BASE + STM32_ADC_IER_OFFSET)
#define STM32_ADC1_CR               (STM32_ADC1_BASE + STM32_ADC_CR_OFFSET)
#define STM32_ADC1_CFGR1            (STM32_ADC1_BASE + STM32_ADC_CFGR1_OFFSET)
#define STM32_ADC1_CFGR2            (STM32_ADC1_BASE + STM32_ADC_CFGR2_OFFSET)
#define STM32_ADC1_DR               (STM32_ADC1_BASE + STM32_ADC_DR_OFFSET)

#define STM32_ADC2_ISR              (STM32_ADC2_BASE + STM32_ADC_ISR_OFFSET)
#define STM32_ADC2_IER              (STM32_ADC2_BASE + STM32_ADC_IER_OFFSET)
#define STM32_ADC2_CR               (STM32_ADC2_BASE + STM32_ADC_CR_OFFSET)
#define STM32_ADC2_CFGR1            (STM32_ADC2_BASE + STM32_ADC_CFGR1_OFFSET)
#define STM32_ADC2_CFGR2            (STM32_ADC2_BASE + STM32_ADC_CFGR2_OFFSET)
#define STM32_ADC2_DR               (STM32_ADC2_BASE + STM32_ADC_DR_OFFSET)

#define STM32_ADC12_CSR             (STM32_ADC12_COMMON_BASE + STM32_ADC_CSR_OFFSET)
#define STM32_ADC12_CCR             (STM32_ADC12_COMMON_BASE + STM32_ADC_CCR_OFFSET)
#define STM32_ADC12_CDR             (STM32_ADC12_COMMON_BASE + STM32_ADC_CDR_OFFSET)
#define STM32_ADC12_CDR2            (STM32_ADC12_COMMON_BASE + STM32_ADC_CDR2_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* ADC interrupt and status register (ISR) and interrupt enable register
 * (IER).  The flags have the same position in both registers.
 */

#define ADC_INT_ADRDY               (1 << 0)   /* ADC ready */
#define ADC_INT_EOSMP               (1 << 1)   /* End of sampling */
#define ADC_INT_EOC                 (1 << 2)   /* End of regular conversion */
#define ADC_INT_EOS                 (1 << 3)   /* End of regular sequence */
#define ADC_INT_OVR                 (1 << 4)   /* Overrun */
#define ADC_INT_JEOC                (1 << 5)   /* End of injected conversion */
#define ADC_INT_JEOS                (1 << 6)   /* End of injected sequence */
#define ADC_INT_AWD1                (1 << 7)   /* Analog watchdog 1 */
#define ADC_INT_AWD2                (1 << 8)   /* Analog watchdog 2 */
#define ADC_INT_AWD3                (1 << 9)   /* Analog watchdog 3 */
#define ADC_INT_JQOVF               (1 << 10)  /* Injected queue overflow */
#define ADC_INT_LDORDY              (1 << 12)  /* Voltage regulator ready */

#define ADC_INT_MASK                (ADC_INT_ADRDY | ADC_INT_EOSMP |  \
                                     ADC_INT_EOC | ADC_INT_EOS |      \
                                     ADC_INT_OVR | ADC_INT_JEOC |     \
                                     ADC_INT_JEOS | ADC_INT_AWD1 |    \
                                     ADC_INT_AWD2 | ADC_INT_AWD3 |    \
                                     ADC_INT_JQOVF | ADC_INT_LDORDY)

/* ADC control register (CR) */

#define ADC_CR_ADEN                 (1 << 0)   /* Enable */
#define ADC_CR_ADDIS                (1 << 1)   /* Disable command */
#define ADC_CR_ADSTART              (1 << 2)   /* Start regular conversion */
#define ADC_CR_JADSTART             (1 << 3)   /* Start injected conversion */
#define ADC_CR_ADSTP                (1 << 4)   /* Stop regular conversion */
#define ADC_CR_JADSTP               (1 << 5)   /* Stop injected conversion */
#define ADC_CR_ADVREGEN             (1 << 28)  /* Voltage regulator enable */
#define ADC_CR_DEEPPWD              (1 << 29)  /* Deep power down */
#define ADC_CR_ADCAL                (1 << 31)  /* Calibration */

/* ADC configuration register 1 (CFGR1) */

#define ADC_CFGR1_DMNGT_SHIFT       (0)        /* Bits 0-1: Data management */
#define ADC_CFGR1_DMNGT_MASK        (3 << ADC_CFGR1_DMNGT_SHIFT)
#  define ADC_CFGR1_DMNGT_DR        (0 << ADC_CFGR1_DMNGT_SHIFT) /* DR only */
#  define ADC_CFGR1_DMNGT_DMA_ONCE  (1 << ADC_CFGR1_DMNGT_SHIFT) /* DMA one shot */
#  define ADC_CFGR1_DMNGT_ADF       (2 << ADC_CFGR1_DMNGT_SHIFT) /* ADF */
#  define ADC_CFGR1_DMNGT_DMA_CIRC  (3 << ADC_CFGR1_DMNGT_SHIFT) /* DMA circular */

#define ADC_CFGR1_RES_SHIFT         (2)        /* Bits 2-3: Resolution */
#define ADC_CFGR1_RES_MASK          (3 << ADC_CFGR1_RES_SHIFT)
#  define ADC_CFGR1_RES_12BIT       (0 << ADC_CFGR1_RES_SHIFT)
#  define ADC_CFGR1_RES_10BIT       (1 << ADC_CFGR1_RES_SHIFT)
#  define ADC_CFGR1_RES_8BIT        (2 << ADC_CFGR1_RES_SHIFT)
#  define ADC_CFGR1_RES_6BIT        (3 << ADC_CFGR1_RES_SHIFT)

#define ADC_CFGR1_EXTSEL_SHIFT      (5)        /* Bits 5-9: Regular trigger */
#define ADC_CFGR1_EXTSEL_MASK       (31 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC_CFGR1_EXTSEL(n)       ((n) << ADC_CFGR1_EXTSEL_SHIFT)

#define ADC_CFGR1_EXTEN_SHIFT       (10)       /* Bits 10-11: Trigger edge */
#define ADC_CFGR1_EXTEN_MASK        (3 << ADC_CFGR1_EXTEN_SHIFT)
#  define ADC_CFGR1_EXTEN_NONE      (0 << ADC_CFGR1_EXTEN_SHIFT)
#  define ADC_CFGR1_EXTEN_RISING    (1 << ADC_CFGR1_EXTEN_SHIFT)
#  define ADC_CFGR1_EXTEN_FALLING   (2 << ADC_CFGR1_EXTEN_SHIFT)
#  define ADC_CFGR1_EXTEN_BOTH      (3 << ADC_CFGR1_EXTEN_SHIFT)

#define ADC_CFGR1_OVRMOD            (1 << 12)  /* Overwrite DR on overrun */
#define ADC_CFGR1_CONT              (1 << 13)  /* Continuous conversion */
#define ADC_CFGR1_AUTDLY            (1 << 14)  /* Delayed conversion mode */
#define ADC_CFGR1_DISCEN            (1 << 16)  /* Discontinuous mode */
#define ADC_CFGR1_DISCNUM_SHIFT     (17)       /* Bits 17-19: Discontinuous count */
#define ADC_CFGR1_DISCNUM_MASK      (7 << ADC_CFGR1_DISCNUM_SHIFT)
#define ADC_CFGR1_JDISCEN           (1 << 20)  /* Injected discontinuous */
#define ADC_CFGR1_JQM               (1 << 21)  /* Injected queue mode */
#define ADC_CFGR1_AWD1SGL           (1 << 22)  /* AWD1 on a single channel */
#define ADC_CFGR1_AWD1EN            (1 << 23)  /* AWD1 on regular channels */
#define ADC_CFGR1_JAWD1EN           (1 << 24)  /* AWD1 on injected channels */
#define ADC_CFGR1_JAUTO             (1 << 25)  /* Automatic injected group */
#define ADC_CFGR1_AWD1CH_SHIFT      (26)       /* Bits 26-30: AWD1 channel */
#define ADC_CFGR1_AWD1CH_MASK       (31 << ADC_CFGR1_AWD1CH_SHIFT)
#  define ADC_CFGR1_AWD1CH(n)       ((n) << ADC_CFGR1_AWD1CH_SHIFT)
#define ADC_CFGR1_JQDIS             (1 << 31)  /* Injected queue disable */

/* ADC configuration register 2 (CFGR2) */

#define ADC_CFGR2_ROVSE             (1 << 0)   /* Regular oversampling enable */
#define ADC_CFGR2_OVSS_SHIFT        (5)        /* Bits 5-8: Oversampling shift */
#define ADC_CFGR2_OVSS_MASK         (15 << ADC_CFGR2_OVSS_SHIFT)
#  define ADC_CFGR2_OVSS(n)         ((n) << ADC_CFGR2_OVSS_SHIFT)
#define ADC_CFGR2_TROVS             (1 << 9)   /* Triggered oversampling */
#define ADC_CFGR2_ROVSM             (1 << 10)  /* Resumed oversampling mode */
#define ADC_CFGR2_BULB              (1 << 13)  /* Bulb sampling mode */
#define ADC_CFGR2_SWTRIG            (1 << 14)  /* Sampling time software trigger */
#define ADC_CFGR2_SMPTRIG           (1 << 15)  /* Sampling time trigger mode */
#define ADC_CFGR2_OVSR_SHIFT        (16)       /* Bits 16-25: Ratio is OVSR + 1 */
#define ADC_CFGR2_OVSR_MASK         (0x3ff << ADC_CFGR2_OVSR_SHIFT)
#  define ADC_CFGR2_OVSR(n)         ((n) << ADC_CFGR2_OVSR_SHIFT)
#define ADC_CFGR2_LSHIFT_SHIFT      (28)       /* Bits 28-31: Left shift */
#define ADC_CFGR2_LSHIFT_MASK       (15 << ADC_CFGR2_LSHIFT_SHIFT)
#  define ADC_CFGR2_LSHIFT(n)       ((n) << ADC_CFGR2_LSHIFT_SHIFT)

/* ADC sample time registers (SMPR1, SMPR2): 3 bits per channel */

#define ADC_SMPR_1p5                0          /* 1.5 ADC clock cycles */
#define ADC_SMPR_2p5                1
#define ADC_SMPR_6p5                2
#define ADC_SMPR_11p5               3
#define ADC_SMPR_23p5               4
#define ADC_SMPR_46p5               5
#define ADC_SMPR_246p5              6
#define ADC_SMPR_1499p5             7

/* ch 0-9: SMPR1, ch 10-18: SMPR2 */

#define ADC_SMPR_SMP_SHIFT(ch)      (((ch) % 10) * 3)
#define ADC_SMPR_SMP_MASK(ch)       (7 << ADC_SMPR_SMP_SHIFT(ch))

#define ADC_SMPR1_CHANNELS          10         /* Channels 0-9 in SMPR1 */

/* ADC channel preselection register (PCSEL): one bit per channel 0-18 */

#define ADC_PCSEL_CH(ch)            (1 << (ch))
#define ADC_PCSEL_MASK              0x0007ffff

/* ADC regular sequence registers (SQR1-SQR4) */

#define ADC_SQR_CHANNEL_MASK        0x1f
#define ADC_SQ_OFFSET               6          /* Gap between SQ fields */

#define ADC_SQR1_L_SHIFT            (0)        /* Bits 0-3: Sequence length - 1 */
#define ADC_SQR1_L_MASK             (15 << ADC_SQR1_L_SHIFT)
#define ADC_SQR1_FIRST              1          /* SQ1-SQ4, SQ1 at bit 6 */
#define ADC_SQR1_LAST               4
#define ADC_SQR1_SQ_OFFSET          6
#define ADC_SQR2_FIRST              5          /* SQ5-SQ9, SQ5 at bit 0 */
#define ADC_SQR2_LAST               9
#define ADC_SQR2_SQ_OFFSET          0
#define ADC_SQR3_FIRST              10         /* SQ10-SQ14 */
#define ADC_SQR3_LAST               14
#define ADC_SQR3_SQ_OFFSET          0
#define ADC_SQR4_FIRST              15         /* SQ15-SQ16 */
#define ADC_SQR4_LAST               16
#define ADC_SQR4_SQ_OFFSET          0

/* ADC data register (DR) */

#define ADC_DR_MASK                 0xffff     /* Right aligned result */

/* ADC offset configuration register (OFCFGRy) */

#define ADC_OFCFGR_POSOFF           (1 << 24)  /* Positive offset */
#define ADC_OFCFGR_USAT             (1 << 25)  /* Unsigned saturation */
#define ADC_OFCFGR_SSAT             (1 << 26)  /* Signed saturation */
#define ADC_OFCFGR_CH_SHIFT         (27)       /* Bits 27-31: Offset channel */
#define ADC_OFCFGR_CH_MASK          (31 << ADC_OFCFGR_CH_SHIFT)

/* ADC offset register (OFRy) */

#define ADC_OFR_OFFSET_MASK         0x003fffff /* Bits 0-21 */

/* ADC gain compensation register (GCOMP) */

#define ADC_GCOMP_COEFF_MASK        0x3fff     /* Bits 0-13 */
#define ADC_GCOMP_GCOMP             (1 << 31)  /* Gain compensation enable */

/* ADC analog watchdog 2 and 3 channel registers (AWD2CR, AWD3CR) */

#define ADC_AWDCR_CH(ch)            (1 << (ch))

/* ADC analog watchdog thresholds (AWDxLTR, AWDxHTR) */

#define ADC_AWD_THR_MASK            0x007fffff /* Bits 0-22 */
#define ADC_AWD1HTR_FILT_SHIFT      (29)       /* Bits 29-31: AWD1 filtering */
#define ADC_AWD1HTR_FILT_MASK       (7 << ADC_AWD1HTR_FILT_SHIFT)
#  define ADC_AWD1HTR_FILT(n)       ((n) << ADC_AWD1HTR_FILT_SHIFT)

/* ADC calibration factor register (CALFACT) */

#define ADC_CALFACT_MASK            0x7f       /* Bits 0-6 */

/* ADC common control register (CCR) */

#define ADC_CCR_DUAL_SHIFT          (0)        /* Bits 0-4: Dual mode */
#define ADC_CCR_DUAL_MASK           (31 << ADC_CCR_DUAL_SHIFT)
#define ADC_CCR_DELAY_SHIFT         (8)        /* Bits 8-11: Sampling delay */
#define ADC_CCR_DELAY_MASK          (15 << ADC_CCR_DELAY_SHIFT)
#define ADC_CCR_DAMDF_SHIFT         (14)       /* Bits 14-15: Dual data format */
#define ADC_CCR_DAMDF_MASK          (3 << ADC_CCR_DAMDF_SHIFT)
#define ADC_CCR_VREFEN              (1 << 22)  /* VREFINT enable */
#define ADC_CCR_TSEN                (1 << 23)  /* Temperature sensor enable */
#define ADC_CCR_VBATEN              (1 << 24)  /* VBAT/4 enable */

/* Internal channel numbers, the same on ADC1 and ADC2 */

#define ADC_CH_VREFINT              0
#define ADC_CH_DAC1_OUT1            14         /* ADC2 only */
#define ADC_CH_DAC1_OUT2            15         /* ADC2 only */
#define ADC_CH_VBAT                 16
#define ADC_CH_VSENSE               17
#define ADC_CH_VCORE                18

#define ADC_NCHANNELS               19

#endif /* __ARCH_ARM_SRC_STM32U3_HARDWARE_STM32U3XX_ADC_H */
