/*
 * STM32L4 SysTick Timer Header
 *
 * Copyright (c) 2024 QEMU Contributors
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef HW_ARM_STM32L4_SYSTICK_H
#define HW_ARM_STM32L4_SYSTICK_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_STM32L4_SYSTICK "stm32l4-systick"
OBJECT_DECLARE_SIMPLE_TYPE(STM32L4SysTickState, STM32L4_SYSTICK)

struct STM32L4SysTickState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    QEMUTimer *timer;
    qemu_irq irq;

    /* Registers */
    uint32_t control;     /* SYST_CSR value */
    uint32_t reload;      /* SYST_RVR value */
    uint32_t current;     /* SYST_CVR value */
    uint32_t calibration; /* SYST_CALIB value */
    
    /* Internal state */
    uint64_t tick_start_time;
    uint32_t freq_hz;
};

/* Register offsets */
#define SYSTICK_CSR    0x00  /* Control and Status Register */
#define SYSTICK_RVR    0x04  /* Reload Value Register */
#define SYSTICK_CVR    0x08  /* Current Value Register */
#define SYSTICK_CALIB  0x0C  /* Calibration Register */

/* CSR bit definitions */
#define SYSTICK_CSR_ENABLE    (1 << 0)   /* Counter enable */
#define SYSTICK_CSR_TICKINT   (1 << 1)   /* Enable SysTick exception */
#define SYSTICK_CSR_CLKSOURCE (1 << 2)   /* Clock source selection */
#define SYSTICK_CSR_COUNTFLAG (1 << 16)  /* Timer counted to zero */

/* Mask for the readable bits in CSR */
#define SYSTICK_CSR_MASK      0x00010007

/* Maximum value for the reload register */
#define SYSTICK_RVR_MASK      0x00FFFFFF

#endif /* HW_ARM_STM32L4_SYSTICK_H */
