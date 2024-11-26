/*
 * STM32L45 RCC (Reset and clock control)
 *
 * Copyright (c) 2024 Antonio Nappa <anappa@inf.uc3m.es>
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
// stm32l45_rcc.h
#ifndef HW_ARM_STM32L45_RCC_H
#define HW_ARM_STM32L45_RCC_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "hw/registerfields.h"
#include "hw/clock.h"

#define TYPE_STM32L45_RCC "stm32l45-rcc"
OBJECT_DECLARE_SIMPLE_TYPE(STM32L45RccState, STM32L45_RCC)


static const uint32_t msi_frequencies[] = {
    100000,   // Range 0
    200000,   // Range 1
    400000,   // Range 2
    800000,   // Range 3
    1000000,  // Range 4
    2000000,  // Range 5
    4000000,  // Range 6  <- Your selected range
    8000000,  // Range 7
    16000000, // Range 8
    24000000, // Range 9
    32000000, // Range 10
    48000000  // Range 11
};

struct STM32L45RccState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;

    /* Registers */
    uint32_t cr;
    uint32_t cfgr;
    uint32_t cir;
    uint32_t apb2rstr;
    uint32_t apb1rstr;
    uint32_t ahbenr;
    uint32_t apb2enr;
    uint32_t apb1enr;
    uint32_t ahb2enr;
    uint32_t bdcr;
    uint32_t csr;
    uint32_t pllcfgr;    // Add this
    uint32_t ccipr;
    uint32_t pllsai1cfgr;

    /* Configuration */
    uint32_t hse_freq;

    /* Clock outputs */
    Clock *sysclk;
    Clock *hclk;
    Clock *pclk1;
    Clock *pclk2;
    bool gpio_clk_enabled[5]; 
};

#endif /* HW_ARM_STM32L45_RCC_H */
