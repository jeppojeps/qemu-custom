/*
 * STM32L45 SoC
 *
 * Copyright (c) 2014 Antonio Nappa <anappa@inf.uc3m.es>
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

#ifndef HW_ARM_STM32L45_SOC_H
#define HW_ARM_STM32L45_SOC_H

#include "hw/misc/stm32f4xx_syscfg.h"
#include "hw//misc/stm32f4xx_exti.h"
#include "hw//misc/stm32l45_rcc.h"
#include "hw//misc/stm32l45_gpio.h"
#include "hw/timer/stm32f2xx_timer.h"
#include "hw/char/stm32f2xx_usart.h"
#include "hw/adc/stm32f2xx_adc.h"
#include "hw/char/stm32f2xx_usart.h"
#include "hw/ssi/stm32f2xx_spi.h"
#include "qom/object.h"
#include "armv7m.h"
#include "hw/clock.h"
#include "hw/or-irq.h"
#include "hw/robot/robot.h"

#define TYPE_STM32L45_SOC "stm32l45-soc"
OBJECT_DECLARE_SIMPLE_TYPE(STM32L45State, STM32L45_SOC)

#define STM_NUM_USARTS 3
#define STM_NUM_TIMERS 4
#define STM_NUM_ADCS 1
#define STM_NUM_SPIS 3

#define FLASH_BASE_ADDRESS 0x08000000
#define FLASH_SIZE (512 * 1024)
#define SRAM_BASE_ADDRESS 0x20000000
#define SRAM_SIZE (128 * 1024)
#define CCM_BASE_ADDRESS 0x10000000
#define CCM_SIZE (32 * 1096)
#define CCM_ALIAS_ADDRESS 0x20020000

struct STM32L45State {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    char *cpu_type;

    ARMv7MState armv7m;

    STM32F4xxSyscfgState syscfg;
    STM32F4xxExtiState exti;
    STM32F2XXUsartState usart[STM_NUM_USARTS];
    STM32F2XXTimerState timer[STM_NUM_TIMERS];
    OrIRQState adc_irqs;
    STM32F2XXADCState adc[STM_NUM_ADCS];
    STM32F2XXSPIState spi[STM_NUM_SPIS];

    MemoryRegion ccm;
    MemoryRegion ccm_alias;
    MemoryRegion sram;
    MemoryRegion flash;
    MemoryRegion flash_alias;
    STM32L45RccState rcc;

    Clock *sysclk;
    Clock *refclk;
    STM32L45GPIOState gpio[9];
};

#endif
