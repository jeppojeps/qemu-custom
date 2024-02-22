/*
 * ST STM32_NUCLEO64 machine
 *
 * Copyright (c) 2023 Antonio Nappa <anappa@inf.uc3m.es>
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

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-clock.h"
#include "qemu/error-report.h"
//#include "hw/arm/stm32f100_soc.h"
#include "hw/arm/stm32l45_soc.h"
#include "hw/arm/boot.h"

/* stm32vldiscovery implementation is derived from netduinoplus2 */

static void stm32nucleo64_init(MachineState *machine)
{
    DeviceState *dev;
    //Object *rcc;

    dev = qdev_new(TYPE_STM32L45_SOC);
    qdev_prop_set_string(dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4"));
    
    
    /* Get RCC object and set HSE frequency */
    //rcc = object_resolve_path_component(OBJECT(dev), "rcc");
    //if (rcc) {
    //    object_property_set_int(rcc, "hse-freq", 8000000, &error_abort);
    //}
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    armv7m_load_kernel(ARM_CPU(first_cpu),
                      machine->kernel_filename,
                      0, FLASH_SIZE);
}


static void stm32nucleo64_machine_init(MachineClass *mc)
{
    mc->desc = "ST STM32NUCLEO64 (Cortex-M4)";
    mc->init = stm32nucleo64_init;
}

DEFINE_MACHINE("stm32nucleo64", stm32nucleo64_machine_init)
