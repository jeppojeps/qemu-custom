/*
 * STM32F405 SoC
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
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include <inttypes.h>
#include "exec/address-spaces.h"
#include "sysemu/sysemu.h"
#include "hw/robot/robot.h"
#include "hw/arm/stm32l45_soc.h"
#include "hw/qdev-clock.h"
#include "hw/misc/unimp.h"

/* Define proper Flash memory addresses */
#define FLASH_MEMORY_BASE     0x08000000  // Flash memory for code
#define FLASH_INTERFACE_BASE  0x40022000  // Flash interface registers


#define SYSCFG_ADD                     0x40010000
static const uint32_t usart_addr[] = { 0x40013800, 0x40014400, 0x40014800,
                                       0x40014C00};
static const uint32_t timer_addr[] = {
    0x40012C00,  // TIM1
    0x40000000,  // TIM2
    0x40000400,  // TIM3
    0x40000800,  // TIM4
    0x40000C00,  // TIM5
    0x40001000,  // TIM6
    0x40001400,  // TIM7
    0x40001800,  // TIM8
    0x40002000,  // TIM9
    0x40002400,  // TIM10
    0x40002800,  // TIM11
    0x40002C00   // TIM12
};
static const uint32_t adc_addr[] = { 0x50040000 };
static const uint32_t spi_addr[] =   { 0x40013000, 0x40003800, 0x40003C00 };
#define EXTI_ADDR                      0x40010400


#define SYSCFG_IRQ 71  
static const int usart_irq[] = { 37, 38, 39}; // USART1, USART2, USART3
static const int timer_irq[] = { 28, 29, 54, 55 }; // TIM1, TIM2, TIM3
#define ADC_IRQ 18  // ADC1_2
static const int spi_irq[] =   { 35, 36, 51}; // SPI1, SPI2, SPI3
static const int exti_irq[] =  { 6, 7, 8, 9, 10}; // EXTI0, EXTI1, EXTI2, EXTI3, EXTI4

static void stm32l45_soc_initfn(Object *obj)
{
    STM32L45State *s = STM32L45_SOC(obj);
    int i;

    /* Initialize NVIC before ARMV7M */
    object_initialize_child(obj, "nvic", &s->nvic, TYPE_STM32L45_NVIC);

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);

    object_initialize_child(obj, "syscfg", &s->syscfg, TYPE_STM32F4XX_SYSCFG);

    /* Initialize RCC */
    object_initialize_child(obj, "rcc", &s->rcc, TYPE_STM32L45_RCC);

    /* Initialize GPIO */
    for (i = 0; i < 8; i++) {  // GPIOA through GPIOH
        char gpio_name[16];
        snprintf(gpio_name, sizeof(gpio_name), "gpio[%c]", 'a' + i);
        object_initialize_child(obj, gpio_name, &s->gpio[i], TYPE_STM32L45_GPIO);
    }


    for (i = 0; i < STM_NUM_USARTS; i++) {
        object_initialize_child(obj, "usart[*]", &s->usart[i],
                                TYPE_STM32F2XX_USART);
    }

    for (i = 0; i < STM_NUM_TIMERS; i++) {
        object_initialize_child(obj, "timer[*]", &s->timer[i],
                                TYPE_STM32F2XX_TIMER);
    }

    for (i = 0; i < STM_NUM_ADCS; i++) {
        object_initialize_child(obj, "adc[*]", &s->adc[i], TYPE_STM32F2XX_ADC);
    }

    for (i = 0; i < STM_NUM_SPIS; i++) {
        object_initialize_child(obj, "spi[*]", &s->spi[i], TYPE_STM32F2XX_SPI);
    }

    object_initialize_child(obj, "exti", &s->exti, TYPE_STM32F4XX_EXTI);

    //s->sysclk = qdev_init_clock_in(DEVICE(s), "sysclk", NULL, NULL, 0);
    //s->refclk = qdev_init_clock_in(DEVICE(s), "refclk", NULL, NULL, 0);
}




static void stm32l45_soc_realize(DeviceState *dev_soc, Error **errp)
{
    STM32L45State *s = STM32L45_SOC(dev_soc);
    MemoryRegion *system_memory = get_system_memory();
    DeviceState *dev, *armv7m;
    DeviceState *rcc_dev;
    SysBusDevice *busdev;
    Error *err = NULL;
    int i;

    /* Realize NVIC first */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->nvic), errp)) {
        return;
    }

    /* Map NVIC at its standard Cortex-M address */
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->nvic), 0, 0xE000E000);

    /* Initialize RCC first as it provides clocks */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->rcc), errp)) {
        return;
    }
    rcc_dev = DEVICE(&s->rcc);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->rcc), 0, 0x40021000);
    qemu_log_mask(LOG_UNIMP, "STM32L4: RCC initialized and mapped to 0x40021000\n");

    // Enable GPIO clocks (needed for peripherals)
    uint32_t initial_ahbenr =
        0x1 << 0 |  // GPIOAEN
        0x1 << 1 |  // GPIOBEN
        0x1 << 2 |  // GPIOCEN
        0x1 << 3 |  // GPIODEN
        0x1 << 4;   // GPIOEEN

   /* Access RCC registers through memory operations */
   MemoryRegion *rcc_mr = &s->rcc.mmio;
   if (rcc_mr && rcc_mr->ops) {
       uint32_t verify_before = 0;
       address_space_read(&address_space_memory,
                         0x40021000 + 0x4C,
                         MEMTXATTRS_UNSPECIFIED,
                         (uint8_t *)&verify_before,
                         sizeof(verify_before));

       qemu_log_mask(LOG_UNIMP, "STM32L4: RCC AHB2ENR before write = 0x%08x\n",
                    verify_before);

       /* Write to AHB2ENR */
       address_space_write(&address_space_memory,
                         0x40021000 + 0x4C,
                         MEMTXATTRS_UNSPECIFIED,
                         (uint8_t *)&initial_ahbenr,
                         sizeof(initial_ahbenr));

       /* Read back for verification */
       uint32_t verify_after = 0;
       address_space_read(&address_space_memory,
                         0x40021000 + 0x4C,
                         MEMTXATTRS_UNSPECIFIED,
                         (uint8_t *)&verify_after,
                         sizeof(verify_after));

       qemu_log_mask(LOG_UNIMP, "STM32L4: RCC AHB2ENR after write: expected=0x%08x actual=0x%08x\n",
                    initial_ahbenr, verify_after);
   }
    //shadow write

    /* Only proceed if we have the memory region */
    if (rcc_mr && rcc_mr->ops) {
        /* Write to AHB2ENR using address_space_write */
        address_space_write(&address_space_memory,
                          0x40021000 + 0x4C,  // RCC base + AHB2ENR offset
                          MEMTXATTRS_UNSPECIFIED,
                          (uint8_t *)&initial_ahbenr,
                          sizeof(initial_ahbenr));

        /* Read back for verification */
        uint32_t verify = 0;
        address_space_read(&address_space_memory,
                          0x40021000 + 0x4C,
                          MEMTXATTRS_UNSPECIFIED,
                          (uint8_t *)&verify,
                          sizeof(verify));

        qemu_log_mask(LOG_UNIMP, "STM32L4: Initial RCC AHB2ENR = 0x%08x\n",
                     verify);
    }


    /*
    memory_region_init_rom(&s->flash, OBJECT(dev_soc), "STM32L45.flash",
                           FLASH_SIZE, &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
    memory_region_init_alias(&s->flash_alias, OBJECT(dev_soc),
                             "STM32L45.flash.alias", &s->flash, 0,
                             FLASH_SIZE);

    memory_region_add_subregion(system_memory, FLASH_BASE_ADDRESS, &s->flash);
    memory_region_add_subregion(system_memory, 0, &s->flash_alias);
    */

    /* Initialize Flash Memory (for code) */
    memory_region_init_rom(&s->flash, OBJECT(dev_soc), "STM32L45.flash",
                          FLASH_SIZE, &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
    memory_region_init_alias(&s->flash_alias, OBJECT(dev_soc),
                            "STM32L45.flash.alias", &s->flash, 0,
                            FLASH_SIZE);

    /* Map Flash memory to correct addresses */
    memory_region_add_subregion(system_memory, FLASH_MEMORY_BASE, &s->flash);
    memory_region_add_subregion(system_memory, 0, &s->flash_alias);

    /* Initialize Flash Interface Registers */
    MemoryRegion *flash_regs = g_new(MemoryRegion, 1);
    memory_region_init_ram_nomigrate(flash_regs, OBJECT(dev_soc),
                                   "stm32l4.flash_regs", 0x400,
                                   &error_fatal);
    memory_region_add_subregion(system_memory, FLASH_INTERFACE_BASE,
                               flash_regs);

    memory_region_init_ram(&s->sram, NULL, "STM32L45.sram", SRAM_SIZE,
                           &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
    memory_region_add_subregion(system_memory, SRAM_BASE_ADDRESS, &s->sram);

    memory_region_init_ram(&s->ccm, NULL, "STM32L45.ccm", CCM_SIZE,
                           &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
    memory_region_add_subregion(system_memory, CCM_BASE_ADDRESS, &s->ccm);

    /* Init CCM alias region */
    memory_region_init_alias(&s->ccm_alias, NULL, "STM32L45.ccm.alias", &s->ccm, 0, CCM_SIZE);
    memory_region_add_subregion(system_memory, CCM_ALIAS_ADDRESS, &s->ccm_alias);

    armv7m = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(armv7m, "num-irq", 75);
    qdev_prop_set_string(armv7m, "cpu-type", s->cpu_type);
    qdev_prop_set_bit(armv7m, "enable-bitband", true);

    /* Connect RCC clock outputs to CPU clocks */
    qdev_connect_clock_in(armv7m, "cpuclk",
                         qdev_get_clock_out(rcc_dev, "sysclk"));
    qdev_connect_clock_in(armv7m, "refclk",
                         qdev_get_clock_out(rcc_dev, "hclk"));

    /* Connect NVIC to ARMV7M */
    object_property_set_link(OBJECT(&s->armv7m), "nvic", 
                           OBJECT(&s->nvic), &error_abort);

    object_property_set_link(OBJECT(&s->armv7m), "memory",
                             OBJECT(system_memory), &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), errp)) {
        return;
    }

    /* System configuration controller */
    dev = DEVICE(&s->syscfg);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->syscfg), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, SYSCFG_ADD);
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(DEVICE(&s->nvic), SYSCFG_IRQ));


    /* GPIO devices */
    for (i = 0; i < 8; i++) {
        dev = DEVICE(&s->gpio[i]);
        qdev_prop_set_uint8(dev, "port-id", i);  // Set port ID
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->gpio[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        uint32_t gpio_addr = 0x48000000 + (i * 0x400);
        sysbus_mmio_map(busdev, 0, gpio_addr);
        qemu_log_mask(LOG_UNIMP, "STM32L4: Initialized GPIO%c at 0x%08x\n",
                     'A' + i, gpio_addr);
    }

    /* Attach UART (uses USART registers) and USART controllers */
    for (i = 0; i < STM_NUM_USARTS; i++) {
        dev = DEVICE(&(s->usart[i]));
        qdev_prop_set_chr(dev, "chardev", serial_hd(i));
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->usart[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, usart_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(DEVICE(&s->nvic), usart_irq[i]));
    }

    /* Timer 2 to 5 */
    for (i = 0; i < STM_NUM_TIMERS; i++) {
        dev = DEVICE(&(s->timer[i]));
        qdev_prop_set_uint64(dev, "clock-frequency", 1000000000);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->timer[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, timer_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(DEVICE(&s->nvic), timer_irq[i]));
    }

    /* ADC device, the IRQs are ORed together */
    if (!object_initialize_child_with_props(OBJECT(s), "adc-orirq",
                                            &s->adc_irqs, sizeof(s->adc_irqs),
                                            TYPE_OR_IRQ, errp, NULL)) {
        return;
    }
    object_property_set_int(OBJECT(&s->adc_irqs), "num-lines", STM_NUM_ADCS,
                            &error_abort);
    if (!qdev_realize(DEVICE(&s->adc_irqs), NULL, errp)) {
        return;
    }
    qdev_connect_gpio_out(DEVICE(&s->adc_irqs), 0,
                          qdev_get_gpio_in(DEVICE(&s->nvic), ADC_IRQ));

    for (i = 0; i < STM_NUM_ADCS; i++) {
        dev = DEVICE(&(s->adc[i]));
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->adc[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, adc_addr[i]);
        sysbus_connect_irq(busdev, 0,
                           qdev_get_gpio_in(DEVICE(&s->adc_irqs), i));
    }

    /* SPI devices */
    for (i = 0; i < STM_NUM_SPIS; i++) {
        dev = DEVICE(&(s->spi[i]));
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->spi[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, spi_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(DEVICE(&s->nvic), spi_irq[i]));
    }

    /* EXTI device */
    dev = DEVICE(&s->exti);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->exti), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, EXTI_ADDR);
    for (i = 0; i < 5; i++) {
        sysbus_connect_irq(busdev, i, qdev_get_gpio_in(DEVICE(&s->nvic), exti_irq[i]));
    }
    for (i = 0; i < 14; i++) {
        qdev_connect_gpio_out(DEVICE(&s->syscfg), i, qdev_get_gpio_in(dev, i));
    }

    create_unimplemented_device("timer[7]",    0x40001400, 0x400);
    create_unimplemented_device("timer[12]",   0x40001800, 0x400);
    create_unimplemented_device("timer[6]",    0x40001000, 0x400);
    create_unimplemented_device("timer[13]",   0x40001C00, 0x400);
    create_unimplemented_device("timer[14]",   0x40002000, 0x400);
    create_unimplemented_device("RTC and BKP", 0x40002800, 0x400);
    create_unimplemented_device("WWDG",        0x40002C00, 0x400);
    create_unimplemented_device("IWDG",        0x40003000, 0x400);
    create_unimplemented_device("I2S2ext",     0x40003000, 0x400);
    create_unimplemented_device("I2S3ext",     0x40004000, 0x400);
    create_unimplemented_device("I2C1",        0x40005400, 0x400);
    create_unimplemented_device("I2C2",        0x40005800, 0x400);
    create_unimplemented_device("I2C3",        0x40005C00, 0x400);
    create_unimplemented_device("CAN1",        0x40006400, 0x400);
    create_unimplemented_device("CAN2",        0x40006800, 0x400);
    create_unimplemented_device("PWR",         0x40007000, 0x400);
    create_unimplemented_device("DAC",         0x40007400, 0x400);
    create_unimplemented_device("timer[1]",    0x40010000, 0x400);
    create_unimplemented_device("timer[8]",    0x40010400, 0x400);
    create_unimplemented_device("SDIO",        0x40012C00, 0x400);
    create_unimplemented_device("timer[9]",    0x40014000, 0x400);
    create_unimplemented_device("timer[10]",   0x40014400, 0x400);
    create_unimplemented_device("timer[11]",   0x40014800, 0x400);
    create_unimplemented_device("CRC",         0x40023000, 0x400);
    create_unimplemented_device("Flash Int",   0x40023C00, 0x400);
    create_unimplemented_device("BKPSRAM",     0x40024000, 0x400);
    create_unimplemented_device("DMA1",        0x40026000, 0x400);
    create_unimplemented_device("DMA2",        0x40026400, 0x400);
    create_unimplemented_device("Ethernet",    0x40028000, 0x1400);
    create_unimplemented_device("USB OTG HS",  0x40040000, 0x30000);
    create_unimplemented_device("DCMI",        0x50050000, 0x400);
    create_unimplemented_device("RNG",         0x50060800, 0x400);
    rb_create(system_memory, 0x50000000);
}

static Property stm32l45_soc_properties[] = {
    DEFINE_PROP_STRING("cpu-type", STM32L45State, cpu_type),
    DEFINE_PROP_END_OF_LIST(),
};

static void stm32l45_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32l45_soc_realize;
    device_class_set_props(dc, stm32l45_soc_properties);
    /* No vmstate or reset required: device has no internal state */
}

static const TypeInfo stm32l45_soc_info = {
    .name          = TYPE_STM32L45_SOC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32L45State),
    .instance_init = stm32l45_soc_initfn,
    .class_init    = stm32l45_soc_class_init,
};

static void stm32l45_soc_types(void)
{
    type_register_static(&stm32l45_soc_info);
}

type_init(stm32l45_soc_types)
