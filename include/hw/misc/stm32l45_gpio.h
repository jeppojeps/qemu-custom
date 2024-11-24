#ifndef HW_STM32L45_GPIO_H
#define HW_STM32L45_GPIO_H

#include "hw/sysbus.h"
#include "qemu/log.h"

#define TYPE_STM32L45_GPIO "stm32l45-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(STM32L45GPIOState, STM32L45_GPIO)

struct STM32L45GPIOState {
    /* <-- SysBusDevice parent --> */
    SysBusDevice parent_obj;

    /* Memory region for this peripheral */
    MemoryRegion mmio;
    
    /* GPIO registers */
    uint32_t MODER;     /* Mode register */
    uint32_t OTYPER;    /* Output type register */
    uint32_t OSPEEDR;   /* Output speed register */
    uint32_t PUPDR;     /* Pull-up/pull-down register */
    uint32_t IDR;       /* Input data register */
    uint32_t ODR;       /* Output data register */
    uint32_t BSRR;      /* Bit set/reset register */
    uint32_t LCKR;      /* Configuration lock register */
    uint32_t AFR[2];    /* Alternate function registers */
    
    /* GPIO port identifier (A=0, B=1, etc) */
    uint8_t port_id;
};

#endif
