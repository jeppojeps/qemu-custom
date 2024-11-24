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

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/misc/stm32l45_rcc.h"
#include "migration/vmstate.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-clock.h"
#include "hw/registerfields.h"

/* Clock frequencies */
#define HSI_FREQ 16000000ULL  /* 16 MHz internal oscillator */
#define LSI_FREQ 32000ULL     /* 32 kHz internal oscillator */
#define HSE_DEFAULT_FREQ 8000000ULL /* 8 MHz external oscillator default */
#define LSE_FREQ 32768ULL     /* 32.768 kHz external oscillator */
#define MSI_FREQ_BASE 100000ULL  /* 100 KHz base frequency */

/* Register field definitions */
REG32(CR, 0x00)
    FIELD(CR, HSION, 0, 1)
    FIELD(CR, HSIRDY, 1, 1)
    FIELD(CR, MSIRANGE, 4, 4)  /* MSI clock ranges */
    FIELD(CR, MSION, 8, 1)
    FIELD(CR, MSIRDY, 9, 1)
    FIELD(CR, HSEON, 16, 1)
    FIELD(CR, HSERDY, 17, 1)
    FIELD(CR, PLLON, 24, 1)
    FIELD(CR, PLLRDY, 25, 1)

REG32(CFGR, 0x04)
    FIELD(CFGR, SW, 0, 2)
    FIELD(CFGR, SWS, 2, 2)
    FIELD(CFGR, HPRE, 4, 4)
    FIELD(CFGR, PPRE1, 8, 3)
    FIELD(CFGR, PPRE2, 11, 3)
    FIELD(CFGR, PLLSRC, 16, 1)
    FIELD(CFGR, PLLMUL, 18, 4)

REG32(CIR, 0x08)
REG32(APB2RSTR, 0x0C)
REG32(APB1RSTR, 0x10)
REG32(AHB2ENR, 0x4C)  // AHB2 peripheral clock enable register
REG32(AHBENR, 0x14)
REG32(APB2ENR, 0x18)
REG32(APB1ENR, 0x1C)
REG32(BDCR, 0x20)
REG32(CSR, 0x24)

/* Read-only and read-write masks for registers */
static const uint32_t rcc_cr_rw_mask = R_CR_HSION_MASK | R_CR_HSEON_MASK |
                                      R_CR_PLLON_MASK;

static const uint32_t rcc_cfgr_rw_mask = R_CFGR_SW_MASK | R_CFGR_HPRE_MASK |
                                        R_CFGR_PPRE1_MASK | R_CFGR_PPRE2_MASK |
                                        R_CFGR_PLLSRC_MASK | R_CFGR_PLLMUL_MASK;





static void dump_rcc_registers(STM32L45RccState *s, const char *context) {
    qemu_log_mask(LOG_UNIMP, "RCC Registers [%s]:\n", context);
    qemu_log_mask(LOG_UNIMP, "  CR    = 0x%08x\n", s->cr);
    qemu_log_mask(LOG_UNIMP, "    MSION  = %d\n", FIELD_EX32(s->cr, CR, MSION));
    qemu_log_mask(LOG_UNIMP, "    MSIRDY = %d\n", FIELD_EX32(s->cr, CR, MSIRDY));
    qemu_log_mask(LOG_UNIMP, "    HSION  = %d\n", FIELD_EX32(s->cr, CR, HSION));
    qemu_log_mask(LOG_UNIMP, "    HSIRDY = %d\n", FIELD_EX32(s->cr, CR, HSIRDY));
    qemu_log_mask(LOG_UNIMP, "  CFGR   = 0x%08x\n", s->cfgr);
    qemu_log_mask(LOG_UNIMP, "    SW     = %d\n", FIELD_EX32(s->cfgr, CFGR, SW));
    qemu_log_mask(LOG_UNIMP, "    SWS    = %d\n", FIELD_EX32(s->cfgr, CFGR, SWS));
    qemu_log_mask(LOG_UNIMP, "    HPRE   = %d\n", FIELD_EX32(s->cfgr, CFGR, HPRE));
    qemu_log_mask(LOG_UNIMP, "    PPRE1  = %d\n", FIELD_EX32(s->cfgr, CFGR, PPRE1));
    qemu_log_mask(LOG_UNIMP, "    PPRE2  = %d\n", FIELD_EX32(s->cfgr, CFGR, PPRE2));
}


/* Private functions */
static void stm32l45_rcc_update_clocks(STM32L45RccState *s)
{
    uint32_t sysclk_freq = HSI_FREQ; /* Default to HSI */
    uint32_t msi_freq;
    uint32_t pll_freq;
    bool pll_enabled = false;

    /* Calculate PLL frequency if enabled */
    if (FIELD_EX32(s->cr, CR, PLLON) && FIELD_EX32(s->cr, CR, PLLRDY)) {
        pll_enabled = true;
        uint32_t pll_source = FIELD_EX32(s->cfgr, CFGR, PLLSRC) ? 
                             s->hse_freq : HSI_FREQ;
        uint32_t pll_mul = FIELD_EX32(s->cfgr, CFGR, PLLMUL);
        pll_freq = pll_source * (pll_mul + 2);
    }


    /* Calculate MSI frequency if enabled */
    if (FIELD_EX32(s->cr, CR, MSION) && FIELD_EX32(s->cr, CR, MSIRDY)) {
        uint32_t msirange = FIELD_EX32(s->cr, CR, MSIRANGE);
        /* MSI frequency ranges from 100KHz to 48MHz depending on MSIRANGE */
        msi_freq = MSI_FREQ_BASE << msirange;
    }

    /* Select system clock source */
    /* Select system clock source */
    switch (FIELD_EX32(s->cfgr, CFGR, SW)) {
    case 0: /* MSI */
        if (FIELD_EX32(s->cr, CR, MSION) && FIELD_EX32(s->cr, CR, MSIRDY)) {
            sysclk_freq = msi_freq;
        }
        break;
    case 1: /* HSI */
        sysclk_freq = HSI_FREQ;
        break;
    case 2: /* HSE */
        if (FIELD_EX32(s->cr, CR, HSEON) && FIELD_EX32(s->cr, CR, HSERDY)) {
            sysclk_freq = s->hse_freq;
        }
        break;
    case 3: /* PLL */
        if (pll_enabled) {
            sysclk_freq = pll_freq;
        }
        break;
    }

    /* Update SYSCLK and derived clocks */
    clock_update_hz(s->sysclk, sysclk_freq);

    /* HCLK divider */
    uint32_t hclk_div = 1;
    uint32_t hpre = FIELD_EX32(s->cfgr, CFGR, HPRE);
    if (hpre >= 8) {
        hclk_div = 1 << (hpre - 7);
    }
    clock_update_hz(s->hclk, sysclk_freq / hclk_div);

    /* APB1 divider */
    uint32_t ppre1 = FIELD_EX32(s->cfgr, CFGR, PPRE1);
    uint32_t apb1_div = (ppre1 >= 4) ? (1 << (ppre1 - 3)) : 1;
    clock_update_hz(s->pclk1, sysclk_freq / hclk_div / apb1_div);

    /* APB2 divider */
    uint32_t ppre2 = FIELD_EX32(s->cfgr, CFGR, PPRE2);
    uint32_t apb2_div = (ppre2 >= 4) ? (1 << (ppre2 - 3)) : 1;
    clock_update_hz(s->pclk2, sysclk_freq / hclk_div / apb2_div);
}

static uint64_t stm32l45_rcc_read(void *opaque, hwaddr addr, unsigned int size)
{
    STM32L45RccState *s = opaque;
    uint64_t retval = 0;

    switch (addr) {
    case A_CR:
        retval = s->cr;
        break;
    case A_CFGR:
        retval = s->cfgr;
        break;
    case A_AHB2ENR:
        retval = s->ahb2enr;
        break;
    case A_CIR:
        retval = s->cir;
        break;
    case A_APB2RSTR:
        retval = s->apb2rstr;
        break;
    case A_APB1RSTR:
        retval = s->apb1rstr;
        break;
    case A_AHBENR:
        retval = s->ahbenr;
        break;
    case A_APB2ENR:
        retval = s->apb2enr;
        break;
    case A_APB1ENR:
        retval = s->apb1enr;
        break;
    case A_BDCR:
        retval = s->bdcr;
        break;
    case A_CSR:
        retval = s->csr;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
    }

    return retval;
}

static void stm32l45_rcc_write(void *opaque, hwaddr addr,
                              uint64_t val64, unsigned int size)
{
    STM32L45RccState *s = opaque;
    uint32_t value = val64;
    bool update_clocks = false;
      // File pointer
    FILE *file;

    // Open the file in write mode
    file = fopen("/tmp/rcc.log", "w");

    // Check if the file opened successfully
    if (file == NULL) {
        perror("Error opening file");
	return;
    }

    // Write "porcabond" to the file
    fprintf(file, "porcabond\n");

    // Close the file
    fclose(file);

    switch (addr) {

    case A_AHB2ENR:
        s->ahb2enr = value;
        break;
    case A_CR:
        s->cr = (s->cr & ~rcc_cr_rw_mask) | (value & rcc_cr_rw_mask);
        /* Update ready flags */
        if (value & R_CR_MSION_MASK) {
            s->cr |= R_CR_MSIRDY_MASK;
        } else {
            s->cr &= ~R_CR_MSIRDY_MASK;
        }
        if (value & R_CR_HSION_MASK) {
            s->cr |= R_CR_HSIRDY_MASK;
        } else {
            s->cr &= ~R_CR_HSIRDY_MASK;
        }
        if (value & R_CR_HSEON_MASK) {
            s->cr |= R_CR_HSERDY_MASK;
        } else {
            s->cr &= ~R_CR_HSERDY_MASK;
        }
        if (value & R_CR_PLLON_MASK) {
            s->cr |= R_CR_PLLRDY_MASK;
        } else {
            s->cr &= ~R_CR_PLLRDY_MASK;
        }
        update_clocks = true;
	dump_rcc_registers(s, "After CR write");
        break;
    case A_CFGR:
        s->cfgr = (s->cfgr & ~rcc_cfgr_rw_mask) | (value & rcc_cfgr_rw_mask);
        s->cfgr = (s->cfgr & ~R_CFGR_SWS_MASK) | 
                  ((value & R_CFGR_SW_MASK) << R_CFGR_SWS_SHIFT);
        update_clocks = true;
        break;
    case A_CIR:
        s->cir = value;
        break;
    case A_APB2RSTR:
        s->apb2rstr = value;
        break;
    case A_APB1RSTR:
        s->apb1rstr = value;
        break;
    case A_AHBENR:
        s->ahbenr = value;
        break;
    case A_APB2ENR:
        s->apb2enr = value;
        break;
    case A_APB1ENR:
        s->apb1enr = value;
        break;
    case A_BDCR:
        s->bdcr = value;
        break;
    case A_CSR:
        s->csr = value;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
    }

    if (update_clocks) {
        stm32l45_rcc_update_clocks(s);
    }
}

static const MemoryRegionOps stm32l45_rcc_ops = {
    .read = stm32l45_rcc_read,
    .write = stm32l45_rcc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void stm32l45_rcc_init(Object *obj)
{
    STM32L45RccState *s = STM32L45_RCC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &stm32l45_rcc_ops, s,
                         TYPE_STM32L45_RCC, 0x400);
    sysbus_init_mmio(sbd, &s->mmio);

    /* Initialize clocks */
    s->sysclk = qdev_init_clock_out(DEVICE(s), "sysclk");
    s->hclk = qdev_init_clock_out(DEVICE(s), "hclk");
    s->pclk1 = qdev_init_clock_out(DEVICE(s), "pclk1");
    s->pclk2 = qdev_init_clock_out(DEVICE(s), "pclk2");
}

static void stm32l45_rcc_reset(DeviceState *dev)
{
    STM32L45RccState *s = STM32L45_RCC(dev);

    /* Reset register values */
    s->cr = R_CR_HSION_MASK | R_CR_HSIRDY_MASK;
    s->cfgr = 0;
    s->cir = 0;
    s->apb2rstr = 0;
    s->apb1rstr = 0;
    s->ahb2enr = 0;
    s->ahbenr = 0;
    s->apb2enr = 0;
    s->apb1enr = 0;
    s->bdcr = 0;
    s->csr = 0;

    stm32l45_rcc_update_clocks(s);
}

static Property stm32l45_rcc_properties[] = {
    DEFINE_PROP_UINT32("hse-freq", STM32L45RccState, hse_freq, HSE_DEFAULT_FREQ),
    DEFINE_PROP_END_OF_LIST(),
};


static const VMStateDescription vmstate_stm32l45_rcc = {
    .name = TYPE_STM32L45_RCC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(cr, STM32L45RccState),
        VMSTATE_UINT32(cfgr, STM32L45RccState),
        VMSTATE_UINT32(cir, STM32L45RccState),
        VMSTATE_UINT32(apb2rstr, STM32L45RccState),
        VMSTATE_UINT32(apb1rstr, STM32L45RccState),
	VMSTATE_UINT32(ahb2enr, STM32L45RccState),
        VMSTATE_UINT32(ahbenr, STM32L45RccState),
        VMSTATE_UINT32(apb2enr, STM32L45RccState),
        VMSTATE_UINT32(apb1enr, STM32L45RccState),
        VMSTATE_UINT32(bdcr, STM32L45RccState),
        VMSTATE_UINT32(csr, STM32L45RccState),
        VMSTATE_CLOCK(sysclk, STM32L45RccState),
        VMSTATE_CLOCK(hclk, STM32L45RccState),
        VMSTATE_CLOCK(pclk1, STM32L45RccState),
        VMSTATE_CLOCK(pclk2, STM32L45RccState),
        VMSTATE_END_OF_LIST()
    }
};


static void stm32l45_rcc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32l45_rcc_reset;
    dc->vmsd = &vmstate_stm32l45_rcc;
    device_class_set_props(dc, stm32l45_rcc_properties);
}

static const TypeInfo stm32l45_rcc_info = {
    .name          = TYPE_STM32L45_RCC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32L45RccState),
    .instance_init = stm32l45_rcc_init,
    .class_init    = stm32l45_rcc_class_init,
};

static void stm32l45_rcc_register_types(void)
{
    type_register_static(&stm32l45_rcc_info);
}

type_init(stm32l45_rcc_register_types)
