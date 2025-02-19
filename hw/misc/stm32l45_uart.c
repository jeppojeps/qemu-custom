#include "qemu/osdep.h"
#include "hw/misc/stm32l45_uart.h"
#include "qemu/error-report.h"
#include "qom/object.h"
#include "migration/vmstate.h"
#include "hw/qdev-properties.h"
#include "chardev/char-fe.h"
#include "hw/qdev-properties-system.h"
#include "hw/qdev-clock.h"
#define R_USART_ISR_IDLE_MASK       (1 << 4)
#define R_USART_ISR_TEACK_MASK      (1U << 21)  // 0x00200000
#define R_USART_ISR_REACK_MASK      (1U << 22)  // 0x00400000

static uint64_t stm32l45_uart_read(void *opaque, hwaddr addr,
                                  unsigned size)
{
    STM32L45UARTState *s = (STM32L45UARTState *)opaque;
    uint64_t ret = 0;

    switch (addr) {
    case A_USART_CR1:
        ret = s->cr1;
        qemu_log_mask(LOG_UNIMP, "UART: Read CR1 = 0x%x\n", (unsigned)ret);
        break;
    case A_USART_CR2:
        ret = s->cr2;
        break;
    case A_USART_CR3:
        ret = s->cr3;
        break;
    case A_USART_CFGR:
        ret = s->cfgr;
        break;
    case A_USART_BRR:
        ret = s->brr;
        break;
    case A_USART_GTPR:
        ret = s->gtpr;
        break;
    case A_USART_RTOR:
        ret = s->rtor;
        break;
    case A_USART_RQR:
        ret = s->rqr;
        break;
    case A_USART_ISR:
        ret = s->isr;
        qemu_log_mask(LOG_UNIMP, "UART: Read ISR = 0x%x (TXE=%d, TC=%d, RXNE=%d)\n", 
                 (unsigned)ret,
                 !!(ret & R_USART_ISR_TXE_MASK),
                 !!(ret & R_USART_ISR_TC_MASK),
                 !!(ret & R_USART_ISR_RXNE_MASK));
        break;
    case A_USART_ICR:
        ret = s->icr;
        break;
    case A_USART_RDR:
        ret = s->rdr;
        s->isr &= ~R_USART_ISR_RXNE_MASK;
        break;
    case A_USART_TDR:
        ret = s->tdr;
        break;
    case A_USART_PRESC:
        ret = s->presc;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                     "stm32l45_uart_read: Bad offset %x\n", (int)addr);
    }

    return ret;
}

static void stm32l45_uart_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned size)
{
    STM32L45UARTState *s = (STM32L45UARTState *)opaque;
    unsigned char ch;

    switch (addr) {
    case A_USART_CR1:
        uint32_t old_value = s->cr1;
        s->cr1 = value;
        
        qemu_log_mask(LOG_UNIMP, "UART CR1 write:\n");
        qemu_log_mask(LOG_UNIMP, "  Old value: 0x%08x\n", old_value);
        qemu_log_mask(LOG_UNIMP, "  New value: 0x%08x\n", value);
        
        // Handle TEACK and REACK flags
        if (value & R_USART_CR1_UE_MASK) {
            // When first enabled, TEACK and REACK should be clear
            s->isr &= ~(R_USART_ISR_TEACK_MASK | R_USART_ISR_REACK_MASK);
        }
        
        qemu_log_mask(LOG_UNIMP, "  UE: %d->%d\n", 
                     !!(old_value & R_USART_CR1_UE_MASK),
                     !!(value & R_USART_CR1_UE_MASK));
        qemu_log_mask(LOG_UNIMP, "  TE: %d->%d\n",
                     !!(old_value & R_USART_CR1_TE_MASK),
                     !!(value & R_USART_CR1_TE_MASK));
        qemu_log_mask(LOG_UNIMP, "  RE: %d->%d\n",
                     !!(old_value & R_USART_CR1_RE_MASK),
                     !!(value & R_USART_CR1_RE_MASK));
    break;
    case A_USART_CR2:
        qemu_log_mask(LOG_UNIMP, "UART: Write to CR2, value=0x%x\n", (unsigned)value);
        s->cr2 = value;
        break;
    case A_USART_CR3:
        qemu_log_mask(LOG_UNIMP, "UART: Write to CR3, value=0x%x\n", (unsigned)value);
        s->cr3 = value;
        break;
    case A_USART_CFGR:
        qemu_log_mask(LOG_UNIMP, "UART: Write to CFGR, value=0x%x\n", (unsigned)value);
        
        // Log specific field values for debugging
        qemu_log_mask(LOG_UNIMP, "UART: CFGR - Prescaler: %d, STOP bits: %d, SWAP: %d\n",
            FIELD_EX32(value, USART_CFGR, PRESCALER),
            FIELD_EX32(value, USART_CFGR, STOP),
            FIELD_EX32(value, USART_CFGR, SWAP));
        
        s->cfgr = value;
        break;
    case A_USART_BRR:
        uint32_t pclk_freq = clock_get_hz(s->pclk1);
        bool over8 = (s->cr1 & R_USART_CR1_OVER8_MASK) != 0;
        uint32_t usartdiv = value & 0xFFFF;
        
        qemu_log_mask(LOG_UNIMP, "UART BRR write:\n");
        qemu_log_mask(LOG_UNIMP, "  Value: 0x%x\n", (unsigned)value);
        qemu_log_mask(LOG_UNIMP, "  PCLK: %u Hz\n", pclk_freq);
        qemu_log_mask(LOG_UNIMP, "  OVER8: %d\n", over8);
        qemu_log_mask(LOG_UNIMP, "  USARTDIV: %u\n", usartdiv);
        
        // Calculate actual baud rate
        uint32_t baud = pclk_freq / (usartdiv * (over8 ? 8 : 16));
        qemu_log_mask(LOG_UNIMP, "  Calculated baud rate: %u\n", baud);
        
        s->brr = value;
        break;
    case A_USART_GTPR:
        s->gtpr = value;
        break;
    case A_USART_RTOR:
        s->rtor = value;
        break;
    case A_USART_RQR:
        s->rqr = value;
        break;
    case A_USART_ICR:
        s->icr = value;
        s->isr &= ~value;  // Clear the bits that were written as 1
        break;
    case A_USART_TDR:
        qemu_log_mask(LOG_UNIMP, "UART: Write to TDR, value=0x%x, CR1=0x%x\n", 
                     (unsigned)value, s->cr1);
        if (s->cr1 & R_USART_CR1_UE_MASK && s->cr1 & R_USART_CR1_TE_MASK) {
            ch = value & 0xFF;
            qemu_chr_fe_write(&s->chr, &ch, 1);
            qemu_log_mask(LOG_UNIMP, "UART: Transmitted character: '%c' (0x%02x)\n", 
                         ch, ch);
        } else {
            qemu_log_mask(LOG_UNIMP, "UART: Transmission blocked - UE:%d TE:%d\n",
                         !!(s->cr1 & R_USART_CR1_UE_MASK),
                         !!(s->cr1 & R_USART_CR1_TE_MASK));
        }
        s->tdr = value;
        s->isr |= R_USART_ISR_TXE_MASK | R_USART_ISR_TC_MASK;
        break;
    case A_USART_PRESC:
        s->presc = value;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                     "stm32l45_uart_write: Bad offset %x\n", (int)addr);
    }
}

static void stm32l45_uart_receive(void *opaque, const uint8_t *buf, int size)
{
    STM32L45UARTState *s = (STM32L45UARTState *)opaque;

    if (!(s->cr1 & R_USART_CR1_UE_MASK) || !(s->cr1 & R_USART_CR1_RE_MASK)) {
        return;
    }

    s->rdr = *buf;
    s->isr |= R_USART_ISR_RXNE_MASK;
}

static int stm32l45_uart_can_receive(void *opaque)
{
    STM32L45UARTState *s = (STM32L45UARTState *)opaque;

    return !(s->isr & R_USART_ISR_RXNE_MASK);
}

static void stm32l45_uart_event(void *opaque, QEMUChrEvent event)
{
    /* Handle modem control events if needed */
}

static const MemoryRegionOps stm32l45_uart_ops = {
    .read = stm32l45_uart_read,
    .write = stm32l45_uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void stm32l45_uart_init(Object *obj)
{
    STM32L45UARTState *s = STM32L45_UART(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    
    qemu_log_mask(LOG_UNIMP, "STM32L4: Initializing UART device\n");

    // Initialize ISR with transmit flags set to indicate TX is ready

    s->isr = R_USART_ISR_TXE_MASK | R_USART_ISR_TC_MASK | R_USART_ISR_TEACK_MASK;


    memory_region_init_io(&s->mmio, obj, &stm32l45_uart_ops, s,
                         TYPE_STM32L45_UART, 0x400);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);

    Error *local_err = NULL;
    qemu_chr_fe_init(&s->chr, NULL, &local_err);
}

static void stm32l45_uart_realize(DeviceState *dev, Error **errp)
{
    STM32L45UARTState *s = STM32L45_UART(dev);
    
    qemu_log_mask(LOG_UNIMP, "STM32L4: Realizing UART device\n");

    qemu_chr_fe_set_handlers(&s->chr,
                            stm32l45_uart_can_receive,
                            stm32l45_uart_receive,
                            stm32l45_uart_event,
                            NULL,
                            s,
                            NULL,
                            true);
}


static Property stm32l45_uart_properties[] = {
    DEFINE_PROP_LINK("chardev", STM32L45UARTState, chr.chr, TYPE_CHARDEV, Chardev *),
    DEFINE_PROP_LINK("pclk1", STM32L45UARTState, pclk1, TYPE_CLOCK, Clock *),
    DEFINE_PROP_LINK("sysclk", STM32L45UARTState, sysclk, TYPE_CLOCK, Clock *),
    DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription vmstate_stm32l45_uart = {
    .name = TYPE_STM32L45_UART,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(cr1, STM32L45UARTState),
        VMSTATE_UINT32(cr2, STM32L45UARTState),
        VMSTATE_UINT32(cr3, STM32L45UARTState),
        VMSTATE_UINT32(brr, STM32L45UARTState),
        VMSTATE_UINT32(isr, STM32L45UARTState),
        VMSTATE_UINT32(tdr, STM32L45UARTState),
        VMSTATE_UINT32(rdr, STM32L45UARTState),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32l45_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32l45_uart_realize;
    dc->vmsd = &vmstate_stm32l45_uart;
    device_class_set_props(dc, stm32l45_uart_properties);
}

static const TypeInfo stm32l45_uart_info = {
    .name          = TYPE_STM32L45_UART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32L45UARTState),
    .instance_init = stm32l45_uart_init,
    .class_init    = stm32l45_uart_class_init,
};

static void stm32l45_uart_register_types(void)
{
    type_register_static(&stm32l45_uart_info);
}

type_init(stm32l45_uart_register_types)
