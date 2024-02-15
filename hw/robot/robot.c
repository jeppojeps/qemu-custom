#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "chardev/char.h"
#include "hw/hw.h"
#include "hw/irq.h"
#include "hw/robot/robot.h"
#include "exec/address-spaces.h"
#include "sysemu/sysemu.h"
#include "hw/robot/robot.h"

static uint64_t rb_read(void *opaque, hwaddr addr, unsigned int size)
{
    rbState *s = opaque;
    qemu_log_mask(LOG_GUEST_ERROR, "%s: read: addr=0x%x size=%d\n",
                  __func__, (int)addr,size);
    return s->roboReg[addr];
}

static void rb_write(void *opaque, hwaddr addr, uint64_t val64, unsigned int size)
{
    /*dummy code for future development*/
    rbState *s = opaque;
    uint32_t value = val64;
    unsigned char ch = value;
    (void)s;
    (void)ch;
    qemu_log_mask(LOG_GUEST_ERROR, "%s: write: addr=0x%x v=0x%x\n",
                  __func__, (int)addr, (int)value);
}

static void rb_init(rbState *s){
    s->roboReg[0]='1';
    s->roboReg[1]='3';
    s->roboReg[2]='3';
    s->roboReg[3]='7';
    s->roboReg[4]='!';
    s->roboReg[5]='!';
}

static const MemoryRegionOps rb_ops = {
    .read = rb_read,
    .write = rb_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 6}
};

rbState *rb_create(MemoryRegion *address_space, hwaddr base)
{
    rbState *s = g_malloc0(sizeof(rbState));
    rb_init(s);
    memory_region_init_io(&s->mmio, NULL, &rb_ops, s, TYPE_ROBOT, 32);
    memory_region_add_subregion(address_space, base, &s->mmio);
    return s;
}

