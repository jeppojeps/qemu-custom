#ifndef HW_ROBOT_H
#define HW_ROBOT_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_ROBOT "ROBOT"

typedef struct rbState rbState;
DECLARE_INSTANCE_CHECKER(rbState, ROBOT, TYPE_ROBOT)

struct rbState
{
    MemoryRegion mmio;
    unsigned char roboReg[6]; 
};

rbState *rb_create(MemoryRegion *address_space, hwaddr base);

#endif //HW_BUTTER_ROBOT_H

