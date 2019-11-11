#include "BaslerDevice.h"

#include "DeviceMaker.h"

GuiDevice* DeviceMaker()
{
    return new BaslerDevice();
}
