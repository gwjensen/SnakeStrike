#ifndef GUIDEVICE_H
#define GUIDEVICE_H

#include <vector>
#include <memory>

#include "GuiCamInfo.h"

class GuiDevice
{
    public:
        GuiDevice();
        virtual ~GuiDevice();

        virtual std::vector< std::unique_ptr< GuiCamInfo > > EnumerateDevices() const = 0;

};

#endif // GUIDEVICE_H
