#ifndef BaslerDevice_h
#define BaslerDevice_h

#include <vector>
#include <memory>

#include "GuiDevice.h"

class BaslerDevice : public GuiDevice
{
    public:
        BaslerDevice();

        std::vector< std::unique_ptr< GuiCamInfo > > EnumerateDevices() const;

};


#endif // BaslerDevice_h
