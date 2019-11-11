#include <QDebug>
#include "BaslerCamInfo.h"

#include "BaslerDevice.h"

BaslerDevice::BaslerDevice()
{

}


std::vector< std::unique_ptr< GuiCamInfo > > BaslerDevice::EnumerateDevices() const
{
    std::vector< std::unique_ptr<GuiCamInfo> > device_list;

    //:NOTE: This will throw an exception if PylonInitialize() was not called before it.
    Pylon::CTlFactory& tl_factory = Pylon::CTlFactory::GetInstance();

    Pylon::DeviceInfoList_t devices;
    int num_devices = 0;
    try
    {
        num_devices = tl_factory.EnumerateDevices(devices);
    }
    catch (Pylon::GenericException  &e)
    {
        qDebug("An exception occurred trying to enumerate devices:");
        qDebug("%s", e.GetDescription());
        return device_list;
    }

    for (int i=0; i < num_devices; ++i)
    {
        std::unique_ptr<BaslerCamInfo> info( new BaslerCamInfo(devices[i]) );
        //info->SetDeviceClass( Pylon::CBaslerUniversalInstantCameraTraits::DeviceClass() );
        std::unique_ptr<GuiCamInfo> base_info( std::move( info ) );
        device_list.push_back( std::move( base_info ) );
    }
    std::cout << "Found " << num_devices << " number of camera devices." << std::endl;
    return device_list;
}
