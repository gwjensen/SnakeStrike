#ifndef BaslerCamInfo_h
#define BaslerCamInfo_h

#include <string>
#include <vector>
#include <map>
#include "pylon/BaslerUniversalInstantCamera.h"
#include <pylon/PylonIncludes.h>

#include "GuiCamInfo.h"

typedef Pylon::CBaslerUniversalInstantCamera Camera_t;

class BaslerCamInfo : public GuiCamInfo, public Pylon::CDeviceInfo
{
    public:
        BaslerCamInfo( const Pylon::CDeviceInfo& iInfo );

        //Modifies a property value.
        void SetPropertyValue( const std::string& iName, const std::string& iValue );

        //Retrieves a list of property names.
        int GetPropertyNames( std::vector< std::string >& oNames ) const;

        //Retrieves a property value.
        bool GetPropertyValue( const std::string& iName, std::string& iValue ) const;

        //Retrieves a human readable name.
        std::string GetFriendlyName() const;

        //Retrieves the unique device name.
        std::string GetFullName() const;

        //Retrieves the device vendor name.
        std::string GetVendorName() const;

    private:
        std::map< std::string, std::string> mProperties;


};

#endif // BaslerCamInfo_h
