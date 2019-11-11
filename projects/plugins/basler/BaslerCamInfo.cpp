#include "BaslerCamInfo.h"

BaslerCamInfo::BaslerCamInfo( const Pylon::CDeviceInfo& iInfo )
    :Pylon::CDeviceInfo( iInfo )
{

}

//Retrieves a list of property names.
int BaslerCamInfo::GetPropertyNames( std::vector< std::string >& oNames ) const
{
    Pylon::StringList_t list;
    int num =  Pylon::CDeviceInfo::GetPropertyNames( list );
    for (auto str : list)
    {
        oNames.push_back( std::string(str) );
    }
    return num;
}

//Retrieves a property value.
bool BaslerCamInfo::GetPropertyValue( const std::string& iName, std::string& oValue ) const
{
    //return Pylon::CBaslerUsbDeviceInfo::GetProperyValue( Pylon::String_t(iName.c_str()),
    //                                                     Pylon::String_t(iValue.c_str()) );
    const std::map< std::string, std::string>::const_iterator iter = mProperties.find( iName );
    bool exists = ( mProperties.end() == iter );

    if (exists)
    {
        oValue = iter->second;
    }

    return exists;
}

//Modifies a property value.
void BaslerCamInfo::SetPropertyValue( const std::string& iName, const std::string& iValue )
{
     Pylon::CDeviceInfo::SetPropertyValue( Pylon::String_t(iName.c_str()),
                                                    Pylon::String_t(iValue.c_str()) );
}

//Retrieves a human readable name.
std::string BaslerCamInfo::GetFriendlyName() const
{
     return std::string( Pylon::CDeviceInfo::GetFriendlyName() );
}

//Retrieves the unique device name.
std::string BaslerCamInfo::GetFullName() const
{
    return  std::string( Pylon::CDeviceInfo::GetFullName() );
}

//Retrieves the device vendor name.
std::string BaslerCamInfo::GetVendorName() const
{
    return  std::string( Pylon::CDeviceInfo::GetVendorName() );
}
