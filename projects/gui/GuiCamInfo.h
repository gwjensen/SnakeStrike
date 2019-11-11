#ifndef GuiCamInfo_h
#define GuiCamInfo_h

#include <string>
#include <vector>

class GuiCamInfo
{
    public:
        GuiCamInfo();
        ~GuiCamInfo();

        //Modifies a property value.
        virtual void SetPropertyValue (const std::string& iName, const std::string& oValue) = 0;

        //Retrieves a list of property names.
        virtual int GetPropertyNames (std::vector< std::string>& oNames) const = 0;

        //Retrieves a property value.
        virtual bool GetPropertyValue (const std::string& iName, std::string& oValue) const = 0;

        //Retrieves a human readable name.
        virtual std::string GetFriendlyName () const = 0;

        //Retrieves the unique device name.
        virtual std::string GetFullName () const = 0;

        //Retrieves the device vendor name.
        virtual std::string GetVendorName () const = 0;

};

#endif // GuiCamInfo_h
