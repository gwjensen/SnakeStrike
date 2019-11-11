#include <pylon/PylonIncludes.h>
#include "BaslerCamera.h"

#include "CamMaker.h"


GuiCamera* CamMaker(std::unique_ptr<GuiCamInfo>& iInfo, const int iIdx, QObject *ipParent)
{
    return new BaslerCamera(iInfo, iIdx, ipParent);
}

void CamMakerInitResources()
{
    std::fprintf( stdout, "Initializing pylon...\n");
    Pylon::PylonInitialize();
}

void CamMakerDestroyResources()
{
    std::fprintf(stdout, "Terminating pylon...\n");
    Pylon::PylonTerminate();
}
