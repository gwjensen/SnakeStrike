#ifndef CamMaker_h
#define CamMaker_h

#include <QObject>
#include <memory>
#include "GuiCamera.h"


extern "C"
{
    void CamMakerInitResources();
    GuiCamera* CamMaker(std::unique_ptr<GuiCamInfo>& iInfo, const int iIdx, QObject *ipParent = 0);
    void CamMakerDestroyResources();
}

#endif // CamMaker_h
