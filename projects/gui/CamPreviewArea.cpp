#include "CamPreviewArea.h"

CamPreviewArea::CamPreviewArea( QWidget* ipParent )
    :QMdiArea( ipParent )
{

}

CamPreviewArea::~CamPreviewArea()
{
    Reset();
}

void CamPreviewArea::Reset()
{
    mLabelPtrList.clear();
}

void CamPreviewArea::RemoveSubwindow( QWidget* iWidget )
{
    removeSubWindow( iWidget );
}

int CamPreviewArea::NumActivePreviews()
{
    return mLabelPtrList.size();
}
