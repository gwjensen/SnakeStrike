#include <QtWidgets>

#include "CamPreviewCanvas.h"
#include "CamPreviewArea.h"

#include "CamPreviewWindow.h"


CamPreviewWindow::CamPreviewWindow(QMdiArea* ipParent, int iCamIdx)
    :QWidget(ipParent),
      mCamIdx(iCamIdx)
{
    setAttribute(Qt::WA_DeleteOnClose);
    mIsUntitled = true;
    mpLabel = new CamPreviewCanvas(this);
    mpLabel->setMinimumSize(400, 400);


    // Set the window title
    QString winName = "Cam_" + QString::number(iCamIdx);
    setWindowTitle( winName );
    setWindowIcon(QIcon("snake_icon_inv.jpg"));

    // Adding a widget as a sub window in the Mdi Area
    mpMyInstance = ipParent->addSubWindow(this);

    mpMyInstance->setMinimumSize(400, 400);
    mpMyInstance->setMaximumSize(500, 500);
    mpMyInstance->adjustSize();

    mpParent = ipParent;
    mShouldClose = false;

}

CamPreviewWindow::~CamPreviewWindow()
{
    static_cast<CamPreviewArea*>(mpParent)->RemoveSubwindow( mpMyInstance );
    delete mpLabel;
}

void CamPreviewWindow::mouseReleaseEvent( QMouseEvent* iEvent )
{
    if(iEvent->button() == Qt::LeftButton)
    {
        emit SignalGrabbedPixelValue( mCamIdx, iEvent->x(), iEvent->y());
    }
}

CamPreviewCanvas* CamPreviewWindow::GetLabel(){
    return mpLabel;
}

void CamPreviewWindow::MarkForClose()
{
    mShouldClose = true;
}

void CamPreviewWindow::closeEvent(QCloseEvent* iEvent)
{
    if ( !mShouldClose )
    {
        iEvent->ignore();
    }
    else
    {
        iEvent->accept();
    }
}

void CamPreviewWindow::UpdateImageSize( int64_t iWidth, int64_t iHeight )
{
    mpLabel->setMinimumSize( iWidth, iHeight );
    mpLabel->adjustSize();
    mpMyInstance->setMinimumSize(iWidth, iHeight);
    mpMyInstance->setMaximumSize(iWidth, iHeight);
    mpMyInstance->adjustSize();
}
