#include <iostream>
#include <QCoreApplication>

#include "CorrespondenceImage.h"



CorrespondenceImage::CorrespondenceImage(QWidget* ipParent)
    :mpParent(ipParent)
{

}

void CorrespondenceImage::mouseReleaseEvent( QMouseEvent* iEvent )
{
    if (iEvent->button() == Qt::LeftButton)
    {
        emit SignalClick(iEvent->x(), iEvent->y());
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
    }

}

void CorrespondenceImage::UpdateImage(QPixmap iPixmap, int iWidth, int iHeight )
{
    std::cerr << "Update pixmap" << std::endl;
    setPixmap(iPixmap);
    setMinimumSize( iWidth, iHeight );
    adjustSize();
    update();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
}
