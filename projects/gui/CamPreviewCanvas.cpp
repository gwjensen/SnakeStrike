#include <Qt>
#include <iostream>

#include "CamPreviewCanvas.h"

CamPreviewCanvas::CamPreviewCanvas(QWidget* ipParent)
    : QLabel(ipParent)
{
    this->setMinimumSize(50,50);
    setScaledContents(false);
    setAlignment( Qt::AlignLeft | Qt::AlignTop );
}

CamPreviewCanvas::~CamPreviewCanvas()
{

}

void CamPreviewCanvas::setPixmap ( const QPixmap& iPixmap)
{
    mPixmap = iPixmap;
    QLabel::setPixmap( iPixmap );
}

int CamPreviewCanvas::heightForWidth( int iWidth ) const
{
    return mPixmap.isNull() ? this->height() : ((qreal)mPixmap.height()*iWidth)/mPixmap.width();
}

QSize CamPreviewCanvas::sizeHint() const
{
    int w = this->width();
    return QSize( w, heightForWidth(w) );
}

QPixmap CamPreviewCanvas::ScaledPixmap() const
{
    return mPixmap.scaled(this->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
}

void CamPreviewCanvas::resizeEvent(QResizeEvent* ipEvent)
{
    if (!mPixmap.isNull())
    {
        QLabel::setPixmap(ScaledPixmap());
    }
}
