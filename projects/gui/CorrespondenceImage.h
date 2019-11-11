#ifndef CorrespondenceImage_h
#define CorrespondenceImage_h

#include <QObject>
#include <QLabel>
#include <QWindow>
#include <QWidget>
#include <QMouseEvent>

#include "CorrespondenceDialog.h"


class CorrespondenceImage : public QLabel
{
    Q_OBJECT
    public:
        CorrespondenceImage(QWidget* ipParent);

    signals:
        void SignalClick(int iX, int iY);

    public slots:
        void UpdateImage(QPixmap iPixmap, int iWidth, int iHeight );

    private:
        void mouseReleaseEvent( QMouseEvent* iEvent );
        QWidget* mpParent;
};

#endif // CorrespondenceImage_h
