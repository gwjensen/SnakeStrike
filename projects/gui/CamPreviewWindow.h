#ifndef CamPreviewWindow_h
#define CamPreviewWindow_h

#include <stdint.h>
#include <QTextEdit>
#include <QMdiArea>
#include <QMdiSubWindow>

#include "CamPreviewCanvas.h"

class CamPreviewWindow : public QWidget
{
    Q_OBJECT

    public:
        CamPreviewWindow(QMdiArea* ipParent, int iCamIdx);
        ~CamPreviewWindow();
        CamPreviewCanvas* GetLabel();
        void closeEvent(QCloseEvent* iEvent);
        void UpdateImageSize( int64_t iWidth, int64_t iHeight );
        void MarkForClose();

    protected:
        void mouseReleaseEvent ( QMouseEvent* iEvent );

    signals:
        void SignalGrabbedPixelValue( uint32_t iCamIdx, uint32_t iX, uint32_t iY);

    private slots:


    private:
        bool mIsUntitled;
        int mCamIdx;
        CamPreviewCanvas* mpLabel;
        QMdiSubWindow* mpMyInstance;
        QMdiArea* mpParent;
        bool mShouldClose;
};

#endif // CamPreviewWindow_h
