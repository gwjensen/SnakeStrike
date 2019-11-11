#ifndef CamPreviewArea_h
#define CamPreviewArea_h

#include <QMdiArea>

#include "CamPreviewCanvas.h"
#include "MainWindow.h"

class CamPreviewArea : public QMdiArea
{
    Q_OBJECT

    public:
        CamPreviewArea(QWidget* ipParent);
        ~CamPreviewArea();
        int NumActivePreviews();
        void Reset();
        void RemoveSubwindow( QWidget* iWidget );

        CamPreviewCanvas* operator [](int i) const {return mLabelPtrList[i];}

    private:
        QVector< CamPreviewCanvas* > mLabelPtrList;
};

#endif // CamPreviewArea_h
