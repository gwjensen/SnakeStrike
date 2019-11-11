#ifndef GrabAndSaveImageTask_h
#define GrabAndSaveImageTask_h

#include <QRunnable>
#include <QObject>
#include <QStringList>
#include <QProcess>

#include "GuiCamera.h"

class GrabAndSaveImageTask : public QObject, public QRunnable
{
    Q_OBJECT
    public:
        GrabAndSaveImageTask( GuiCamera* iCam,
                              QTextStream* iStream,
                              uint32_t iMaxNumImages,
                              const QString& iSaveDir,
                              bool iMakeVideo);
        void run( );
        //static QMutex mutex;

    signals:
        void finished();
        void KickOffFfmpeg( QStringList iArgs );

    private:
        GuiCamera*      mpCamera;
        uint32_t        mRecordCount;
        uint32_t        mCamIdx;
        uint32_t        mMaxNumImages;
        QString         mSaveDir;
        QTextStream*    mpStream;
        QMutex*         mpMutex;
        bool            mMakeVideo;
        QProcess*       mpProcess;
};

#endif // GrabAndSaveImageTask_h
