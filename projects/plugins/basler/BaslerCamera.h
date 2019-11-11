#ifndef BaslerCamera_h
#define BaslerCamera_h

#include <QObject>
#include <QMutex>
#include <pylon/PylonIncludes.h>
#include <pylon/usb/_BaslerUsbCameraParams.h>
#include <QTextStream>
#include <QRunnable>
#include <QReadWriteLock>
#include <QFile>
#include <pylon/BaslerUniversalGrabResultPtr.h>

#include "HardwareTriggerConfiguration.h"
#include "GuiCamera.h"
#include "GuiImage.h"
#include "BaslerCamInfo.h"
#include "GrabAndSaveImageTask.h"


typedef Pylon::CBaslerUniversalGrabResultPtr GrabResultPtr_t;

const uint32_t NUM_THREADS_IN_GRABBING_POOL = 15;

class BaslerCamera : public GuiCamera
{
    Q_OBJECT
    public:
        BaslerCamera( std::unique_ptr<GuiCamInfo>& iInfo, const int iIdx, QObject *ipParent = 0 );

        ~BaslerCamera();
        bool MakeConnect();
        void Open();
        void Close();

        bool CheckForWaitingImage(  );//returns false on timeout
        GuiImage RetrieveWaitingImage( std::string& oError );//returns empty gui image with empty mat

        void SetupHardwareTrigger();
        void SetupSoftwareTrigger();

        uint64_t IncreaseCount( );
        uint64_t IncreaseDropCount();

        void MaxBuffer( int64_t iNewNum );

        void UpdateConfig(  const std::string& iGenApiConfigFile );

        const GenApi::INodeMap* NodeMap() const;
        double          Hz();
        int             CamIdx() const;
        const QString   FirmwareLevel();
        const QString   Name() const;
        int64_t         MaxBuffer() const;
        int64_t         Width();
        int64_t         Height();

        bool IsHardwareTriggered();
        bool IsConnected();
        bool IsOpen();

        bool IsReadyForTrigger() const;

    protected:
        bool  IsStopped();
        bool  IsStoppedNonMutex();



    public slots:
        void StartGrabbingPreview();
        void StartGrabbingRecord( QString iDir, int iNumImages );
        void StopGrabbing();
        void GrabThreadDone();

        //these signals are inherited from GuiCamera and need to be used.
//    signals:
//        void SignalGrabbing();
//        void SignalGrabDone(int i, QPixmap imPixmap);
//        void SignalFinished();
//        void SignalError(QString err);
//        void SignalImageSaved();
//        void SignalImageDropped();
//        void SignalRefreshCameras();

    private:
        Camera_t*           mCamera;
        GenApi::INodeMap*   mNodeMap;
        Basler_UsbCameraParams::TriggerSourceEnums  mTriggerSource;
        Pylon::CHardwareTriggerConfiguration*       mTriggerConfig;
        GrabResultPtr_t  mpGrabResult;

};



#endif // BaslerCamera_h
