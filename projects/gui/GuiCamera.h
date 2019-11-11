#ifndef GuiCamera_h
#define GuiCamera_h

#include <QMutex>
#include <QReadWriteLock>
#include <QTextStream>
#include <QFile>
#include <memory>

#include "ProjectDialog.h" //for the constants
#include "GuiImage.h"
#include "GuiCamInfo.h"

const QString DEBUG_TIMESTAMP_OUTPUT_DIR = "/tmp/camera_capture";

class GuiCamera : public QObject
{
    Q_OBJECT
    public:
        //parent of null means we can move it to another thread.
        explicit GuiCamera(QObject *ipParent = nullptr);

        //parent of null means we can move it to another thread.
        explicit GuiCamera( std::unique_ptr<GuiCamInfo>& iInfo, const int iIdx, QObject *ipParent = 0);

        virtual ~GuiCamera(){ }

        virtual bool MakeConnect() = 0;
        //virtual bool MakeConnect( const GuiCamInfo& iInfo ) = 0;
        virtual void Open() = 0;
        virtual void Close() = 0;

        virtual void SetupHardwareTrigger() = 0;
        virtual void SetupSoftwareTrigger() = 0;

        virtual bool CheckForWaitingImage() = 0;
        virtual GuiImage RetrieveWaitingImage( std::string& oError ) = 0;

        virtual void MaxBuffer( int64_t iNewNum ) = 0;
        virtual uint64_t IncreaseCount() = 0;
        virtual uint64_t IncreaseDropCount() = 0;
        virtual uint64_t Count() const final;
        virtual uint64_t DropCount() const final;
        virtual uint32_t IncrementGrabThreads() final;
        virtual uint32_t DecrementGrabThreads() final;
        virtual void UpdateConfig(  const std::string& iGenApiConfigFile ) = 0;
        virtual void OutputVideo( ) final;
        virtual void OutputImages( ) final;


        virtual int64_t MaxBuffer() const = 0;
        virtual int64_t Height() = 0;
        virtual int64_t Width() = 0;
        virtual const QString Name() const = 0;
        virtual double Hz() = 0;
        virtual const QString FirmwareLevel() = 0;
        virtual int CamIdx() const = 0;
        virtual uint64_t RecordCount() final;

        virtual bool IsReadyForTrigger() const = 0;
        virtual bool IsHardwareTriggered() = 0;
        virtual bool IsConnected() = 0;
        virtual bool IsOpen() = 0;

        virtual QString ImageTimestepFileName( const QString& iDirectory ) const final;

        virtual QString ImageOutputDir( const QString& iBaseDir ) const final;
        virtual void WriteImageInfoToLog( uint64_t iCurImageCount, uint64_t iTimestampNanoSecs ) final;

        //default writes to DEBUG_TIMESTAMP
        virtual void InitFileWrites( const QString& iDir = DEBUG_TIMESTAMP_OUTPUT_DIR) final;

        virtual void CleanUpFileWrites() final;

        virtual int& RetrievalTimeout() final;
        virtual int RetrievalTimeout() const final;

    public slots: //called in response to a particular signal
        virtual void StartGrabbingPreview() = 0;
        virtual void StartGrabbingRecord( QString iDir, int iNumImages ) = 0;
        virtual void StopGrabbing() = 0;
        virtual void GrabThreadDone() = 0;

    signals:
        void SignalGrabbing();
        void SignalGrabDone(int i, QPixmap imPixmap);
        void SignalFinished();
        void SignalError(QString err);
        void SignalImageSaved();
        void SignalImageDropped();
        void SignalRefreshCameras();

    protected:
        friend class GrabAndSaveImageTask;
        virtual bool  IsStopped() = 0;

        QMutex      mMutex;
        QObject*            mpParent;
        std::unique_ptr<GuiCamInfo>  mCamInfo;
        int         mIdx;
        bool        mIsStopping;
        bool        mIsConnected;
        bool        mIsOpened;
        uint64_t    mRecordCount;
        uint64_t    mDropCount;
        uint32_t            mNumGrabThreadsRunning;
        QFile*              mImageTimestampsFile;
        QTextStream*        mStream;
        bool        mHardwareTriggered;


        QReadWriteLock      mRwLock;
        int                 mNumBurstImages;
        int    mImageRetrieveTimeout;
        bool   mMakeVideoOuput;
};

#endif // GuiCamera_h

