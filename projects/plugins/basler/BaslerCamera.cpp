#include <QCoreApplication>
#include <QTime>
#include <QThread>
#include <QStringList>
#include <QTextStream>
#include <QThreadPool>
#include <QMessageBox>
#include <pylon/ConfigurationEventHandler.h>
#include <GenApi/IInteger.h>
#include <pylon/ImagePersistence.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "BaslerCamera.h"

//we expect parent to usually be null because otherwise we aren't allowed to move this camera to a new thread
BaslerCamera::BaslerCamera(std::unique_ptr<GuiCamInfo>& iInfo,
                           const int iIdx,
                           QObject *ipParent)
    : GuiCamera(iInfo, iIdx, ipParent),
      mTriggerConfig(NULL)
{
    try
    {
       mCamera = new Camera_t();
       mNodeMap = 0x0;
    }
    catch (...)
    {
        qDebug("An exception occurred during camera construction.");
    }
}

BaslerCamera::~BaslerCamera()
{
    try
    {
        StopGrabbing();
        mCamera->StopGrabbing();

        if ( NULL == mTriggerConfig )
        {
            mCamera->DeregisterConfiguration( mTriggerConfig );
            mTriggerConfig = NULL;
        }

        if (NULL != mCamera)
        {
            if (IsOpen())
            {
                Close();
            }
        }
    }
    catch (Pylon::GenericException  err)
    {
        std::cerr << "Caught exception on destruction of camera: "
                  << err.GetDescription() << std::endl;
    }
    delete mCamera;
}

bool BaslerCamera::IsReadyForTrigger() const
{
    mCamera->AcquisitionStatusSelector.SetValue( Basler_UniversalCameraParams::AcquisitionStatusSelector_FrameBurstTriggerWait );
    return mCamera->AcquisitionStatus.GetValue();
}

void BaslerCamera::SetupHardwareTrigger()
{
    mHardwareTriggered = true;

    bool was_open = IsOpen();
    if (was_open)
    {
        Close();
    }
    mMutex.lock();
    mCamera->RegisterConfiguration( new Pylon::CHardwareTriggerConfiguration,
                                    Pylon::RegistrationMode_ReplaceAll,
                                    Pylon::Cleanup_Delete );
    mMutex.unlock();
    if (was_open)
    {
        Open();
        mCamera->AcquisitionBurstFrameCount.SetValue( 255 );
        assert( 255 == mCamera->AcquisitionBurstFrameCount.GetValue() );
    }
    else
    {
        Open();
        mCamera->AcquisitionBurstFrameCount.SetValue( 255 );
        assert( 255 == mCamera->AcquisitionBurstFrameCount.GetValue() );
    }

//    // Set the acquisition mode to continuous (the acquisition mode must
//    // be set to continuous when frame burst start triggering is on)
//    camera->AcquisitionMode.SetValue( Basler_UsbCameraParams::AcquisitionMode_Continuous );
//    assert( camera->AcquisitionMode.GetValue() == Basler_UsbCameraParams::AcquisitionMode_Continuous);
//    // Select the frame burst start trigger
//    camera->TriggerSelector.SetValue(Basler_UsbCameraParams::TriggerSelector_FrameStart);
//    assert( camera->TriggerSelector.GetValue() == Basler_UsbCameraParams::TriggerSelector_FrameStart);
//    // Set the mode for the selected trigger
//    camera->TriggerMode.SetValue( Basler_UsbCameraParams::TriggerMode_Off );
//    assert( camera->TriggerMode.GetValue() == Basler_UsbCameraParams::TriggerMode_Off  );

//    camera->TriggerSelector.SetValue(Basler_UsbCameraParams::TriggerSelector_FrameBurstStart);
//    assert(camera->TriggerSelector.GetValue() == Basler_UsbCameraParams::TriggerSelector_FrameBurstStart);
//    // Set the mode for the selected trigger
//    camera->TriggerMode.SetValue( Basler_UsbCameraParams::TriggerMode_On );
//    assert( camera->TriggerMode.GetValue() == Basler_UsbCameraParams::TriggerMode_On );

//    // Set the source for the selected trigger
//    camera->TriggerSource.SetValue( Basler_UsbCameraParams::TriggerSource_Line3 );
//    assert( camera->TriggerSource.GetValue() == Basler_UsbCameraParams::TriggerSource_Line3 );
//    // Set the activation mode for the selected trigger to rising edge
//    camera->TriggerActivation.SetValue( Basler_UsbCameraParams::TriggerActivation_RisingEdge );
//    assert( camera->TriggerActivation.GetValue() == Basler_UsbCameraParams::TriggerActivation_RisingEdge );
    // Set the acquisition burst frame count

}

void BaslerCamera::SetupSoftwareTrigger()
{
    mHardwareTriggered = false;
    if (IsOpen())
    {
        Close();
    }
    mMutex.lock();
    mCamera->RegisterConfiguration( new Pylon::CAcquireContinuousConfiguration,
                                    Pylon::RegistrationMode_ReplaceAll,
                                    Pylon::Cleanup_Delete );
    mMutex.unlock();
    Open();

    mCamera->TriggerSelector.SetValue(Basler_UniversalCameraParams::TriggerSelector_FrameBurstStart);
    assert(mCamera->TriggerSelector.GetValue() == Basler_UniversalCameraParams::TriggerSelector_FrameBurstStart);

    mCamera->TriggerMode.SetValue( Basler_UniversalCameraParams::TriggerMode_Off );
    assert( mCamera->TriggerMode.GetValue() == Basler_UniversalCameraParams::TriggerMode_Off );
}

bool BaslerCamera::MakeConnect()
{
    QMutexLocker locker(&mMutex);
    try
    {
        Pylon::CTlFactory& TlFactory = Pylon::CTlFactory::GetInstance();
        Pylon::CDeviceInfo* tmp = dynamic_cast<BaslerCamInfo*>(mCamInfo.get());
        mCamera->Attach( TlFactory.CreateDevice( *tmp ) );
        mIsConnected = true;
    }
    catch (Pylon::GenericException  &e)
    {
        qDebug("An exception occurred:");
        qDebug("%s", e.GetDescription());
    }
    catch (...)
    {
        qDebug("An exception occurred during camera connection.");
    }

    return mIsConnected;
}

double BaslerCamera::Hz()
{
    QMutexLocker locker( &mMutex );
    double hz = 0;
    if (IsOpen())
    {
        hz = mCamera->ResultingFrameRate.GetValue();
    }
    else
    {
        Open();
        hz = mCamera->ResultingFrameRate.GetValue();
        Close();
    }
    return hz;
}

void BaslerCamera::Open()
{
    if (!mIsConnected)
    {
        MakeConnect();
    }

    try
    {
        QMutexLocker locker(&mMutex);
        mCamera->Open();
        if (IsOpen())
        {
            mNodeMap = &(mCamera->GetNodeMap());
            if (GenApi::IsWritable(mCamera->ChunkModeActive))
            {
                mCamera->ChunkModeActive.SetValue( true );
                mCamera->ChunkEnable.SetValue( true );
                mCamera->ChunkSelector.SetValue( Basler_UniversalCameraParams::ChunkSelector_Timestamp );
            }
            else
            {
                throw RUNTIME_EXCEPTION("The camera doesn’t support chunk features. ");
            }
        }
        else
        {
            std::fprintf(stderr, "Major Error during open of camera.\n");
        }
    }
    catch (const Pylon::GenericException &e)
    {
        //qDebug("An exception occurred during camera opening.");
        std::cerr << "An exception occurred." << std::endl << e.GetDescription() << std::endl;
    }
}

void BaslerCamera::Close()
{
    QMutexLocker locker( &mMutex );

    mCamera->Close(); // should not throw exceptions
    mIsOpened = IsOpen();
    mIsConnected = false;
    mCamera->DestroyDevice();
}


bool BaslerCamera::CheckForWaitingImage(  ) //returns false on timeout
{
    return mCamera->RetrieveResult( mImageRetrieveTimeout,
                                    mpGrabResult,
                                    Pylon::TimeoutHandling_Return);
}

GuiImage BaslerCamera::RetrieveWaitingImage( std::string& oError )
{
    if (mpGrabResult->GrabSucceeded())
    {
        uint64_t record_count = IncreaseCount();

        uint64_t new_img_time_ns = mpGrabResult->ChunkTimestamp.GetValue();
        WriteImageInfoToLog( record_count, new_img_time_ns );

        cv::Mat cv_img_bayerbg8 = cv::Mat( mpGrabResult->GetHeight(),
                                           mpGrabResult->GetWidth(),
                                           CV_8UC1,
                                           (uint8_t*)mpGrabResult->GetBuffer());
        cv::Mat cv_bgr8_image;
        cv::Mat cv_rgb8_image;
        //:TRICKY: CV_BayerBG2RGB and CV_BayerBG2BGR are the same enum value, i.e. CV_BayerBG2BGR
        // so we have to further convert to RGB.
        cv::cvtColor( cv_img_bayerbg8, cv_bgr8_image, CV_BayerBG2RGB );
        cv::cvtColor( cv_bgr8_image, cv_rgb8_image, CV_BGR2RGB );
        return GuiImage( "", mIdx, record_count, cv_rgb8_image, true, new_img_time_ns );
    }
    else
    {
        oError = mpGrabResult->GetErrorDescription().c_str();
        return GuiImage("", -1, -1, cv::Mat(), false, 0);
    }
}


void BaslerCamera::StartGrabbingRecord( QString iDir, int iNumImages )
{
    assert( 0 <= iNumImages );
    if ( 0 == iNumImages )
    {
        //Have to send these two signals so the book keeping on the RecordDialog functions correctly.
        emit SignalGrabbing();
        emit SignalFinished();
        return;
    }
    std::cout << "Starting Recording on cam " << mIdx << std::endl;
    Pylon::CImageFormatConverter fc;
    QString save_dir = ImageOutputDir( iDir );
    QDir dir(save_dir);
    if (!dir.exists())
    {
        dir.mkpath(".");
    }
    try
    {
        mIsStopping = false;
        mRecordCount = 0;
        if (mHardwareTriggered)
        {
            mCamera->StartGrabbing(iNumImages);
        }
        else
        {
            mCamera->StartGrabbing(iNumImages, Pylon::GrabStrategy_OneByOne);
        }
    }
    catch (Pylon::GenericException  err)
    {
        std::cerr << "Did you hotplug/unplug a camera? " << err.GetDescription() << std::endl;
        CleanUpFileWrites();
        emit SignalRefreshCameras();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        return;
    }
    emit SignalGrabbing();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
    InitFileWrites( save_dir );

    GrabAndSaveImageTask*  pool_task;

    pool_task = new GrabAndSaveImageTask( this, mStream, iNumImages, save_dir, mMakeVideoOuput );

    pool_task->setAutoDelete( true );
    assert(pool_task->autoDelete());
    QObject::connect( pool_task, SIGNAL(finished()), this, SLOT(GrabThreadDone()) );
    QThreadPool::globalInstance()->start( pool_task );
    IncrementGrabThreads();

}

void BaslerCamera::GrabThreadDone()
{
    if  (DecrementGrabThreads() == 0)
    {
        std::cerr << "Frames dropped for cam " << mIdx << ":" << mDropCount <<  std::endl;
        mCamera->StopGrabbing();

        CleanUpFileWrites();

        emit SignalFinished();
    }
}

uint64_t BaslerCamera::IncreaseCount( )
{
    QWriteLocker lock(&mRwLock);
    emit SignalImageSaved();
    return mRecordCount++; //Yes, this one should return the value before incrementing.
}

uint64_t BaslerCamera::IncreaseDropCount()
{
    QWriteLocker lock(&mRwLock);
    emit SignalImageDropped();
    return ++mDropCount;
}

void BaslerCamera::StartGrabbingPreview()
{
    std::cerr << "StartGrabbing() called for cam " << mIdx << std::endl;
    mIsStopping = false;
    bool undo_trigger = mHardwareTriggered;

    try
    {
        if (undo_trigger)
        {
            SetupSoftwareTrigger();
        }
    }
    catch (Pylon::GenericException  err)
    {
        std::cerr << "Did you hotplug/unplug a camera? " << err.GetDescription() << std::endl;
        CleanUpFileWrites();
        emit SignalRefreshCameras();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        return;
    }

    InitFileWrites( );
    try
    {
        mCamera->StartGrabbing( Pylon::GrabStrategy_LatestImageOnly );
    }
    catch (Pylon::GenericException  err)
    {
        std::cerr << "Did you hotplug/unplug a camera? " << err.GetDescription() << std::endl;
        CleanUpFileWrites();
        emit SignalRefreshCameras();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        return;
    }
    emit SignalGrabbing();
    try
    {
        while (!IsStopped())
        {
            if (CheckForWaitingImage())
            {
                std::string error;
                GuiImage im = RetrieveWaitingImage( error );
                if (!im.IsEmpty())
                {
                    emit SignalGrabDone(mIdx, im.ToQPixmap());
                }
            }
            //This stops the while loop if a stopGrab event has been sent but is still in the queue.
            QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        }
        mCamera->StopGrabbing();
        QThread::sleep( 1 );
        if  (undo_trigger)
        {
            SetupHardwareTrigger();
        }
    }
    catch (Pylon::GenericException  err)
    {
        std::cerr << "Did you hotplug/unplug a camera? " << err.GetDescription() << std::endl;
        CleanUpFileWrites();
        emit SignalRefreshCameras();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        return;
    }
    CleanUpFileWrites();
    emit SignalFinished();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
}

void BaslerCamera::UpdateConfig( const std::string& iGenApiConfigFile )
{
    Pylon::String_t file( iGenApiConfigFile.c_str() );
    Pylon::CFeaturePersistence::Load( file, mNodeMap, true );
}


const QString BaslerCamera::FirmwareLevel()
{
    QString level = "";
    if (IsOpen())
    {
        level = mCamera->DeviceFirmwareVersion.GetValue().c_str();
    }
    else
    {
        Open();
        level = mCamera->DeviceFirmwareVersion.GetValue().c_str();
        Close();
    }
    return level;
}

void BaslerCamera::StopGrabbing()
{
    mIsStopping = true;
}

bool BaslerCamera::IsConnected()
{
    mIsConnected = mCamera->IsPylonDeviceAttached(); // does not throw exceptions
    return mIsConnected;
}

bool BaslerCamera::IsOpen()
{
    if (mCamera == NULL)
    {
        return false;
    }
    mIsOpened = mCamera->IsOpen(); // does not throw exceptions
    return mIsOpened;
}

bool BaslerCamera::IsHardwareTriggered()
{
    return mHardwareTriggered;
}

bool BaslerCamera::IsStopped()
{
    QMutexLocker locker(&mMutex);
    return mIsStopping;
}

bool BaslerCamera::IsStoppedNonMutex()
{
    return mIsStopping;
}

int64_t BaslerCamera::Height()
{
    if (!mIsOpened)
    {
        Open();
    }
    const GenApi::CIntegerPtr map = mNodeMap->GetNode("Height");
    return map->GetValue();
}

int64_t BaslerCamera::Width()
{
    if (!mIsOpened)
    {
        Open();
    }
    const GenApi::CIntegerPtr map = mNodeMap->GetNode("Width");
    return map->GetValue();
}

const QString BaslerCamera::Name() const
{
    return QString( mCamInfo->GetFriendlyName().c_str() );
}

int64_t BaslerCamera::MaxBuffer() const
{
    return mCamera->MaxNumBuffer.GetValue();
}

void BaslerCamera::MaxBuffer( int64_t iNewNum )
{
    mCamera->MaxNumBuffer.SetValue( iNewNum );
}

int BaslerCamera::CamIdx() const
{
    return mIdx;
}

const GenApi::INodeMap* BaslerCamera::NodeMap() const
{
    return mNodeMap;
}



