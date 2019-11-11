#include <vector>
#include <memory>
#include <dlfcn.h>
#include <QThread>
#include <QCoreApplication>
#include <QEventLoop>

#include "GuiDevice.h"
#include "MainWindow.h"

#include "GuiCamArray.h"


GuiCamArray::GuiCamArray( QObject* ipParent, const QString& iCamLibPath )
    : QObject(ipParent),
      mpParent(ipParent),
      mStopped(true),
      mConnected(false),
      mOpened(false),
      mpLibHndl(NULL),
      mCamLibPath( iCamLibPath )
{ }

GuiCamArray::~GuiCamArray()
{
    for (uint32_t i =0; i < mCamList.size(); ++i)
    {
        mCamList[i]->StopGrabbing();
        //emit camList[i]->signalFinished();
        //delete camera; don't need to delete unique_ptr
    }
    mCamList.clear();
    foreach (QThread* thread, mCamThreads)
    {
        thread->exit(0);
        thread->wait();
        thread->deleteLater(); // <-- ensures that it will be deleted later when the (main?) event loop executes again
    }
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
    mCamThreads.clear();
}

void GuiCamArray::PopulateCams(QStringList& oCameraNames)
{
    oCameraNames.clear();
    if (NULL != mpLibHndl)
    {
        void* new_cam_mkr_destroy = dlsym( mpLibHndl, "CamMakerDestroyResources");
        if (NULL == new_cam_mkr_destroy)
        {
            std::cerr << "The CamMakerDestroyResources function does not exist in the supplied library." << std::endl;
        }
        else
        {
            //:TRICKY: The following two lines can look very confusing. The reason is that we aren't
            // allowed to cast a void* to a function*. So in order to do that, we instead have to do this
            // referencing and dereferencing workaround.
            void* (*new_cam_destroy_ptr)();
            *(void**)(&new_cam_destroy_ptr) = new_cam_mkr_destroy;
            new_cam_destroy_ptr();
        }
        if (0 != dlclose(mpLibHndl))
        {
            std::cerr << "Error closing previous dynamic library handle." << std::endl;
        }
    }

    mpLibHndl = dlopen(mCamLibPath.toStdString().c_str(), RTLD_LAZY);
    if (NULL == mpLibHndl)
    {
        std::cerr << "Could not load the requested camera library." << std::endl;
        std::cerr << dlerror() << std::endl;
        return;
    }

    void* new_cam_mkr_init = dlsym( mpLibHndl, "CamMakerInitResources");
    if (NULL == new_cam_mkr_init)
    {
        std::cerr << "The CamMakerInitResources function does not exist in the supplied library." << std::endl;
        return;
    }
    //:TRICKY: The following two lines can look very confusing. The reason is that we aren't
    // allowed to cast a void* to a function*. So in order to do that, we instead have to do this
    // referencing and dereferencing workaround.
    void* (*new_cam_init_ptr)();
    *(void**)(&new_cam_init_ptr) = new_cam_mkr_init;
    new_cam_init_ptr();

    void* device_factory_mkr = dlsym( mpLibHndl, "DeviceMaker" );
    if (NULL == device_factory_mkr)
    {
        std::cerr << "The DeviceMaker function does not exist in the supplied library." << std::endl;
        return;
    }

    //:TRICKY: The following three lines can look very confusing. The reason is that we aren't
    // allowed to cast a void* to a function*. So in order to do that, we instead have to do this
    // referencing and dereferencing workaround.
    GuiDevice* (*device_factory_ptr)();
    *(void**)(&device_factory_ptr) = device_factory_mkr;
    GuiDevice* device_factory = device_factory_ptr();

    std::vector< std::unique_ptr< GuiCamInfo > > device_info_list = device_factory->EnumerateDevices();
    std::vector< QString > firmware_levels;

    void* new_cam_mkr = dlsym( mpLibHndl, "CamMaker");
    if (NULL == new_cam_mkr)
    {
        std::cerr << "The CamMaker function does not exist in the supplied library." << std::endl;
        return;
    }

    //:TRICKY: The following two lines can look very confusing. The reason is that we aren't
    // allowed to cast a void* to a function*. So in order to do that, we instead have to do this
    // referencing and dereferencing workaround.
    GuiCamera* (*new_cam_ptr)(std::unique_ptr<GuiCamInfo>&, const int, QObject*);
    *(void**)(&new_cam_ptr) = new_cam_mkr;

    for (uint32_t i =0; i < device_info_list.size(); ++i)
    {
        std::unique_ptr<GuiCamera> p_camera( new_cam_ptr(device_info_list[i], i, nullptr));
        QStringList split_string = p_camera->FirmwareLevel().split(";");
        if (p_camera->IsHardwareTriggered())
        {
            p_camera->SetupHardwareTrigger();
        }
        else
        {
            p_camera->SetupSoftwareTrigger();
        }
        uint32_t split_len = split_string.size();
        firmware_levels.push_back( split_string[ split_len - 3 ] +
                                   "_" +
                                   split_string[ split_len - 2 ] +
                                   "_" +
                                   split_string[ split_len - 1 ] );

        QThread* thread = new QThread(mpParent);
        p_camera->moveToThread( thread );
        mCamThreads.push_back( thread );

        QObject::connect( &*p_camera,
                          SIGNAL(SignalGrabDone(int, QPixmap)),
                          (MainWindow*)mpParent,
                          SLOT(UpdateCamPreviewWindow(int,QPixmap)));

        bool ret = connect( this,
                            SIGNAL( SignalStartGrabbingPreviewAll() ),
                            &*p_camera,
                            SLOT( StartGrabbingPreview() ) );
        if (!ret)
        {
            std::cerr << "Error creating startGrabbingAll() connection." << std::endl;
        }

        ret = connect( this,
                       SIGNAL( SignalStartGrabbingRecordAll( QString, int ) ),
                       &*p_camera,
                       SLOT( StartGrabbingRecord( QString, int ) ) );
        if (!ret)
        {
            std::cerr << "Error creating startGrabbingAll() connection." << std::endl;
        }

        ret = connect( this,
                       SIGNAL( SignalStopGrabbingAll() ),
                       &*p_camera,
                       SLOT( StopGrabbing() ) );
        if (!ret)
        {
            std::cerr << "Error creating stopGrabbingAll() connection." << std::endl;
        }

        ret = connect( &*p_camera, SIGNAL(SignalRefreshCameras()), mpParent, SLOT(RefreshCameras()));
        if (!ret)
        {
            std::cerr << "Error createing RefreshCameras() connection." << std::endl;
        }
        oCameraNames.append( p_camera->Name() );
        mCamList.push_back( std::move(p_camera) );
        thread->start();
    }

    for (uint32_t i=1; i < firmware_levels.size() && firmware_levels.size() > 1; ++i)
    {
        if (firmware_levels[0] != firmware_levels[i])
        {
            QMessageBox msg_box;
            QString output = "This may not be an issue, though it can mean\n "
                             "that the cameras might have different  \n"
                             "configurations available which could cause\n"
                             "unforseen problems.\n"
                             "---------------------------------------------\n";
            for (uint32_t j=0; j < mCamList.size(); ++j)
            {
                output += "Camera " +
                          QString::number( mCamList[j]->CamIdx() ) +
                          " has " +
                          mCamList[j]->FirmwareLevel() +
                          "\n";
            }
            msg_box.setText( output );
            msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
            msg_box.setWindowTitle("Firmware of cameras doesn't match!!");
            msg_box.exec();
            break;
        }
    }
}

bool GuiCamArray::GetCamera( const uint32_t iCamIndex, GuiCamera*& oCamera )
{
    if (iCamIndex < mCamList.size())
    {
        oCamera = &*(mCamList[iCamIndex]);
        return true;
    }
    return false;
}

void GuiCamArray::StartGrabbingRecord( const QString& iDir, int iNumImages )
{
    mStopped = false;
    std::cerr << "Telling all cams to start recording...numCams=" << mCamList.size() << std::endl;
    emit SignalStartGrabbingRecordAll( iDir, iNumImages );
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
}

void GuiCamArray::StartGrabbingPreview()
{
    mStopped = false;
    std::cerr << "Telling all cams to start previewing...numCams=" << mCamList.size() << std::endl;
    emit SignalStartGrabbingPreviewAll();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
}

void GuiCamArray::StopGrabbing()
{
    std::cerr << "Telling all cams to stop grabbing...numCams=" << mCamList.size() << std::endl;
    emit SignalStopGrabbingAll();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
    mStopped = true;
}

int GuiCamArray::Size()
{
    return mCamList.size();
}

void GuiCamArray::Open()
{
    mOpened = true;
    for (uint32_t i = 0; i < mCamList.size(); ++i)
    {
        mCamList[i]->Open();
    }
}

void GuiCamArray::Close()
{
    mOpened = false;
    mConnected = false;
    mStopped = true;
    for (uint32_t i = 0; i < mCamList.size(); ++i)
    {
        mCamList[i]->Close();
    }
}

void GuiCamArray::MakeConnect()
{
    mConnected = true;
    for (uint32_t i = 0; i < mCamList.size(); ++i)
    {
        mCamList[i]->MakeConnect();
    }
}

void GuiCamArray::SetMaxBuffer( int64_t iNumImages )
{
    for (uint32_t i = 0; i < mCamList.size(); ++i)
    {
        mCamList[i]->MaxBuffer(iNumImages);
    }
}

void GuiCamArray::SetRetrievalTimeout( uint64_t iTimeoutMilSecs )
{
    for (uint32_t i = 0; i < mCamList.size(); ++i)
    {
        mCamList[i]->RetrievalTimeout() = iTimeoutMilSecs;
    }
}
void GuiCamArray::OutputVideo( )
{
    for (uint32_t i = 0; i < mCamList.size(); ++i)
    {
        mCamList[i]->OutputVideo();
    }
}

void GuiCamArray::OutputImages( )
{
    for (uint32_t i = 0; i < mCamList.size(); ++i)
    {
        mCamList[i]->OutputImages();
    }
}
