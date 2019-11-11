#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <QThread>
#include <QDir>
#include <QCoreApplication>
#include <QEventLoop>
#include <QChar>
#include <QDir>
#include <QDebug>
#include "io/File.h"

#include "GuiImage.h"

#include "GrabAndSaveImageTask.h"

GrabAndSaveImageTask::GrabAndSaveImageTask( GuiCamera* iCam,
                                            QTextStream* iStream,
                                            uint32_t iMaxNumImages,
                                            const QString& iSaveDir,
                                            bool iMakeVideo)
    : mpCamera( iCam ),
      mMaxNumImages( iMaxNumImages ),
      mSaveDir( iSaveDir ),
      mpStream( iStream ),
      mMakeVideo( iMakeVideo ),
      mpProcess( NULL )
{
    mpCamera = iCam;
    mpMutex = &(mpCamera->mMutex);
    mRecordCount = iCam->RecordCount();
    mCamIdx = iCam->CamIdx();
}

void GrabAndSaveImageTask::run( )
{
    QThread::sleep(1); //so we don't start dropping images before they're ready
    std::string error_string;

    float video_output_hz = 30;

    std::string video_filename = CreateVideoFilename(mCamIdx,
                                                     mpCamera->Width(),
                                                     mpCamera->Height(),
                                                     mMaxNumImages,
                                                     mpCamera->Hz(),
                                                     video_output_hz);
    QString path = mSaveDir + QDir::separator() + QString( video_filename.c_str() );

    cv::VideoWriter video(path.toStdString(),  cv::CAP_FFMPEG, CV_FOURCC('F','F','V','1'), video_output_hz, cv::Size(mpCamera->Width(),mpCamera->Height()));

    while (!mpCamera->IsStopped() && mpCamera->RecordCount() < mMaxNumImages)
    {
        mpMutex->lock();//This lock doesn't slow things down as Basler only allows one camera to wait for a result anyways.
        if (mpCamera->CheckForWaitingImage())
        {
            mRecordCount = mpCamera->Count(); //called here because RetrieveWaitingImage increments value
            GuiImage image = mpCamera->RetrieveWaitingImage( error_string );
            if (!image.IsEmpty())
            {
                mpMutex->unlock();

                //Save file info to file ( needs synchronization, hence the late unlock) );
                if (!mMakeVideo)
                {
                    path = mSaveDir +
                           QDir::separator() +
                           QString::number(mCamIdx) +
                           "-" +
                           QString::number(mRecordCount) +
                           ".png";

                    *mpStream <<  QString::number(mpCamera->CamIdx()) +
                                  "_" +
                                  QString::number(mRecordCount) +
                                  ".png\t\t\t\t" +
                                  QString::number(image.Timestamp()) +
                                  "\n";
                    image.SavePng( path.toStdString() );

                }
                else
                {
                    //:NOTE: opencv assumes the image is in BGR, so it converts it to RGB before saving. Since
                    // we assume the image is in RGB, we have to do this extra conversion.
                    cv::cvtColor(image, image, CV_RGB2BGR);
                    video.write( image );
                }
            }
            else
            {
                mpMutex->unlock();
                std::fprintf(stderr, "Error grabbing Image. Error'%s'\n", error_string.c_str() );
            }
        }
        else
        {
            mpMutex->unlock();
        }
        //:NOTE: This is here to stop the while loop if a stopGrab event has been sent but is still in the queue.
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
    }
    if (!mMakeVideo)
    {
        //:NOTE: The file gets created because we init it for all scopes, so if we are not writing
        //video, we need to delete it.
        QFile file (mSaveDir + QDir::separator() + QString( video_filename.c_str() ));
        file.remove();
    }

    emit finished();
}
