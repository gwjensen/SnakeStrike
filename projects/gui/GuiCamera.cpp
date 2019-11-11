#include <QDir>
#include <QDateTime>
#include <iostream>
#include "GuiCamInfo.h"

#include "GuiCamera.h"

GuiCamera::GuiCamera(QObject* ipParent )
    :QObject(ipParent),
      mpParent(ipParent),
      mIdx(-1),
      mIsStopping(false),
      mIsConnected(false),
      mIsOpened(false),
      mRecordCount(0),
      mDropCount(0),
      mNumGrabThreadsRunning(0),
      mImageTimestampsFile(NULL),
      mStream(NULL),
      mHardwareTriggered(true),
      mImageRetrieveTimeout( DEFAULT_IMAGE_RETRIEVE_TIMEOUT ),
      mMakeVideoOuput(true)
{   }

GuiCamera::GuiCamera( std::unique_ptr<GuiCamInfo>& iInfo, const int iIdx, QObject* ipParent)
    :QObject(ipParent),
      mpParent(ipParent),
      mCamInfo( std::move(iInfo) ),
      mIdx(iIdx),
      mIsStopping(false),
      mIsConnected(false),
      mIsOpened(false),
      mRecordCount(0),
      mDropCount(0),
      mNumGrabThreadsRunning(0),
      mImageTimestampsFile(NULL),
      mStream(NULL),
      mHardwareTriggered(true),
      mImageRetrieveTimeout( DEFAULT_IMAGE_RETRIEVE_TIMEOUT ),
      mMakeVideoOuput(true)
{}



QString GuiCamera::ImageTimestepFileName( const QString& iDirectory ) const
{
    return  iDirectory +
            QDir::separator() +
            QString( "ImageTimestamps_cam" +
                     QString::number(mIdx)+ "_" +
                     QDateTime::currentDateTime().toString("dd.MM.yyyy.hh.mm") +
                     ".txt") ;
}

QString GuiCamera::ImageOutputDir( const QString& iBaseDir ) const
{
    return iBaseDir + QDir::separator() + "cam" + QString::number(mIdx) + QDir::separator();
}

 void GuiCamera::WriteImageInfoToLog( uint64_t iCurImageCount, uint64_t iTimestampNanoSecs )
{
    *mStream <<  QString::number(mIdx) + "_" +
                 QString::number(iCurImageCount) + ".png\t\t\t\t"
                 + QString::number(iTimestampNanoSecs) + "\n";
}

void GuiCamera::InitFileWrites( const QString& iDir )
{
    QDir dir( iDir );
    if (!dir.exists())
    {
        dir.mkpath(".");
    }

    QString path = ImageTimestepFileName( iDir );
    if (mImageTimestampsFile != NULL)
    {
        mImageTimestampsFile->close();
        delete mImageTimestampsFile;
        delete mStream;
    }
    mImageTimestampsFile = new QFile( path );
    if (!mImageTimestampsFile->open(QIODevice::ReadWrite))
    {
        std::cerr << "Error opening " << path.toStdString() << std::endl;
    }
    mStream = new QTextStream( mImageTimestampsFile );
    *mStream << "\t\tFilename\t\t\t\tTimestep Difference\n";

}

void GuiCamera::CleanUpFileWrites()
{
    if (mStream != NULL)
    {
        mStream->flush();
        delete mStream;
        mStream = NULL;
    }
    if (mImageTimestampsFile !=NULL)
    {
        mImageTimestampsFile->close();
        delete mImageTimestampsFile;
        mImageTimestampsFile = NULL;
    }
}

int& GuiCamera::RetrievalTimeout()
{
    return mImageRetrieveTimeout;
}

int GuiCamera::RetrievalTimeout() const
{
    return mImageRetrieveTimeout;
}

uint64_t GuiCamera::RecordCount()
{
    QWriteLocker lock(&mRwLock);
    return mRecordCount;
}

uint32_t GuiCamera::IncrementGrabThreads()
{
    QWriteLocker lock(&mRwLock);
    return ++mNumGrabThreadsRunning;
}

uint32_t GuiCamera::DecrementGrabThreads()
{
    QWriteLocker lock(&mRwLock);
    return --mNumGrabThreadsRunning;
}

uint64_t GuiCamera::Count() const
{
    return mRecordCount;
}

uint64_t GuiCamera::DropCount() const
{
    return mDropCount;
}

void GuiCamera::OutputVideo( )
{
    mMakeVideoOuput = true;
}

void GuiCamera::OutputImages( )
{
    mMakeVideoOuput = false;
}

