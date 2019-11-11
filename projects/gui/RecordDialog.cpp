#include <QProcess>
#include <QFileDialog>
#include <QCloseEvent>
#include <iostream>
#include <QMessageBox>
#include <QThread>
#include <QDebug>
#include <QDateTime>
#include <QTextStream>
#include <QDirIterator>
#include <QTimer>
#include <exception>
#include <QDir>

#include "utils/ErrorHandling.h"
#include "CountdownDialog.h"

#include "ui_Record.h"
#include "RecordDialog.h"

RecordDialog::RecordDialog(GuiCamArray*& impCamList,
                            const ProjectDialog*& iProjDetails,
                            const RecordType& iRecordAction,
                            uint32_t iCamHerz,
                            bool iXmlAlreadyExists,
                            QWidget* ipParent)
    : QDialog(ipParent),
      mpUi(new Ui::RecordDialog),
      mpProjDetails( iProjDetails ),
      mpProcess(NULL),
      mRunCount(0),
      mBackUpImageCount(0),
      mCancelled(false),
      mRecording(false),
      mKeyPressed(false),
      mCamHerz( iCamHerz ),
      mXmlAlreadyExists( iXmlAlreadyExists ),
      mType(iRecordAction)
{
    mpUi->setupUi(this);
    mpUi->lineEdit_calibrateImage->setText(mpProjDetails->ProjectDir() +
                                    QDir::separator() +
                                    mpProjDetails->ProjectCalibImage());

    mpUi->cameraHz->setReadOnly( true );
    mpUi->cameraHz->setText( QString::number( mCamHerz ) );
    on_numImages_valueChanged(0);

    mpCamList = impCamList;
    mpFileDialog = new QFileDialog( this );
    //mpFileDialog->setFileMode( QFileDialog::ExistingFile );
    mpFileDialog->setOption( QFileDialog::ShowDirsOnly );

    mpBufDialog = new BufferingDialog( this );
    mpBufDialog->setModal( true );
    mpBufDialog->setWindowTitle( "Saving Images...");
    connect( mpBufDialog,
             SIGNAL( QuitSavingImages() ),
             this,
             SLOT( on_stopRecordingButton_clicked() ));

    //cmdlinempProcess = new QmpProcess( this );
    mpCapturingMsg = new QMessageBox();
    mpCapturingMsg->setText("Capturing...");
    mpCapturingMsg->setStandardButtons(0);
    mpCapturingMsg->setWindowIcon(QIcon("snake_icon_inv.jpg"));
    mpCapturingMsg->setWindowTitle("FYI");
    mpCapturingMsg->setModal(false);
    mpCapturingMsg->setWindowFlags(mpCapturingMsg->windowFlags() | Qt::WindowStaysOnTopHint);


    for (int i=0; i < mpCamList->Size(); ++i)
    {
        connect( &(*mpCamList)[i],
                 SIGNAL( SignalFinished() ),
                 this,
                 SLOT( UpdateCamDoneGrabbing() ));
        connect( &(*mpCamList)[i],
                 SIGNAL( SignalGrabbing() ),
                 this,
                 SLOT( UpdateCamStartGrabbing() ));
        connect( &(*mpCamList)[i],
                 SIGNAL( SignalImageSaved() ),
                 mpBufDialog,
                 SLOT( IncrementImageSavedCount() ));
        connect( &(*mpCamList)[i],
                 SIGNAL( SignalImageDropped() ),
                 mpBufDialog,
                 SLOT( IncrementImageDroppedCount() ));
    }
    connect( this,
             SIGNAL(SignalCreateCVConfigFile( QString )),
             this,
             SLOT(CreateCVConfigFile( QString )) );
    ConfigureDialogForType();
}

RecordDialog::~RecordDialog()
{
    while (mRunCount > 0)
    {
        QThread::msleep( 200 );

        //still need to process events otherwise we are in a dead loop
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
    }
    if (NULL != mpProcess)
    {
        delete mpProcess;
    }
    delete mpUi;
}

void RecordDialog::keyPressEvent ( QKeyEvent * event )
{
    if (NULL != event)
    {
        //:KLUDGE: here only so that the function signature doesn't need to change
    }
    mKeyPressed = true;
}

void RecordDialog::closeEvent(QCloseEvent *event)
{
    event->ignore();
    if (mRunCount != 0)
    {
        QMessageBox msg_box;
        msg_box.setText( "Cannot exit while mRecording.");
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Error!");
        msg_box.exec();
    }
    else
    {
        hide();
    }
}

void RecordDialog::RecordingDone()
{
    //Display dialog showing buffering progress
    mpCapturingMsg->hide();
    if (!mCancelled)
    {
        if (mRecording && mRunCount != 0)
        {
            mpBufDialog->show();
        }
    }
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
}

void RecordDialog::on_numImages_valueChanged(int iArg)
{
    if (iArg > 0)
    {
        //:KLUDGE: here so that the function signature doesn't need to change
    }
    if (mCamHerz > 0)
    {
        if (mpUi->numImages->value() > 0)
        {
            const double secs = mpUi->numImages->value() /
                              static_cast<double>(mCamHerz + std::numeric_limits<float>::epsilon());
            mpUi->labelCaptureNumSeconds->setText( QString::number( secs , 'f', 1 ) + " seconds");
        }
        else
        {
            mpUi->labelCaptureNumSeconds->setText( QString::number( 0 , 'f', 1 ) + " seconds");
        }
    }
    else
    {
       mpUi->labelCaptureNumSeconds->setText( "\xE2\x88\x9E seconds");
    }

}

void RecordDialog::FreezeDialog( bool iFreeze )
{
    mpUi->stopRecordingButton->setEnabled( iFreeze );
    mpUi->startRecordingButton->setEnabled( !iFreeze );
    mpUi->checkBox_makeVideo->setEnabled( !iFreeze );

    if (MASK != mType)
    {
        mpUi->numImages->setEnabled( !iFreeze );
    }
}

void RecordDialog::on_startRecordingButton_clicked()
{
    bool make_video = mpUi->checkBox_makeVideo->isChecked() && DATA == mType;
    if ( mRunCount < 0 )
    {
        //There must have been a few quick clicks on start preview and then record, so we have
        //some threads that need to finish up before we can start processing. Just ignore this click.
        return;
    }
    if (mRunCount > 0)
    {
        mpCamList->StopGrabbing();
        while (mRunCount > 0)
        {
            QThread::msleep( 200 );

            //still need to process events otherwise we are in a dead loop
            QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        }
    }
    std::exception_ptr pException;
    try
    {
        mpCamList->SetMaxBuffer( mpUi->numImages->value() );
    }
    catch (...)
    {
        if (0 == mpUi->numImages->value())
        {
            return;
        }
        else
        {
            pException = std::current_exception();
            HandleException( pException );
        }
    }

    QString cal_image = mpProjDetails->ProjectDir() +
                        QDir::separator() +
                        mpProjDetails->ProjectCalibImage();
    std::cerr << "CalImage: '" << cal_image.toUtf8().constData() << "'" << std::endl;
    if (CALIBRATION == mType && ( cal_image.size() == 0 || !QFile( cal_image ).exists() ))
    {
        std::cerr << "FAIL: " << cal_image.toStdString() << std::endl;

        QMessageBox msg_box;
        msg_box.setText( "You must supply a valid calibration image file.");
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Error!");
        msg_box.exec();
        return;
    }

    if (DATA == mType)
    {
        QDir().mkdir( mpUi->lineEdit_outputDir->text() );
    }

    QString outputDir = mpUi->lineEdit_outputDir->text();
    QDir().mkdir( mpUi->lineEdit_outputDir->text() );
    if (0 == mpCamList->Size())
    {
        QMessageBox msg_box;
        msg_box.setText( "You need to attach a camera to the system in order to record.");
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Error!");
        msg_box.exec();
        return;
    }
    std::cerr << "Output dir: '" << outputDir.toUtf8().constData() << "'" << std::endl;

    CountdownDialog* timer = new CountdownDialog( );
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);

    mMutex.lock();
    FreezeDialog( true );
    mRecording = true;
    double hz = mpUi->cameraHz->text().toDouble();
    double seconds = mpUi->numImages->value() / hz;

    mpBufDialog->ResetValues( 0, mpUi->numImages->value() * mpCamList->Size());
    mMutex.unlock();

    if (mpUi->radioButton_hardwareTrigger->isChecked())
    {
        if (make_video)
        {
            mpCamList->OutputVideo();
        }
        else
        {
            mpCamList->OutputImages();
        }
        mpCamList->StartGrabbingRecord( outputDir, mpUi->numImages->value() );

        for (int i=0; i < mpCamList->Size(); ++i)
        {
            int count = 0;
            while( false == (*mpCamList)[i].IsReadyForTrigger() && count < 4 )
            {
                 QThread::msleep( 200 );
                 ++count;
            }
            if (count >= 8)
            {
                fprintf(stderr, "Error, cam wasn't ready for hardware trigger.\n");
                QMessageBox msg_box;
                msg_box.setText( "One or more cameras had trouble getting into a waiting state for the hardware trigger. Try mRecording again.");
                msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
                msg_box.setWindowTitle("Error!");
                msg_box.exec();

                FreezeDialog( false );
                mRecording = false;

                if (DATA == mType)
                {
                    QDir tmp(outputDir);
                    tmp.removeRecursively();
                }
                return;
            }
        }

        if (NULL != mpProcess)
        {
            delete mpProcess;
        }
        mpProcess = new QProcess(this);

        mpProcess->setProcessChannelMode(QProcess::MergedChannels);

        QString shellfilename = mpProjDetails->ProjectDir() +
                                QDir::separator() +
                                mpProjDetails->ProjectTriggerScript();
        QFileInfo shell_fileinfo( shellfilename );
        QString shell_suffix = shell_fileinfo.suffix();

        QString default_script = mpProjDetails->ProjectDir() +
                                QDir::separator() +
                                DEFAULT_SCRIPT_DIR +
                                QDir::separator() +
                                DEFAULT_TRIGGER_SCRIPT;

        QStringList args;
        if ( shellfilename == default_script)
        {
            args << shellfilename <<  QString::number(seconds+1);
        }
        else
        {
            //:TODO: Need a way for the user to be able to pass this in without having to rebuild
            //If your script parameters are different than the default, you can modify them here.
        }

        if (mpUi->checkBox_keyPressStart->isChecked())
        {
            QMessageBox msg;
            msg.setText("Waiting For Key Press to start capture...");
            msg.setStandardButtons(0);
            msg.setWindowIcon(QIcon("snake_icon_inv.jpg"));
            msg.setWindowTitle("FYI");
            //msg.show();
            mKeyPressed = false;
            while (!mKeyPressed)
            {
                QThread::msleep( 100 );
                QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
            }
        }
        else
        {
            //:TODO: Allow user to change timer countdown value
            if (!timer->Countdown( 5 ))
            {
                delete timer;
                return;
            }
        }

        double wait = seconds * 1000;
        QTimer::singleShot( int(wait), this, SLOT( RecordingDone() ));
        mpCapturingMsg->show();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);

        if ( shell_suffix == "py")
        {
            mpProcess->start( "python", args );
        }
        else
        {
            //:TODO: allow users to not be confined to using a python script
            //If your script isn't a python script you can change that here.
        }

        delete timer;
    }
    else{
        if (mpUi->checkBox_keyPressStart->isChecked())
        {
            mKeyPressed = false;
            while (!mKeyPressed)
            {
                QThread::msleep( 100 );
                QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
            }
        }
        else
        {
            //:TODO: Allow users to change this countdown value
            if (!timer->Countdown( 5 ))
            {
                delete timer;
                return;
            }
        }
        float wait = seconds * 1000;
        QTimer::singleShot( int(wait), this, SLOT( RecordingDone() ));
        mpCapturingMsg->show();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        mpCamList->StartGrabbingRecord( outputDir, mpUi->numImages->value() );
        delete timer;
    }
}

void RecordDialog::on_stopRecordingButton_clicked()
{
    mMutex.lock();
    mCancelled = true;
    mpUi->stopRecordingButton->setEnabled( false );
    mMutex.unlock();

    mpCamList->StopGrabbing();
    while (mRunCount != 0)
    {
        QThread::msleep( 200 );

        //still need to mpProcess events otherwise we are in a dead loop
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
    }
    mMutex.lock();
    mRecording = false;
    mMutex.unlock();
    FreezeDialog( false );
    //try to kill code taking images.
    //cmdlinempProcess->close();
}

void RecordDialog::UpdateCamDoneGrabbing()
{
    mMutex.lock();
    std::cerr << "Done: Run Count was " << mRunCount;
    --mRunCount;
    std::cerr << " and now is " << mRunCount << std::endl;    
    mMutex.unlock();
    if (mRecording)
    {
        if (0 == mRunCount)
        {
            mMutex.lock();

            mpUi->stopRecordingButton->setEnabled( false );
            mpUi->startRecordingButton->setEnabled( true );
            mpBufDialog->hide();
            QCoreApplication::processEvents(QEventLoop::AllEvents, 1);

            QMessageBox msg_box;
            msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
            msg_box.setWindowTitle(" ");
            msg_box.setWindowFlags(msg_box.windowFlags() | Qt::WindowStaysOnTopHint);

            QString outputDir = mpUi->lineEdit_outputDir->text();
            if (mCancelled)
            {
                mCancelled = false;
                mRecording = false;
                mMutex.unlock();
                msg_box.setText( "Recording Cancelled.");
            }
            else
            {
                mRecording = false;
                mMutex.unlock();
                msg_box.setText( "All done!");
                if (DATA == mType)
                {
                    ++mCaptureCount;
                    mpUi->lineEdit_outputDir->setText( mNewCapturePath +
                                                       QString("%1").arg(mCaptureCount, 3, 10, QChar('0')));
                }

            }
            emit SignalCreateCVConfigFile( outputDir );
            QCoreApplication::processEvents(QEventLoop::AllEvents, 1);

            msg_box.exec();
            msg_box.raise();

            if (MASK != mType)
            {
                mpUi->numImages->setEnabled( true );
            }
            FreezeDialog( false );
        }
    }
}

void RecordDialog::UpdateCamStartGrabbing()
{
    mMutex.lock();
    std::cerr << "Start: Run Count was " << mRunCount;
    ++mRunCount;
    std::cerr << " and now is " << mRunCount << std::endl;
    mMutex.unlock();
}

void RecordDialog::CreateCVConfigFile( QString iOutputDir )
{
    std::cerr << "DEBUG: Output directory passed to createCVConfigFile:'"
              << iOutputDir.toStdString()
              << "'" << std::endl;
    QString path = iOutputDir + QDir::separator();
    switch ( mType )
    {
        case CALIBRATION:
            path += DEFAULT_CALIB_IMAGES_XML;
        break;
        case DATA:
            path += DEFAULT_DATA_IMAGES_XML;
        break;
        case MASK:
            path += DEFAULT_MASK_IMAGES_XML;
        break;
        default:
            path += "THIS_FILE_SHOULDNT_EXIST.xml";
    }
    QFile file( path );
    if (!file.open(QIODevice::WriteOnly))
    {
        std::cerr << "Error opening " << path.toUtf8().constData() << std::endl;
    }
    QTextStream stream( &file );
    if (CALIBRATION == mType)
    {
        stream << "<?xml version=\"1.0\"?>\n" << "<opencv_storage>\n" << "   <images>\n";
        stream <<  "        " << mpProjDetails->ProjectCalibImage() << "\n";
    }
    else
    {
        stream << "<?xml version=\"1.0\"?>\n" <<
                  "<opencv_storage>\n" <<
                  "<nCameras>" <<
                  QString::number( mpCamList->Size() ) <<
                  "</nCameras>\n" <<
                  "   <images>\n";
    }

    QDirIterator dir_it(iOutputDir,
                       QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot,
                       QDirIterator::Subdirectories);
    QDir output_dir( mpProjDetails->ProjectDir() );
    while (dir_it.hasNext())
    {
        QString dir_path = dir_it.next();
        std::cerr << "DEBUG:" << dir_path.toUtf8().constData()<< std::endl;
        if ( mpUi->checkBox_makeVideo->isChecked() && mType == DATA )
        {
            QDirIterator fileIt( dir_path,
                                 QStringList() << "*.avi", QDir::Files,
                                 QDirIterator::NoIteratorFlags);
            while (fileIt.hasNext())
            {
                QString filePath = output_dir.relativeFilePath(fileIt.next());
                stream << "        " << QDir::toNativeSeparators(filePath) << "\n";
            }
        }
        else
        {
            QDirIterator fileIt( dir_path,
                                 QStringList() << "*.png", QDir::Files,
                                 QDirIterator::NoIteratorFlags);
            while (fileIt.hasNext())
            {
                QString filePath = output_dir.relativeFilePath(fileIt.next());
                stream << "        " << QDir::toNativeSeparators(filePath) << "\n";
            }
        }
    }
    stream << "   </images>\n" << "</opencv_storage>\n";
}

void RecordDialog::on_radioButton_hardwareTrigger_clicked()
{
    QMutexLocker locker( &mMutex );
    if (mpUi->radioButton_hardwareTrigger->isChecked())
    {
        for (int i =0; i < mpCamList->Size(); ++i)
        {
            (*mpCamList)[i].SetupHardwareTrigger();
        }
    }
}

void RecordDialog::on_radioButton_softwareTrigger_clicked()
{
    QMutexLocker locker( &mMutex );
    if (mpUi->radioButton_softwareTrigger->isChecked())
    {
        for (int i =0; i < mpCamList->Size(); ++i)
        {
            (*mpCamList)[i].SetupSoftwareTrigger();
        }
    }
}

void RecordDialog::ConfigureDialogForType()
{
    setWindowTitle( "Record");

    if (CALIBRATION == mType)
    {
        mpUi->checkBox_makeVideo->setVisible( false );
        mpUi->checkBox_makeVideo->setChecked( false );
        mNewCapturePath = mpProjDetails->ProjectDir() +
                          QDir::separator() +
                          mpProjDetails->CalibDir();

        mCaptureCount = 0;
        mpUi->lineEdit_outputDir->setText( mNewCapturePath );

        mpUi->label_calibration->setEnabled( true );
        if ( mXmlAlreadyExists )
        {
            mpUi->titleLabel->setText( "Record Calibration Images (Overwrites Previous)" );
        }
        else
        {
            mpUi->titleLabel->setText( "Record Calibration Images" );
        }
    }
    else if (MASK == mType )
    {
        mpUi->checkBox_makeVideo->setVisible( false );
        mpUi->checkBox_makeVideo->setChecked( false );
        mNewCapturePath = mpProjDetails->ProjectDir() +
                          QDir::separator() +
                          mpProjDetails->MaskDir();

        mCaptureCount = 0;
        mpUi->lineEdit_outputDir->setText( mNewCapturePath );

        mpUi->label_calibration->hide();
        mpUi->lineEdit_calibrateImage->hide();

        mBackUpImageCount = mpUi->numImages->value();
        mpUi->titleLabel->setText( "Record Background/Mask Images" );

        mpUi->numImages->setValue( 1 );
        mpUi->numImages->setEnabled( false );
    }
    else if (DATA == mType)
    {
        mpUi->checkBox_makeVideo->setVisible( true );
        mpUi->checkBox_makeVideo->setChecked( true );
        mNewCapturePath = mpProjDetails->ProjectDir() +
                          QDir::separator() +
                          mpProjDetails->DataDir() +
                          QDir::separator() +
                          mpProjDetails->DataFileNameBase() + "_";
        QDirIterator it(mpProjDetails->ProjectDir() +
                        QDir::separator() +
                        mpProjDetails->DataDir(),
                        QDirIterator::Subdirectories);
        mCaptureCount = 0;
        while (it.hasNext())
        {
            QDir folder(it.next());
            QStringList name_parts_list = folder.dirName().split("_");
            if ( name_parts_list.size() >= 2)
            {
                //returns 0 on failure and should never be 0 if it is a proper folder.
                uint32_t tmp = name_parts_list[name_parts_list.size() -1].toInt();
                if ( tmp > mCaptureCount )
                {
                    mCaptureCount = tmp;
                }
            }
        }
        ++mCaptureCount;
        mpUi->lineEdit_outputDir->setText( mNewCapturePath +
                                           QString("%1").arg(mCaptureCount, 3, 10, QChar('0')));

        mpUi->label_calibration->hide();
        mpUi->lineEdit_calibrateImage->hide();

        mpUi->titleLabel->setText( "Record Data Images" );
        mpUi->numImages->setValue( mBackUpImageCount );
        mpUi->numImages->setEnabled( true );

    }
    else
    {
        //Should never reach this condition.
        assert(false);
    }
}

void RecordDialog::on_checkBox_makeVideo_stateChanged()
{
    if ( mpUi->checkBox_makeVideo->isChecked() )
    {
        mpCamList->OutputVideo();
    }
    else
    {
        mpCamList->OutputImages();
    }
}
