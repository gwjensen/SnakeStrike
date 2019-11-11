#include <QCloseEvent>
#include <QDir>
#include <QStandardPaths>
#include <QMessageBox>
#include <QProcess>
#include <QFile>
#include <QFileDialog>
#include <QXmlStreamReader>
#include <QXmlDefaultHandler>
#include <iostream>
#include <cstdlib> //system()
#include <pylon/PylonImage.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "RecordDialog.h"
#include "MainWindow.h"

#include "ui_CamCalibrate.h"
#include "CamCalibrateDialog.h"


const uint32_t DEFAULT_MINIMUM_MATCHES = 40;

CamCalibrateDialog::CamCalibrateDialog( const ProjectDialog*& iProjDetails, QWidget* ipParent)
    : QDialog(ipParent),
      mpUi(new Ui::CamCalibrateDialog),
      mCalibrationImage(""),
      mImagesXml(""),
      mNumCams(0),
      mpProcess(NULL),
      mpParent( ipParent ),
      mpProjDetails( iProjDetails ),
      mpRecordDialog( NULL )
{
    mpUi->setupUi(this);
    mpUi->textEdit->setVisible( false );
    mpUi->textEdit->setReadOnly( true );
    mpUi->textEdit->setTextInteractionFlags( Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard );

    mpUi->stopButton->setEnabled( false );
    on_checkBox_verboseoutput_toggled();

}

CamCalibrateDialog::~CamCalibrateDialog()
{
    delete mpUi;
    if( mpProcess != NULL){
        on_stopButton_clicked();
        delete mpProcess;
    }
    if (mpRecordDialog != NULL)
    {
        delete mpRecordDialog;
    }
}


void CamCalibrateDialog::closeEvent(QCloseEvent* iEvent)
{
    iEvent->ignore();
    hide();
    static_cast<MainWindow*>(mpParent)->UpdateState();
}

void CamCalibrateDialog::UpdateXMLFromRecord( const QString& iFilename)
{
    mpUi->lineEdit_inputXML->setText( iFilename + QDir::separator() + DEFAULT_CALIB_IMAGES_XML);
}

bool CamCalibrateDialog::CalibrationImagesXMLExists()
{
    QFileInfo cal_xml( mpProjDetails->ProjectDir() +
                       QDir::separator() +
                       mpProjDetails->ProjectCalibXml());
    if ( cal_xml.exists() )
    {
        mpUi->calibrateCameraButton->setEnabled( true );
        mpUi->lineEdit_inputXML->setText( cal_xml.absoluteFilePath() );
        return true;
    }
    mpUi->calibrateCameraButton->setEnabled( false );
    return false;
}

bool CamCalibrateDialog::FinishedCalibrationExists()
{
    QFileInfo cal_xml( mpProjDetails->ProjectDir() +
                       QDir::separator() +
                       mpProjDetails->FinishedCalibXml());
    if ( cal_xml.exists() )
    {
        return true;
    }
    return false;
}

void CamCalibrateDialog::FreezeDialog( bool iFreezeDialog )
{
    mMutex.lock();
    if (CalibrationImagesXMLExists())
    {
        mpUi->calibrateCameraButton->setEnabled( !iFreezeDialog );
    }
    else
    {
        mpUi->calibrateCameraButton->setEnabled( false );
    }
    mpUi->checkBox_featureextraction->setEnabled( !iFreezeDialog );
    mpUi->checkBox_verboseoutput->setEnabled( !iFreezeDialog );
    mpUi->lineEdit_inputXML->setEnabled( !iFreezeDialog );

    mpUi->spinBox_minimalMatches->setEnabled( !iFreezeDialog );
    mMutex.unlock();
    update();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
}

void CamCalibrateDialog::on_calibrateCameraButton_clicked()
{
    QString xml_file = mpUi->lineEdit_inputXML->text();
    QFileInfo fi( xml_file );
    if (0 == xml_file.size() || !QFile(xml_file).exists() || fi.suffix() != "xml")
    {
        QMessageBox msg_box;
        msg_box.setText( "You must input a valid XML file.");
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Error!");
        msg_box.exec();
        return;
    }
    QString output_dir =  mpProjDetails->ProjectDir();
    if (0 == output_dir.size() || !QDir(output_dir).exists())
    {
        QMessageBox msg_box;
        msg_box.setText( "You must input a valid output directory.");
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Error!");
        msg_box.exec();
        return;
    }

    QString calibration_exe = "example_ccalib_multi_cameras_calibration";
    QString exe_full_path = QStandardPaths::findExecutable( calibration_exe );

    if (0 == exe_full_path.size())
    {
        QMessageBox msg_box;
        msg_box.setText( "You must add the openCV exe directories to your system path "
                         "variable. Also, make sure you built the examples for ccalib.");
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Error!");
        msg_box.exec();
        return;
    }
    std::cerr << "Starting Process" << std::endl;
    FreezeDialog( true );

    if (NULL != mpProcess)
    {
        delete mpProcess;
    }

    mpProcess = new QProcess(this);
    connect( mpProcess, SIGNAL( readyReadStandardOutput() ), this, SLOT( PullNewVerboseOutput() ) );

    mpProcess->setWorkingDirectory( output_dir );

    QString args = exe_full_path + " -nc " + QString::number( mNumCams );

    args += " -pw " + QString::number( mpUi->spinBox_calWidth->value() );
    args += " -ph " + QString::number( mpUi->spinBox_calHeight->value() );
    args += " -ct 0"; //camera pinhole
    args += " -nm " + QString::number( mpUi->spinBox_minimalMatches->value() );

    if (mpUi->checkBox_featureextraction->isChecked())
    {
        args += " -fe 1";
    }
    if (mpUi->checkBox_verboseoutput->isChecked())
    {
        args += " -v 1";
    }

    args += " \"" + mpUi->lineEdit_inputXML->text() + "\"";

    mpProcess->setProcessChannelMode( QProcess::MergedChannels );
    connect( mpProcess, SIGNAL(finished(int)), this, SLOT(on_ProcessFinished(int)));
    mpUi->textEdit->clear();
    std::cerr << args.toStdString() << std::endl;

    mShellFilename = output_dir + "/.snake_tracker_cam_config.sh";
    QFile sh_file( mShellFilename );

    if (sh_file.open( QIODevice::ReadWrite ))
    {
        if (!sh_file.setPermissions(QFile::ReadOwner|
                                    QFile::WriteOwner|
                                    QFile::ExeOwner|
                                    QFile::ReadGroup|
                                    QFile::ExeGroup|
                                    QFile::ReadOther|
                                    QFile::ExeOther))
        {
            std::cerr << "Error doing file operations in " + output_dir.toStdString() << std::endl;
            QMessageBox msg_box;
            msg_box.setText( "Error doing file operations in " + output_dir );
            msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
            msg_box.setWindowTitle("Error!");
            msg_box.exec();
            FreezeDialog( false );
            return;
        }
        QTextStream stream( &sh_file );
        stream << "#!/usr/bin/env bash" << "\n";
        stream << args << "\n\n";
        sh_file.close();
        mpProcess->start( "/bin/bash", QStringList() << mShellFilename );
        //process->start( "/home/snakes/Config.sh");

    }
    else
    {
        QMessageBox msg_box;
        msg_box.setText( "Error opening file in " + output_dir );
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Error!");
        msg_box.exec();
        FreezeDialog( false );
        return;
    }

    if (mpProcess->state() == QProcess::NotRunning)
    {
        std::cerr << "The error was " <<
                     QString::number( mpProcess->error() ).toUtf8().constData() <<
                     std::endl;
    }

    mpUi->stopButton->setEnabled( true );
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
}

void CamCalibrateDialog::on_ProcessFinished(int iErrCode)
{
    mMutex.lock();
    if (mShellFilename.size() > 0)
    {
        QFile::remove( mShellFilename );
    }
    mShellFilename = "";
    mpUi->textEdit->append( "\nCalibration Finished with exit code " +
                            QString::number(iErrCode) +
                            ".\n\n");

    if (0 == iErrCode)
    {
        std::cerr <<  "\nCalibration Finished with exit code " <<
                      QString::number(iErrCode).toStdString() <<
                      ".\n\n" << std::endl;

        //:KLUDGE:Doing this weird file move because I didn't want to change the opencv
        //code and it drops the file in the current working directory.
        QDir tmp_dir( mpProjDetails->ProjectDir() );
        QString final_location = mpProjDetails->ProjectDir() +
                                 QDir::separator() +
                                 mpProjDetails->FinishedCalibXml();
        tmp_dir.remove( final_location ); //otherwise would fail to overwrite
        assert( tmp_dir.rename( DEFAULT_CALIB_DONE_XML, final_location));
    }

    mpUi->stopButton->setEnabled( false );
    mMutex.unlock();
    FreezeDialog( false );
}

void CamCalibrateDialog::on_stopButton_clicked()
{
    mMutex.lock();
    mpUi->stopButton->setEnabled( false );
    //QString pid = QString::number( mpProcess->processId() );

    if (mpUi->checkBox_featureextraction->isChecked())
    {
        system("pkill example_ccalib_");
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
    }
    // windows & linux: kill main process
    if (mpProcess->state() == QProcess::Running)
    {
        mpProcess->kill();
    }

    if (mShellFilename.size() > 0)
    {
        QFile::remove( mShellFilename );
        mShellFilename = "";
    }

    mMutex.unlock();
    FreezeDialog( false );
}

void CamCalibrateDialog::on_checkBox_verboseoutput_toggled()
{
    if (mpUi->checkBox_verboseoutput->isChecked())
    {
        mpUi->textEdit->setVisible( true );
        resize( 623, 494 );
    }
    else
    {
        mpUi->textEdit->setVisible( false );
        resize( 623, 280 );
    }
}

void CamCalibrateDialog::on_lineEdit_inputXML_textChanged()
{
    QMutexLocker locker(&mMutex);
    QString text = mpUi->lineEdit_inputXML->text();
    QFileInfo fi( text );
    if (fi.suffix() == "xml" && CalibrationImagesXMLExists())
    {
        bool ret = ProcessXMLFile( text );
        if (!ret)
        {
            std::cerr << "Error in inputXml_textChanged() slot." << std::endl;
        }
    }
}

bool CamCalibrateDialog::ProcessXMLFile( const QString& iXmlFile )
{
    QXmlStreamReader xml_reader;
    QFile file(iXmlFile);

    if (!file.open( QIODevice::ReadOnly ))
    {
        std::cerr << " Not able to open XML file. " << iXmlFile.toStdString() << std::endl;
        QMessageBox msg_box;
        msg_box.setText( "Not able to open XML file.");
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Error!");
        mpUi->lineEdit_inputXML->setText("");
        msg_box.exec();
        mNumCams = 0;
        mpUi->spinBox_numCams->setValue( mNumCams );
        return false;
    }
    xml_reader.setDevice( &file );
    QString name = xml_reader.name().toString();
    xml_reader.readNext();
    xml_reader.readNext(); // skipping xml tag
    xml_reader.readNext();//skipping the opencv_storage tag
    QStringList images;
    while (!xml_reader.isEndDocument())
    {
        if (xml_reader.isStartElement())
        {
            name = xml_reader.name().toString();
            if ("images" == name)
            {
               images = xml_reader.readElementText().split( QRegExp( "\n|\r\n|\r" ) );;
            }
            else if ("nCameras" == name)
            {
                std::cerr << " Incorrectly formatted configuration XML." << std::endl;
                QMessageBox msg_box;
                msg_box.setText( "Incorrectly formatted configuration XML.");
                msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
                msg_box.setWindowTitle("Error!");
                mpUi->lineEdit_inputXML->setText("");
                msg_box.exec();
                file.close();
                mNumCams = 0;
                mpUi->spinBox_numCams->setValue( mNumCams );
                return false;
            }
            else
            {
                std::cerr << " This configuration XML is waaaay wrong." << std::endl;
                QMessageBox msg_box;
                msg_box.setText( "This configuration XML is waaaay wrong.");
                msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
                msg_box.setWindowTitle("Error!");
                mpUi->lineEdit_inputXML->setText("");
                msg_box.exec();
                file.close();
                mNumCams = 0;
                mpUi->spinBox_numCams->setValue( mNumCams );
                return false;
            }
        }
        else if (xml_reader.isEndElement())
        {
            xml_reader.readNext();
        }
        else
        {
            xml_reader.readNext();
        }
    }
    if (xml_reader.hasError())
    {
        std::cerr << "XML error: " << xml_reader.errorString().data() << std::endl;
    }

    if (images.size() == 0)
    {
        std::cerr << " No Images in XML config." << std::endl;
        QMessageBox msg_box;
        msg_box.setText( "No Images in XML config.");
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Error!");
        mpUi->lineEdit_inputXML->setText("");
        msg_box.exec();
        file.close();
        return false;
    }

    //Process the image string for 2 things : 
    //              1) the first image is the calibration image 
    //              2) the number of cameras
    int first = 0;
    while (images[first].replace( " ", "").size() == 0)
    {
        ++first;
    }
    mCalibrationImage = mpProjDetails->ProjectDir() + QDir::separator();
    mCalibrationImage += images[first];
    mNumCams = 0;
    for (int i =first + 1; i < images.size(); ++i)
    {
        if (images[i].size() < 7)
        {
            //:WARNING: Code is based on the idea that the image files look like this '0-0.png'
            continue;
        }
        QStringList file = QDir::fromNativeSeparators( images[i] ).split( "/" );
        int cam = QString( file[ file.size() -1 ].split("-")[0] ).toInt();
        if (cam > mNumCams)
        {
            mNumCams = cam;
        }
    }

    ++mNumCams;//Have to deal with the 0 indexing

    mpUi->spinBox_numCams->setValue( mNumCams );
    QSize size = QImage( mCalibrationImage ).size();

    std::cerr << "The calibration image is : " <<
                 size.height() <<
                 " by " <<
                 size.width() <<
                 std::endl;
    mpUi->spinBox_calHeight->setValue( size.height() );
    mpUi->spinBox_calWidth->setValue( size.width() );
    return true;
}

void CamCalibrateDialog::PullNewVerboseOutput()
{
    QByteArray output = mpProcess->readAllStandardOutput();
    QStringList str_lines = QString(output).split("\n");
    QFileInfo calib_file( mpProjDetails->ProjectDir() +
                          QDir::separator() +
                          mpProjDetails->ProjectCalibImage() );
    QString config_output_filename = calib_file.path() + QDir::separator() + "configOutput.txt";
    QFile file( config_output_filename );
    file.open( QIODevice::WriteOnly | QIODevice::Append );
    QTextStream stream( &file );
    mMutex.lock();
    foreach (QString line, str_lines)
    {
        stream << line << "\n";
        if (!mpUi->checkBox_veryVerboseoutput->isChecked() &&
                ( line.contains("open image") ||
                line.contains("number of") ||
                line.contains("has too few") ||
                line.size() < 5))
        {
            continue;
        }
        mpUi->textEdit->append( line );
    }
    mMutex.unlock();
}

void CamCalibrateDialog::on_checkBox_verboseoutput_clicked()
{
    if (mpUi->checkBox_verboseoutput->isChecked())
    {
        mpUi->checkBox_veryVerboseoutput->setEnabled( true );
    }
    else
    {
        mpUi->checkBox_veryVerboseoutput->setEnabled( false );
    }
}

void CamCalibrateDialog::on_checkBox_useDefaultMinMatches_clicked()
{
    if (mpUi->checkBox_useDefaultMinMatches->isChecked())
    {
        mpUi->spinBox_minimalMatches->setValue( DEFAULT_MINIMUM_MATCHES );
        mpUi->spinBox_minimalMatches->setEnabled( false );
    }
    else
    {
        mpUi->spinBox_minimalMatches->setEnabled( true );
    }
}

void CamCalibrateDialog::on_recordCalibImagesButton_clicked()
{
    if (static_cast<MainWindow*>(mpParent)->CamHz() > DEFAULT_HERZ_CALIB_UPPER_END)
    {
        QMessageBox msg_box;
        QString output = "The frame rate of the cameras is too high. There will be many images that look similar"
                         " and this can create problems for the calibration. Lower the frame rate of the cameras to "
                         " be " + QString::number(DEFAULT_HERZ_CALIB_UPPER_END) + "Hz or under.";
        msg_box.setText( output );
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Frame rate too high!");
        msg_box.exec();
    }
    else
    {
        mpUi->textEdit->clear();

        if (NULL != mpRecordDialog)
        {
            delete mpRecordDialog;
            disconnect(this, SLOT(UpdateXMLFromRecord(QString)));
        }
        mpRecordDialog = static_cast<MainWindow*>(mpParent)->CreateRecordDialog( CALIBRATION,
                                                                                 CalibrationImagesXMLExists() );
        mpRecordDialog->show();
        connect(mpRecordDialog,
                SIGNAL(SignalCreateCVConfigFile(QString)),
                this,
                SLOT(UpdateXMLFromRecord(QString)));
    }
}
