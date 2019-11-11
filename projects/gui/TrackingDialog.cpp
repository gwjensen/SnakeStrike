#include <QCloseEvent>
#include <QFileDialog>
#include <QColorDialog>
#include <QColor>
#include <QXmlStreamReader>
#include <QDateTime>
#include <QIODevice>
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QTextStream>
#include <QDirIterator>
#include <QThreadPool>
#include <iostream>
#include <malloc.h>

#include "common_types.h"
#include "io/File.h"
#include "TriangulationTask.h"
#include "ThresholdDialog.h"
#include "CorrespondenceDialog.h"

#include "ui_Tracking.h"
#include "TrackingDialog.h"

TrackingDialog::TrackingDialog(ProjectDialog*& iProjInfo, QWidget* ipParent)
    : QDialog(),
      mpUi(new Ui::TrackingDialog),
      mpParent( ipParent ),
      mpProjInfo( iProjInfo ),
      mpProcess( NULL ),
      mpOutputFile( NULL ),
      mRunning( false ),
      mNumCams(0),
      mpCorresDialog(NULL),
      mpTriTask(NULL)
{
    mpUi->setupUi(this);
    RefreshFileInfo();
    SetThresholdBounds( mpProjInfo->HLeftBound(),
                        mpProjInfo->SLeftBound(),
                        mpProjInfo->VLeftBound(),
                        mpProjInfo->HRightBound(),
                        mpProjInfo->SRightBound(),
                        mpProjInfo->VRightBound());
    mpUi->spinBox_maxNumPoints->setValue( mpProjInfo->NumPointsToTrack() );
    mpUi->spinBox_filterSize->setValue( mpProjInfo->FilterSize() );
    mpUi->spinBox_numIterations->setValue( mpProjInfo->FilterIterations() );
    mpUi->horizontalSlider_threshold->setValue( mpProjInfo->FilterThreshold() );
}

TrackingDialog::~TrackingDialog()
{
    std::cerr << "Destructing TrackingDialog." << std::endl;
    if (NULL != mpCorresDialog )
    {
        delete mpCorresDialog;
    }

    delete mpUi;
    //::INFO:: we have to ask politely for the C memory allocator to return memory to OS
    //otherwise it looks like we have a memory leak, when we don't.
    malloc_trim(0);
}

void TrackingDialog::RefreshFileInfo()
{
    //Refresh Calibration xml info
    if (ProcessXMLFile( mpProjInfo->ProjectDir() +
                        QDir::separator() +
                        mpProjInfo->FinishedCalibXml() ))
    {
        mpUi->spinBox_minCamsForTriangulation->setMaximum( mpUi->numCamsInFile->value());

        //Setup checkboxes for not using certain cameras in the triangulation.
        QVBoxLayout *lay = new QVBoxLayout(this);
        for (int i=0; i < mpUi->numCamsInFile->value(); ++i)
        {
            QCheckBox *dynamic = new QCheckBox("Cam " + QString::number(i));
            dynamic->setObjectName("checkBox_cam" + QString::number(i));
            dynamic->setChecked(true);
            lay->addWidget(dynamic);
        }
        mpUi->groupBox_camsForTriangulation->setLayout(lay);
    }
    else
    {
        QMessageBox msg_box;
        msg_box.setText( "There was an error with the finished calibration file. "
                        "Perhaps it is corrupted or doesn't exist?");
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Error!");
        msg_box.exec();
        mpUi->spinBox_minCamsForTriangulation->setMaximum( 99 );
    }

    //Refresh Info about data files
    PopulateDataFiles();
}

void TrackingDialog::PopulateDataFiles()
{
    mpUi->fileSelectionBox->clear();

    QString directory(mpProjInfo->ProjectDir() + QDir::separator() + mpProjInfo->DataDir());
    QDirIterator it(directory,
                    QStringList() << DEFAULT_DATA_IMAGES_XML,
                    QDir::Files, QDirIterator::Subdirectories);
    QStringList images;
    std::cerr << "---------------------------------------------------------" << std::endl;
    while (it.hasNext())
    {
        QString file = it.next();
        std::cerr << "Found Image file: " << file.toStdString() << std::endl;
        file.remove( directory ).remove( QDir::separator() ).remove( DEFAULT_DATA_IMAGES_XML );
        images << file;
    }
    std::cerr << "---------------------------------------------------------" << std::endl;

    images.sort();
    mpUi->fileSelectionBox->addItems( images );
}

void TrackingDialog::PromptSaveChanges()
{
    //The spinboxes are bounded so they can't go negative.
    if (static_cast<uint32_t>(mpUi->spinBox_HLeftbound->value())         != mpProjInfo->HLeftBound()       ||
        static_cast<uint32_t>(mpUi->spinBox_SLeftbound->value())         != mpProjInfo->SLeftBound()       ||
        static_cast<uint32_t>(mpUi->spinBox_VLeftbound->value())         != mpProjInfo->VLeftBound()       ||
        static_cast<uint32_t>(mpUi->spinBox_HRightbound->value())        != mpProjInfo->HRightBound()      ||
        static_cast<uint32_t>(mpUi->spinBox_SRightbound->value())        != mpProjInfo->SRightBound()      ||
        static_cast<uint32_t>(mpUi->spinBox_VRightbound->value())        != mpProjInfo->VRightBound()      ||
        static_cast<uint32_t>(mpUi->spinBox_maxNumPoints->value())       != mpProjInfo->NumPointsToTrack() ||
        static_cast<uint32_t>(mpUi->spinBox_filterSize->value())         != mpProjInfo->FilterSize()       ||
        static_cast<uint32_t>(mpUi->spinBox_numIterations->value())      != mpProjInfo->FilterIterations() ||
        static_cast<uint32_t>(mpUi->horizontalSlider_threshold->value()) != mpProjInfo->FilterThreshold()   )
    {
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(this, "Exiting", "Save changes to threshold settings?",
                                        QMessageBox::Yes|QMessageBox::No);
        if (QMessageBox::Yes == reply)
        {
            mpProjInfo->HLeftBound()       = mpUi->spinBox_HLeftbound->value();
            mpProjInfo->SLeftBound()       = mpUi->spinBox_SLeftbound->value();
            mpProjInfo->VLeftBound()       = mpUi->spinBox_VLeftbound->value();
            mpProjInfo->HRightBound()      = mpUi->spinBox_HRightbound->value();
            mpProjInfo->SRightBound()      = mpUi->spinBox_SRightbound->value();
            mpProjInfo->VRightBound()      = mpUi->spinBox_VRightbound->value();
            mpProjInfo->NumPointsToTrack() = mpUi->spinBox_maxNumPoints->value();
            mpProjInfo->FilterSize()       = mpUi->spinBox_filterSize->value();
            mpProjInfo->FilterIterations() = mpUi->spinBox_numIterations->value();
            mpProjInfo->FilterThreshold()  = mpUi->horizontalSlider_threshold->value();

            mpProjInfo->WriteConfigFile();
        }
    }
}

void TrackingDialog::closeEvent(QCloseEvent *event)
{
    delete mpCorresDialog;
    mpCorresDialog = NULL;
    PromptSaveChanges();
    event->accept();
    //event->ignore();
    //hide();
}

void TrackingDialog::SetThresholdBounds( uint32_t iLH,
                                         uint32_t iLS,
                                         uint32_t iLV,
                                         uint32_t iRH,
                                         uint32_t iRS,
                                         uint32_t iRV )
{
    mMutex.lock();

    mpUi->spinBox_HLeftbound->setValue( iLH );
    mpUi->spinBox_SLeftbound->setValue( iLS );
    mpUi->spinBox_VLeftbound->setValue( iLV );

    mpUi->spinBox_HRightbound->setValue( iRH );
    mpUi->spinBox_SRightbound->setValue( iRS );
    mpUi->spinBox_VRightbound->setValue( iRV );

    mMutex.unlock();
}

void TrackingDialog::on_pushButton_previewThreshold_clicked()
{
    QString data_file( mpProjInfo->ProjectDir() +
                      QDir::separator() +
                      mpProjInfo->DataDir() +
                      QDir::separator() +
                      mpUi->fileSelectionBox->currentText() +
                      QDir::separator() +
                      DEFAULT_DATA_IMAGES_XML );
    if (data_file.size() < 1 || !QFile( data_file ).exists())
    {
        QMessageBox msg_box;
        std::cerr << "ERROR FILE: " << data_file.toStdString() << std::endl;
        msg_box.setText( "Data image xml appears to be corrupted.");
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Error!");
        msg_box.exec();
        return;
    }

    FreezeDialog( true );
    ThresholdDialog thresholdDialog( mpProjInfo, data_file, this );
    thresholdDialog.exec();
    FreezeDialog( false );
}

void TrackingDialog::FreezeDialog( bool iFreezeDialog )
{
    mMutex.lock();
    mpUi->startTrackingButton->setEnabled( !iFreezeDialog );
    mpUi->pushButton_leftbound->setEnabled( !iFreezeDialog );
    mpUi->pushButton_rightbound->setEnabled( !iFreezeDialog );
    mpUi->pushButton_previewThreshold->setEnabled( !iFreezeDialog );

    mpUi->checkBox_mask->setEnabled( !iFreezeDialog );
    mpUi->checkBox_saveUndistortedImages->setEnabled( !iFreezeDialog );
    mpUi->checkBox_vizCameraPose->setEnabled( !iFreezeDialog );
    mpUi->checkBox_vizPointCorrespondences->setEnabled( !iFreezeDialog);
    mpUi->checkBox_vizThresholds->setEnabled( !iFreezeDialog );
    mpUi->groupBox_camsForTriangulation->setEnabled( !iFreezeDialog );
    mpUi->checkBox_useSavedMarkedPoints->setEnabled( !iFreezeDialog );
    mpUi->fileSelectionBox->setEnabled( !iFreezeDialog );
    mpUi->checkBox_vizUndistortedImages->setEnabled( !iFreezeDialog );

    mpUi->spinBox_numIterations->setEnabled( !iFreezeDialog );
    mpUi->spinBox_filterSize->setEnabled( !iFreezeDialog );
    mpUi->horizontalSlider_threshold->setEnabled( !iFreezeDialog );

    if (mpUi->checkBox_vizCameraPose->isChecked())
    {
        mpUi->spinBox_vizCameraPose->setEnabled( !iFreezeDialog );
    }

    if (mpUi->checkBox_vizPointCorrespondences->isChecked())
    {
        mpUi->spinBox_vizPointCorrespondences->setEnabled( !iFreezeDialog );
    }

    if (mpUi->checkBox_vizThresholds->isChecked())
    {
        mpUi->spinBox_vizThresholds->setEnabled( !iFreezeDialog );
    }

    if (mpUi->checkBox_vizUndistortedImages->isChecked())
    {
        mpUi->spinBox_vizUndistortedImages->setEnabled( !iFreezeDialog );
    }

    mpUi->spinBox_HLeftbound->setEnabled( !iFreezeDialog );
    mpUi->spinBox_HRightbound->setEnabled( !iFreezeDialog );
    mpUi->spinBox_minCamsForTriangulation->setEnabled( !iFreezeDialog );
    mpUi->spinBox_maxNumPoints->setEnabled( !iFreezeDialog );
    mpUi->spinBox_SLeftbound->setEnabled( !iFreezeDialog );
    mpUi->spinBox_SRightbound->setEnabled( !iFreezeDialog );
    mpUi->spinBox_VLeftbound->setEnabled( !iFreezeDialog );
    mpUi->spinBox_VRightbound->setEnabled( !iFreezeDialog );

    if (mpUi->checkBox_saveUndistortedImages->isChecked() )
    {
        mpUi->lineEdit_undistortedOutput->setEnabled( !iFreezeDialog );
    }

    mpUi->checkBox_undistortImages->setEnabled( !iFreezeDialog );
    mMutex.unlock();

    update();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);

}

void TrackingDialog::PullNewVerboseOutput()
{
    //QByteArray output = process->readAllStandardOutput();
    //QStringList strLines = QString(output).split("\n");


    //QTextStream out( outputFile );
//    foreach (QString line, strLines){
////        mpUi->textEdit->append( line );
//        std::cerr << "SimplePointTracking: " <<  line.toStdString() << std::endl;
//        out << line << '\n';
//    }
}

bool TrackingDialog::PopulateTrackerConfig( TrackerConfigFile& oConfig, uint32_t& oCaptureCount, std::string& oTriDir)
{
    QString base_dir = mpProjInfo->ProjectDir() + QDir::separator();
    QString calFile( base_dir + mpProjInfo->FinishedCalibXml() );
    if (calFile.size() < 1 || !QFile( calFile ).exists())
    {
        QMessageBox msg_box;
        msg_box.setText( "Calibration image might be missing or corrupted.");
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Error!");
        msg_box.exec();
        return false;
    }

    QString data_file( base_dir +
                      QDir::separator() +
                      mpProjInfo->DataDir() +
                      QDir::separator() +
                      mpUi->fileSelectionBox->currentText() +
                      QDir::separator() +
                      DEFAULT_DATA_IMAGES_XML );
    if (data_file.size() < 1 || !QFile( data_file ).exists())
    {
        std::cerr << "ERR FILE: " << data_file.toStdString() << std::endl;

        QMessageBox msg_box;
        msg_box.setText( "Image data xml might be corrupted or missing.");
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Error!");
        msg_box.exec();
        return false;
    }

    QString mask_file(  base_dir + mpProjInfo->ProjectMaskXml() );
    if (mpUi->checkBox_mask->isChecked())
    {
        if (mask_file.size() < 1 || !QFile( mask_file ).exists())
        {
            std::cerr << "ERR FILE: " << mask_file.toStdString() << std::endl;
            QMessageBox msg_box;
            msg_box.setText( "Mask image xml might be corrupted or missing.");
            msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
            msg_box.setWindowTitle("Error!");
            msg_box.exec();
            return false;
        }
    }

    QString undist_dir(  base_dir +
                        DEFAULT_UNDISTORTED_DATA_NAME_BASE +
                       QDir::separator() +
                       mpUi->fileSelectionBox->currentText() );
    if (mpUi->checkBox_saveUndistortedImages->isChecked())
    {
        QDir().mkdir( undist_dir );
    }

    QString tri_dir( base_dir +
                    mpProjInfo->TriangDir() +
                    QDir::separator() +
                    mpUi->fileSelectionBox->currentText());
    oTriDir = tri_dir.toStdString();
    QDir().mkdir( tri_dir );

    //Now we need to increment the filename in case there are multiple attempts
    QDirIterator it(tri_dir,
                    QDirIterator::Subdirectories);
    oCaptureCount = 0;
    while (it.hasNext())
    {
        QDir folder(it.next());
        QStringList name_parts_list = folder.dirName().split("_");
        if ( name_parts_list.size() >= 2)
        {
            //returns 0 on failure and should never be 0 if it is a proper folder.
            uint32_t tmp = name_parts_list[name_parts_list.size() -1].toInt();
            if ( tmp > oCaptureCount )
            {
                oCaptureCount = tmp;
            }
        }
    }
    ++oCaptureCount;

    QString tri_path = tri_dir +
                      QDir::separator() +
                      mpProjInfo->TriangNameBase() +
                      "_" +
                      QString::number( oCaptureCount ) +
                      ".xml" ;
    if (!QDir(  tri_dir ).exists())
    {
        std::cerr << "ERR DIR: " << (tri_dir).toStdString() << std::endl;

        QMessageBox msg_box;
        msg_box.setText( "There seems to be an error in the triangulated files directory.");
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Error!");
        msg_box.exec();
        return false;
    }

    QVector<int> cams_not_include;
    for (int i = 0; i < mpUi->numCamsInFile->value(); ++i)
    {
        QCheckBox* cam_box = mpUi->groupBox_camsForTriangulation->findChild<QCheckBox*>("checkBox_cam"+ QString::number(i));
        if (cam_box && !cam_box->isChecked())
        {
            cams_not_include.push_back(i);
        }
    }
    if (mpUi->numCamsInFile->value() - cams_not_include.size() < 2)
    {
        QMessageBox msg_box;
        msg_box.setText( "We need at least two cameras in order to do triangulation.");
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Error!");
        msg_box.exec();
        return false;
    }

    oConfig.projDir = mpProjInfo->ProjectDir().toStdString();
    oConfig.dataFileLocation =  QString( mpProjInfo->DataDir() +
                                                   QDir::separator() +
                                                   mpUi->fileSelectionBox->currentText() +
                                                   QDir::separator() +
                                                   DEFAULT_DATA_IMAGES_XML).toStdString();
    if (mpUi->checkBox_mask->isChecked())
    {
        oConfig.maskFileLocation = mpProjInfo->ProjectMaskXml().toStdString();
    }
    else
    {
        oConfig.maskFileLocation = "";
    }
    oConfig.calibFileLocation = mpProjInfo->FinishedCalibXml().toStdString();
    if (mpUi->checkBox_saveUndistortedImages->isChecked())
    {
        oConfig.writeUndistImages = undist_dir.toStdString();
    }
    else
    {
        oConfig.writeUndistImages = "";
    }
    oConfig.triangulationOutput = tri_path.toStdString();
    if (mpUi->checkBox_vizUndistortedImages->isChecked())
    {
        oConfig.vizUndistortedImages = mpUi->spinBox_vizUndistortedImages->value();
    }
    else
    {
        oConfig.vizUndistortedImages = -1;
    }

    if (mpUi->checkBox_vizPointCorrespondences->isChecked())
    {
        oConfig.vizPointCorrespondences = mpUi->spinBox_vizPointCorrespondences->value();
    }
    else
    {
        oConfig.vizPointCorrespondences = -1;
    }

    if (mpUi->checkBox_vizCameraPose->isChecked())
    {
        oConfig.vizCameraPose = mpUi->spinBox_vizCameraPose->value();
    }
    else
    {
        oConfig.vizCameraPose = -1;
    }

    if (mpUi->checkBox_vizThresholds->isChecked())
    {
        oConfig.vizThresholds = mpUi->spinBox_vizThresholds->value();
    }
    else
    {
        oConfig.vizThresholds = -1;
    }

    oConfig.camIndexesToExclude.clear();
    for (int32_t i = 0; i< cams_not_include.size(); ++i)
    {
       oConfig.camIndexesToExclude.insert( cams_not_include[i] );
    }
    oConfig.tryToUseSavedMarkedPoints = mpUi->checkBox_useSavedMarkedPoints->isChecked() ;
    oConfig.undistortImagesBool = mpUi->checkBox_undistortImages->isChecked();
    oConfig.viz3dTriangulatedPoints = false;
    oConfig.noiseFilterSize = mpUi->spinBox_filterSize->value();
    oConfig.noiseIterations = mpUi->spinBox_numIterations->value();
    oConfig.noiseThreshold = mpUi->horizontalSlider_threshold->value();
    oConfig.numCameras = mpUi->numCamsInFile->value();
    oConfig.minNumCamsTriangulation = mpUi->spinBox_minCamsForTriangulation->value();
    oConfig.maxNumPoints = mpUi->spinBox_maxNumPoints->value();
    oConfig.hLeftbound = mpUi->spinBox_HLeftbound->value();
    oConfig.sLeftbound = mpUi->spinBox_SLeftbound->value();
    oConfig.vLeftbound = mpUi->spinBox_VLeftbound->value();
    oConfig.hRightbound = mpUi->spinBox_HRightbound->value();
    oConfig.sRightbound = mpUi->spinBox_SRightbound->value();
    oConfig.vRightbound = mpUi->spinBox_VRightbound->value();

    return true;
}

void TrackingDialog::on_startTrackingButton_clicked()
{

    mLastCaptureCount = 0;
    mLastTriDir.clear();
    if (PopulateTrackerConfig( mLastTrackerConfig, mLastCaptureCount, mLastTriDir))
    {
        //Read in the dialog's arguments and write out a config file
        QString path =QString(mLastTriDir.c_str()) +
                      QDir::separator() +
                      QString( DEFAULT_SPT_CONFIG_NAME + QString::number( mLastCaptureCount ) + ".json") ;


        WriteConfig( path.toStdString() , mLastTrackerConfig );
        //Kick off the QProcess using the arg filename
        FreezeDialog( true );

        if (NULL != mpOutputFile)
        {
            mpOutputFile->close();
            delete mpOutputFile;
        }
        mpOutputFile = new QFile(path +
                               QString( "TriangulationOutput_" +
                               QString::number( mLastCaptureCount ) +
                               ".txt") );
        mMutex.lock();
        mRunning = true;
        mpOutputFile->open( QIODevice::WriteOnly );

        mMutex.unlock();

        if (NULL != mpTriTask)
        {
            mpTriTask->mWaitingOnUserHelp = false;
        }

        mpTriTask = new TriangulationTask( path, this );
        mpTriTask->setAutoDelete( true );
        QObject::connect( mpTriTask, SIGNAL(finished(int)), this, SLOT(on_ProcessFinished(int)) );
        qRegisterMetaType< std::vector<std::vector<SmtPixel> > >("std::vector<std::vector<SmtPixel> >");
        qRegisterMetaType< std::vector<SmtImage> >("std::vector<SmtImage>");
        qRegisterMetaType< std::pair<unsigned long, std::set<unsigned long> > >("std::pair<unsigned long, std::set<unsigned long> >");
        QObject::connect( mpTriTask, SIGNAL( GetUserHelpForCorrespondence(TriangulationTask*,
                                                                     const std::vector< SmtImage >,
                                                                     const int,
                                                                     const std::vector< std::vector< SmtPixel > >,
                                                                     const std::pair<unsigned long, std::set<unsigned long> >,
                                                                     const int)),
                          this, SLOT( UserCorrespondenceHelp(TriangulationTask*,
                                                             const std::vector< SmtImage >,
                                                             const int,
                                                             const std::vector< std::vector< SmtPixel > >,
                                                             const std::pair<unsigned long, std::set<unsigned long> >,
                                                             const int)));
        QThreadPool::globalInstance()->start( mpTriTask );

//        while (!QThreadPool::globalInstance()->waitForDone(500))
//        {
//            QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
//        }
//        delete task;
    }
}

void TrackingDialog::UserCorrespondenceHelp( TriangulationTask* iTask,
                                            std::vector< SmtImage > iTimestepImages,
                                            const int iTimestepValue,
                                            std::vector< std::vector< SmtPixel > > iTimestepMarkersFound,
                                            std::pair<unsigned long, std::set<unsigned long> > iCamsToExclude,
                                            const int iNumMarkersToFind)
{
    std::cerr << "Bringing up dialog for user help with correspondence." << std::endl;
    if (NULL != mpCorresDialog)
    {
        delete mpCorresDialog;
    }
    TriangulationPipeline* pipeline = TriangulationPipeline::Instance();
    mpCorresDialog = new CorrespondenceDialog(this,
                                           pipeline->Images(),
                                           iTimestepValue,
                                           pipeline->MarkersFound(),
                                           pipeline->CamsToExclude(),
                                           iNumMarkersToFind);

    connect( mpCorresDialog,
             SIGNAL(Cancelled()),
             mpTriTask->mpEventLoop,
             SLOT(quit()));
    connect( mpCorresDialog,
             SIGNAL(Cancelled()),
             this,
             SLOT(on_cancelTrackingButton_clicked()));
    connect( mpCorresDialog,
             SIGNAL(Completed()),
             mpTriTask,
             SLOT(ContinueRunning()));
    connect( mpCorresDialog,
             SIGNAL(ProcessingDone()),
             this,
             SLOT(WriteMarkedPoints()), Qt::QueuedConnection);
    std::cerr << "Showing dialog" << std::endl;

//    QEventLoop loop;
//    connect( &corres_dialog, &CorrespondenceDialog::Cancelled, &loop, &QEventLoop::quit );
//    connect( &corres_dialog, &CorrespondenceDialog::Completed, &loop, &QEventLoop::quit );
    mpCorresDialog->show();
    //QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
    std::cerr << "Waiting for user to finish." << std::endl;
    //loop.exec();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
}

void TrackingDialog::WriteMarkedPoints()
{
    assert(mpCorresDialog != NULL);
    if (mpCorresDialog->Complete())
    {
        std::cerr << "Successfully got correspondence information." << std::endl;
        QString path =QString( mLastTriDir.c_str()) +
                      QDir::separator() +
                      QString( DEFAULT_SPT_CONFIG_NAME + QString::number( mLastCaptureCount ) + ".json") ;
        WriteMarkedPointsToFile( path.toStdString(),
                                 mpCorresDialog->MarkedPoints(),
                                 mpCorresDialog->BestFitInfoForPoints() );
    }
    else
    {
        std::cerr << "Error getting user help to mark initial starting point configuration." << std::endl;
    }
    emit mpCorresDialog->Completed();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
}

void TrackingDialog::on_ProcessFinished(int iErrCode)
{
    PullNewVerboseOutput();
    mMutex.lock();
    mRunning = false;
    mpOutputFile->close();
    mpTriTask = NULL;
    std::cerr << "\nTriangulation Finished with exit code " << iErrCode << ".\n\n";

    mMutex.unlock();
    FreezeDialog( false );
}

void TrackingDialog::on_cancelTrackingButton_clicked()
{
    if (mRunning)
    {
        PullNewVerboseOutput();
        //Terminate Process
        mMutex.lock();
//        QString pid = QString::number( process->processId() );

//    #ifdef __linux__
//        if( mpUi->checkBox_vizCameraPose->isChecked() ||
//                mpUi->checkBox_vizPointCorrespondences->isChecked() ||
//                mpUi->checkBox_vizThresholds->isChecked() ||
//                mpUi->checkBox_vizTriangulatedPoints->isChecked() ||
//                mpUi->checkBox_vizUndistortedImages->isChecked())
//        {
//            //while( process->state() == QProcess::Running){
//            //    process->waitForFinished( 200 );
//                //system("kill " +  pid);
//                system("pkill SimplePoint");
                QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
//            //}
//        }
//    #endif
//        // windows & linux: kill main process
//        if( process->state() == QProcess::Running ){
//            process->kill();
//        }

        mpTriTask->Cancel();
        //mpTriTask->mWaitingOnUserHelp = false;
        mpTriTask = NULL;
        std::fprintf(stderr, "\n\n\nCall to Cancel Pipeline sent.....\n\n\n");


        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        mRunning = false;
        PullNewVerboseOutput();
        mpOutputFile->close();
        mMutex.unlock();
        FreezeDialog( false );
    }
    else{
        //PromptSaveChanges();
        close();
    }
}


void TrackingDialog::on_pushButton_leftbound_clicked()
{
    QColorDialog colorDiag;
    QColor color;
    color.setHsv( mpUi->spinBox_HLeftbound->value(),
                  mpUi->spinBox_SLeftbound->value(),
                  mpUi->spinBox_VLeftbound->value() );
    colorDiag.setCurrentColor( color );
    colorDiag.exec();

    color = colorDiag.currentColor();

    int h, s, v;
    color.getHsv( &h, &s, &v );
    QString back_str("background-color: #" +
                     QString(color.red() < 16? "0" : "") +
                     QString::number(color.red(),16) +
                     QString(color.green() < 16? "0" : "") +
                     QString::number(color.green(),16) +
                     QString(color.blue() < 16? "0" : "") +
                     QString::number(color.blue(),16) + ";");
    std::cerr << "From ColorDialog: " << back_str.toStdString() << std::endl;
    mMutex.lock();
    mpUi->spinBox_HLeftbound->setValue( h );
    mpUi->spinBox_SLeftbound->setValue( s );
    mpUi->spinBox_VLeftbound->setValue( v );
    mMutex.unlock();
}

void TrackingDialog::on_pushButton_rightbound_clicked()
{
    QColorDialog color_diag;
    QColor color;
    color.setHsv( mpUi->spinBox_HRightbound->value(),
                  mpUi->spinBox_SRightbound->value(),
                  mpUi->spinBox_VRightbound->value() );
    color_diag.setCurrentColor( color );
    color_diag.exec();

    color = color_diag.currentColor();

    int h, s, v;
    color.getHsv( &h, &s, &v );
    QString backstr("background-color: #" +
                    QString(color.red() < 16? "0" : "") +
                    QString::number(color.red(),16) +
                    QString(color.green() < 16? "0" : "") +
                    QString::number(color.green(),16) +
                    QString(color.blue() < 16? "0" : "") +
                    QString::number(color.blue(),16) + ";");
    std::cerr << "From ColorDialog: " << backstr.toStdString() << std::endl;

    mMutex.lock();
    mpUi->spinBox_HRightbound->setValue( h );
    mpUi->spinBox_SRightbound->setValue( s );
    mpUi->spinBox_VRightbound->setValue( v );
    mMutex.unlock();
}

void TrackingDialog::UpdateLeftboundButtonColor()
{
    QColor color;
    color.setHsv( mpUi->spinBox_HLeftbound->value(),
                  mpUi->spinBox_SLeftbound->value(),
                  mpUi->spinBox_VLeftbound->value() );
    QString s("background-color: #" +
              QString(color.red() < 16? "0" : "") +
              QString::number(color.red(),16) +
              QString(color.green() < 16? "0" : "") +
              QString::number(color.green(),16) +
              QString(color.blue() < 16? "0" : "") +
              QString::number(color.blue(),16) + ";");



    std::cerr << "From Spinbox: " << s.toStdString() << std::endl;
    mMutex.lock();
    mpUi->pushButton_leftbound->setStyleSheet(s);
    mpUi->pushButton_leftbound->update();
    mMutex.unlock();
}

void TrackingDialog::UpdateRightboundButtonColor()
{
    QColor color;
    color.setHsv( mpUi->spinBox_HRightbound->value(),
                  mpUi->spinBox_SRightbound->value(),
                  mpUi->spinBox_VRightbound->value() );
    QString s("background-color: #" +
              QString(color.red() < 16? "0" : "") +
              QString::number(color.red(),16) +
              QString(color.green() < 16? "0" : "") +
              QString::number(color.green(),16) +
              QString(color.blue() < 16? "0" : "") +
              QString::number(color.blue(),16) + ";");

    std::cerr << "From Spinbox: " << s.toStdString() << std::endl;
    mMutex.lock();
    mpUi->pushButton_rightbound->setStyleSheet(s);
    mpUi->pushButton_rightbound->update();
    mMutex.unlock();
}

void TrackingDialog::on_spinBox_HLeftbound_editingFinished()
{
    //Color doesn't render correctly so it might be confusing to change the button color
    //mutex.lock();
    //mutex.unlock();
    //updateLeftboundButtonColor();
}

void TrackingDialog::on_spinBox_HRightbound_editingFinished()
{
    //Color doesn't render correctly so it might be confusing to change the button color
    //mutex.lock();
    //mutex.unlock();
    //updateRightboundButtonColor();
}

void TrackingDialog::on_spinBox_SLeftbound_editingFinished()
{
    //Color doesn't render correctly so it might be confusing to change the button color
    //mutex.lock();
    //mutex.unlock();
    //updateLeftboundButtonColor();
}

void TrackingDialog::on_spinBox_SRightbound_editingFinished()
{
    //Color doesn't render correctly so it might be confusing to change the button color
    //mutex.lock();
    //mutex.unlock();
    //updateRightboundButtonColor();
}

void TrackingDialog::on_spinBox_VLeftbound_editingFinished()
{
    //Color doesn't render correctly so it might be confusing to change the button color
    //mutex.lock();
    //mutex.unlock();
    //updateLeftboundButtonColor();
}

void TrackingDialog::on_spinBox_VRightbound_editingFinished()
{
    //Color doesn't render correctly so it might be confusing to change the button color
    //mutex.lock();
    //mutex.unlock();
    //updateRightboundButtonColor();
}

void TrackingDialog::on_checkBox_undistortImages_toggled()
{
    mMutex.lock();
    if (mpUi->checkBox_undistortImages->isChecked())
    {
        mpUi->checkBox_saveUndistortedImages->setEnabled( true );
        mpProjInfo->SaveUndistort() = true;
    }
    else
    {
        mpProjInfo->SaveUndistort() = false;
        mpUi->checkBox_saveUndistortedImages->setEnabled( false );
        mpUi->lineEdit_undistortedOutput->setEnabled( false );
        mpUi->lineEdit_undistortedOutput->clear();
    }
    mMutex.unlock();
    QCoreApplication::processEvents();
}

void TrackingDialog::on_checkBox_saveUndistortedImages_toggled()
{
    QMutexLocker locker( &mMutex );
    if (mpUi->checkBox_saveUndistortedImages->isChecked())
    {
        mpUi->lineEdit_undistortedOutput->setEnabled( true );
        if (mpUi->fileSelectionBox->currentText() != "")
        {
            mpUi->lineEdit_undistortedOutput->setText( mpProjInfo->ProjectDir() +
                                                     QDir::separator() +
                                                     DEFAULT_UNDISTORTED_DATA_NAME_BASE +
                                                     QDir::separator() +
                                                     mpUi->fileSelectionBox->currentText());
        }
        else
        {
            mpUi->lineEdit_undistortedOutput->clear();
        }
    }
    else
    {
        mpUi->lineEdit_undistortedOutput->setEnabled( false );
        mpUi->lineEdit_undistortedOutput->clear();
    }
    mpUi->lineEdit_undistortedOutput->repaint();
}

void TrackingDialog::on_checkBox_vizUndistortedImages_toggled()
{
    QMutexLocker locker( &mMutex );
    if (mpUi->checkBox_vizUndistortedImages->isChecked())
    {
        mpUi->spinBox_vizUndistortedImages->setEnabled( true );
    }
    else
    {
        mpUi->spinBox_vizUndistortedImages->setEnabled( false );
    }
    mpUi->spinBox_vizUndistortedImages->repaint();
}

void TrackingDialog::on_checkBox_vizThresholds_toggled()
{
    QMutexLocker locker( &mMutex );
    if (mpUi->checkBox_vizThresholds->isChecked())
    {
        mpUi->spinBox_vizThresholds->setEnabled( true );
    }
    else
    {
        mpUi->spinBox_vizThresholds->setEnabled( false );
    }
    mpUi->spinBox_vizThresholds->repaint();
}

void TrackingDialog::on_checkBox_vizPointCorrespondences_toggled()
{
    QMutexLocker locker( &mMutex );
    if (mpUi->checkBox_vizPointCorrespondences->isChecked())
    {
        mpUi->spinBox_vizPointCorrespondences->setEnabled( true );
    }
    else
    {
        mpUi->spinBox_vizPointCorrespondences->setEnabled( false );
    }
    mpUi->spinBox_vizPointCorrespondences->repaint();
}

void TrackingDialog::on_checkBox_vizCameraPose_toggled()
{
    QMutexLocker locker( &mMutex );
    if (mpUi->checkBox_vizCameraPose->isChecked())
    {
        mpUi->spinBox_vizCameraPose->setEnabled( true );
    }
    else
    {
         mpUi->spinBox_vizCameraPose->setEnabled( false );
    }
    mpUi->spinBox_vizCameraPose->repaint();
}

void TrackingDialog::on_checkBox_mask_toggled()
{
    if (mpUi->checkBox_mask->isChecked())
    {
        mpProjInfo->UseMask() = true;
    }
    else
    {
        mpProjInfo->UseMask() = false;
    }
}

bool TrackingDialog::ProcessXMLFile( QString iXmlFile )
{
    bool found_cam_num = false;
    std::string tag = "<nCameras>";
    QFile input_file(iXmlFile);
    if (input_file.open(QIODevice::ReadOnly))
    {
       QTextStream in(&input_file);
       while (!in.atEnd())
       {
          QString line = in.readLine();

          if (line.toStdString().find( tag ) != std::string::npos)
          {
              mMutex.lock();
              mNumCams = (line.remove( "nCameras" ).remove( '<' ).remove( '>' ). remove( "\\" ).remove( '/' )).toInt();
              mMutex.unlock();
              found_cam_num = true;
              break;
          }
       }
       input_file.close();
    }

    mMutex.lock();
    mpUi->numCamsInFile->setValue( mNumCams );
    mMutex.unlock();
    return found_cam_num;
}

uint32_t TrackingDialog::MaxNumPoints()
{
    uint32_t value = mpUi->spinBox_maxNumPoints->value();
    return value;
}

void TrackingDialog::MaxNumPoints(const uint32_t iNewVal)
{
    mpUi->spinBox_maxNumPoints->setValue(iNewVal);
}

uint32_t TrackingDialog::FilterSize()
{
    return mpUi->spinBox_filterSize->value();
}

uint32_t TrackingDialog::NumIterations()
{
    return mpUi->spinBox_numIterations->value();
}

uint32_t TrackingDialog::LowerNoiseThreshold()
{
    return mpUi->horizontalSlider_threshold->value();
}

cv::Scalar TrackingDialog::Leftbound()
{
    return cv::Scalar( mpUi->spinBox_HLeftbound->value(),
                       mpUi->spinBox_SLeftbound->value(),
                       mpUi->spinBox_VLeftbound->value());
}

cv::Scalar TrackingDialog::Rightbound()
{
    return cv::Scalar( mpUi->spinBox_HRightbound->value(),
                       mpUi->spinBox_SRightbound->value(),
                       mpUi->spinBox_VRightbound->value());
}

void TrackingDialog::SetFilterSize( uint32_t iSize )
{
    mpUi->spinBox_filterSize->setValue( iSize );
}

void TrackingDialog::SetNumIterations( uint32_t iNum )
{
    mpUi->spinBox_numIterations->setValue( iNum );
}

void TrackingDialog::SetLowerNoiseThreshold( uint32_t iThresh )
{
    mpUi->horizontalSlider_threshold->setValue( iThresh );
}
