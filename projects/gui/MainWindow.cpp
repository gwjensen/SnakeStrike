#include <QSettings>
#include <QDesktopWidget>
#include <QFileDialog>
#include <pylon/PylonIncludes.h>
#include <QThread>
#include <QMessageBox>
#include <QPixmap>
#include <QStyleFactory>
#include <QDebug>
#include <QScrollBar>
#include <opencv2/core.hpp>

#include "GuiCamera.h"
#include "GuiCamArray.h"

#include "ui_MainWindow.h"
#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      mpUi(new Ui::MainWindow),
      mpCamCalDialog(NULL),
      mpProjDialog(NULL),
      mpRecordDialog(NULL),
      mpTrackDialog(NULL),
      mpTreeModel(NULL),
      mpVis3dDialog(NULL),
      mpExportDialog(NULL),
      mpHelpDialog(NULL),
      mpCamList(NULL),
      mPlay(false),
      mCamHerz(0)
{
    mpUi->setupUi(this);
    Q_INIT_RESOURCE(icons);
    QPixmap pm("://snake_icon_inv.jpg");
    setWindowIcon(QIcon(pm));

    //Testing css skins
    qApp->setStyle(QStyleFactory::create("Fusion"));

    mpFileDialog = new QFileDialog( this );
    mpFileDialog->setFileMode( QFileDialog::ExistingFile );

    this->setWindowTitle("SnakeStrike - Simple 3D Tracking");
    resize(QDesktopWidget().availableGeometry(this).size() * 0.85); // Scale window to 85% screen size

    mpUi->camFrame->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    mpUi->camFrame->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    QSettings settings("config.ini", QSettings::IniFormat); // Read desired results folder from config file

    QString path = settings.value("mainwindow/path", QDir::homePath().append("/PWVi_Results")).toString(); // Use home/PWVi_Results if unset
    if (!QDir(path).exists())
    {
        QDir().mkpath(path);
    }
    mDir.cd(path);

    RefreshCameras();

    mpUi->editProjectSettingsButton->setEnabled( false );
    UpdateHzInfo();
}

MainWindow::~MainWindow()
{
    delete mpCamCalDialog;
    delete mpTrackDialog;
    mpCamList->Close();
    delete mpCamList;
    delete mpTreeModel;
    delete mpProjDialog;
    mpUi->camFrame->closeAllSubWindows();
    foreach (CamPreviewWindow* win, mCamWindowList)
    {
        if (win != NULL)
        {
            delete win;
        }
    }
    mCamWindowList.clear();

    delete mpUi;
}

void MainWindow::InitDependentDialogs()
{
    if (NULL != mpProjDialog)
    {
        mpProjDialog->disconnect( SIGNAL( SignalRetrieveImageTimeoutChanged() ) );
        delete mpProjDialog;
    }
    mpProjDialog = new ProjectDialog( this );
    connect( mpProjDialog,
             SIGNAL( SignalRetrieveImageTimeoutChanged() ),
             this,
             SLOT( UpdateProjectSettings() ));

    if (NULL != mpCamCalDialog )
    {
        delete mpCamCalDialog;
    }

    mpCamCalDialog = new CamCalibrateDialog( const_cast<const ProjectDialog*&>(mpProjDialog), this );
    mpCamCalDialog->setModal( true );
    mpCamCalDialog->setWindowTitle( "Calibrate Camera");

    if (NULL != mpExportDialog)
    {
        delete mpExportDialog;
    }
    mpExportDialog = new ExportDialog( mpProjDialog, this );
    mpExportDialog->setModal( true );
    mpExportDialog->setWindowTitle(" Export ");


    if ( NULL != mpVis3dDialog )
    {
        delete mpVis3dDialog;
    }

    mpVis3dDialog = new VisualizationDialog( mpProjDialog, this );
    mpVis3dDialog->setModal( true );
    mpVis3dDialog->setWindowTitle( "Viewing 3d Points");


}

void MainWindow::closeEvent(QCloseEvent *event)
{
    mpUi->camFrame->closeAllSubWindows();
    if (mpUi->camFrame->currentSubWindow())
    {
        event->ignore();
    }
    event->accept();
}

CamPreviewWindow* MainWindow::CreatePreviewWindow(int iCamIdx)
{
    CamPreviewWindow* child = new CamPreviewWindow( mpUi->camFrame, iCamIdx );
    mpUi->camFrame->addSubWindow(child);

    return child;
}

CamPreviewWindow* MainWindow::ActivePreviewWindow() const
{
    if (QMdiSubWindow* activeSubWindow = mpUi->camFrame->activeSubWindow())
    {
        return qobject_cast<CamPreviewWindow *>(activeSubWindow->widget());
    }
    return 0;
}

CamPreviewWindow* MainWindow::FindPreviewWindow(const int iCamIdx) const
{
    return mCamWindowList[iCamIdx];
}

void MainWindow::RefreshCameras()
{
    if (NULL != mpCamList)
    {
        for (int32_t i=0; i< mpCamList->Size(); ++i )
        {
            GuiCamera* camera;
            bool ret = mpCamList->GetCamera( i, camera );
            if (!ret)
            {
                std::cerr << "Issue with getting camera from Camera list in MainWindow" << std::endl;
            }
            else
            {
                camera->disconnect( SIGNAL(SignalRefreshCameras()) );
            }
        }
        mpCamList->Close();
        delete mpCamList;
    }

    if (NULL != mpProjDialog)
    {
        mpCamList = new GuiCamArray(this, mpProjDialog->CamLibPath());
    }
    else
    {
        mpCamList = new GuiCamArray(this, ProjectDialog::DefaultCamLibPath());
    }
    QStringList camera_names;
    mpCamList->PopulateCams( camera_names );

    RefreshPreviewWindows();
    UpdateHzInfo();
    RefreshCameraTree( camera_names );

}

void MainWindow::RefreshPreviewWindows()
{
    if (mCamWindowList.size() > 0)
    {
        foreach (CamPreviewWindow* p_cam_window, mCamWindowList)
        {
            delete p_cam_window;
        }
        mCamWindowList.clear();
        mCamPreviewList.clear();
    }

    for (int i =0; i < mpCamList->Size(); ++i)
    {
        GuiCamera* camera;
        bool ret = mpCamList->GetCamera( i, camera );
        if (!ret)
        {
            std::cerr << "Issue with getting camera from Camera list in MainWindow" << std::endl;
        }

        CamPreviewWindow* prev_window = new CamPreviewWindow( mpUi->camFrame, i );
        mCamPreviewList.push_back( prev_window->GetLabel() );
        prev_window->UpdateImageSize( camera->Width(),camera->Height() );
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);

        mCamWindowList.push_back( prev_window );

        mCamWindowList[i]->show();
    }
    if ( mpUi->camFrame->verticalScrollBar()->isVisible() ||
         mpUi->camFrame->horizontalScrollBar()->isVisible() )
    {
        mpUi->camFrame->cascadeSubWindows();
    }
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);

}

void MainWindow::RefreshCameraTree(const QStringList& iCameraNames)
{
    mpTreeModel = new CameraInfoTree( iCameraNames );

    mpUi->treeView->setModel( mpTreeModel );
    mpUi->treeView->expandAll();
    mpUi->treeView->show();
}


void MainWindow::on_previewButton_clicked()
{
    if ( NULL == mpRecordDialog )
    {
        //We create a record dialog here because if preview button is clicked before any record
        //dialogs are created we can't keep track of whether any cameras are running at the moment
        //or not. We just need the object, thats why we don't show the dialog.
        mpRecordDialog = CreateRecordDialog( DATA, false );
    }

    if (mpUi->previewButton->text() == "Cam Preview")
    {
        for (int i = 0; i < mpCamList->Size(); ++i)
        {
            (*mpCamList)[i].MakeConnect();
            (*mpCamList)[i].Open();

            mpUi->previewButton->setText("Preview");
            mpUi->previewButton->repaint();
        }
    }
    else
    {
        if (!mPlay)
        {
            for (int i = 0; i < mpCamList->Size(); ++i)
            {
                if (!(*mpCamList)[i].IsOpen())
                {
                    (*mpCamList)[i].Open();
                }
            }
            mpUi->previewButton->setText("Pause");
            mpUi->previewButton_2->setText("Pause");
            mpUi->previewButton->repaint();
            mpUi->previewButton_2->setText("Pause");
            mpCamList->StartGrabbingPreview();
            mPlay = true;

        }
        else
        {
            mpCamList->StopGrabbing();
            mpUi->previewButton->setText("Preview");
            mpUi->previewButton_2->setText("Preview");
            mpUi->previewButton->repaint();
            mpUi->previewButton_2->repaint();
            mPlay = false;
        }
    }
    while (!mpCamList->Stopped())
    {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        QThread::msleep(100);
    }
}

void MainWindow::UpdateCamPreviewWindow( int iLabelIdx, QPixmap iImage )
{
    CamPreviewCanvas* label;
    if (this->GetLabel( iLabelIdx, label ))
    {
        label->setPixmap(iImage);
        label->update();
    }
    else
    {
        std::cerr << "Error updating Viewfinder because label idx was not valid. Idx="
                  << iLabelIdx
                  <<  std::endl;
    }
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
}

bool MainWindow::GetLabel( const int iLabelIdx, CamPreviewCanvas*& oLabelPtr )
{
    if (iLabelIdx < mCamPreviewList.size())
    {
        oLabelPtr = mCamPreviewList[iLabelIdx];
        if (oLabelPtr != 0x0)
        {
            return true;
        }
    }
    return false;
}

void MainWindow::HideOptionsUnopenedProject( bool iHide )
{
    mpUi->editProjectSettingsButton->setEnabled( !iHide );

    mpUi->trackingButton->setEnabled( !iHide );
    mpUi->pushButton_export->setEnabled( !iHide );
    mpUi->visualisationButton->setEnabled( !iHide );
    if (mpCamList->Size() > 0)
    {
        mpUi->previewButton->setEnabled( !iHide );
        mpUi->previewButton_2->setEnabled( !iHide );
        mpUi->camConfigButton->setEnabled( !iHide );
        mpUi->camConfigButton_2->setEnabled( !iHide );
        mpUi->calibrateButton->setEnabled( !iHide );
        mpUi->getMaskButton->setEnabled( !iHide );
        if (NULL != mpCamCalDialog && mpCamCalDialog->FinishedCalibrationExists())
        {
            mpUi->recordButton->setEnabled( !iHide );
        }
        else
        {
            mpUi->recordButton->setEnabled( false );
        }
    }
    else
    {
        mpUi->previewButton->setEnabled( false );
        mpUi->previewButton_2->setEnabled( false );
        mpUi->camConfigButton->setEnabled( false );
        mpUi->camConfigButton_2->setEnabled( false );
        mpUi->calibrateButton->setEnabled( false );
        mpUi->getMaskButton->setEnabled( false );
        mpUi->recordButton->setEnabled( false );
    }
}

void MainWindow::on_openProjectButton_clicked()
{
    QString dir_str = QFileDialog::getExistingDirectory(this, tr("Load Exisiting Project"),
                                                     "",
                                                     QFileDialog::ShowDirsOnly
                                                     | QFileDialog::DontResolveSymlinks);
    if ( !dir_str.isEmpty() )
    {
        dir_str = QDir::toNativeSeparators(dir_str);
        QFileInfo config_file( dir_str + QDir::separator() + DEFAULT_PROJECT_FILE);
        if (config_file.exists())
        {
            InitDependentDialogs();
            mpProjDialog->ReadConfigFile( config_file.absoluteFilePath() );
            mpCamCalDialog->CalibrationImagesXMLExists();

            mpUi->projectName_label->setText( mpProjDialog->Name() );
            HideOptionsUnopenedProject( false );
        }
    }
}

void MainWindow::on_createProjectButton_clicked()
{
    QString dir_str = QFileDialog::getExistingDirectory(this, tr("New Project Directory"),
                                                 "",
                                                 QFileDialog::ShowDirsOnly
                                                 | QFileDialog::DontResolveSymlinks);
    if (!dir_str.isEmpty())
    {
        dir_str = QDir::toNativeSeparators(dir_str);
        InitDependentDialogs();
        mpProjDialog->SetupNewProject( dir_str );
        if (mpProjDialog->ProjectOpened())
        {
            mpCamCalDialog->CalibrationImagesXMLExists();
            mpUi->projectName_label->setText( mpProjDialog->Name() );
            HideOptionsUnopenedProject( false );
        }
    }
}

void MainWindow::on_editProjectSettingsButton_clicked()
{
    assert( mpProjDialog != NULL );
    mpProjDialog->show();
}


void MainWindow::on_trackingButton_clicked()
{
    assert( mpProjDialog != NULL ); //shouldn't be able to do tracking w/o a project open.
//    if (NULL != mpTrackDialog)
//    {
//        delete mpTrackDialog;
//    }
    if (NULL != mpTrackDialog)
    {
        delete mpTrackDialog;
    }
    mpTrackDialog = new TrackingDialog( mpProjDialog, this);
    mpTrackDialog->setModal( true );
    mpTrackDialog->setWindowTitle( " Triangulation of 3d Points");
    mpTrackDialog->show();
}

void MainWindow::on_calibrateButton_clicked()
{
    assert( NULL != mpCamCalDialog );
    mpCamCalDialog->show();
}

void MainWindow::on_visualisationButton_clicked()
{
    assert( NULL != mpVis3dDialog );
    assert( mpProjDialog != NULL ); //shouldn't be able to do visualisation w/o a project open.
    if (mpVis3dDialog->PopulateTrackingFiles())
    {
       mpVis3dDialog->show();
    }
}

void MainWindow::on_pushButton_export_clicked()
{
    assert( NULL != mpExportDialog );
    mpExportDialog->PopulateTrackingFiles();
    mpExportDialog->show();
}

void MainWindow::UpdateHzInfo()
{
    //update Hz info in record dialog
    double hz = 0;
    bool hz_set = false;
    bool all_hz_match = true;
    for (int i=0; i < mpCamList->Size(); ++i)
    {
        if (!hz_set)
        {
            hz = (*mpCamList)[i].Hz();
            hz_set = true;
            continue;
        }
        if ((*mpCamList)[i].Hz() != hz)
        {
            all_hz_match = false;
        }

    }
    if (all_hz_match)
    {
        mCamHerz =  hz;
    }
    else
    {
        QMessageBox msg_box;
        QString output = "";
        for (int i=0; i < mpCamList->Size(); ++i)
        {
            GuiCamera& camera = (*mpCamList)[i];
            output += "Camera "
                      + QString::number( camera.CamIdx() )
                      + " is at "
                      + QString::number( camera.Hz() )
                      + " Hz.\n";
        }
        output += "\nLoad a camera config file to bring all cameras to the same configuration "
                  "before proceeding with capturing images.";
        msg_box.setText( output );
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Frame Rates of cameras don't match!!");
        msg_box.exec();
    }
}

void MainWindow::on_camConfigButton_clicked()
{
    mpFileDialog->exec();

    if (mpFileDialog->selectedFiles().size() > 0)
    {
        QString genicam_config_filename = mpFileDialog->selectedFiles()[0] ;
        std::cerr << "The selected file is '"
                  << genicam_config_filename.toUtf8().constData()
                  << "'"
                  << std::endl;
        mpCamList->StopGrabbing();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);

        ///:TODO: :KLUDGE: this is a hack - should use the same method as in the record.cpp to keep track of starting and stoppping cameras
        QThread::msleep( 1000 );
        for (int i=0; i < mpCamList->Size(); ++i)
        {
            GuiCamera& camera = (*mpCamList)[i];
            camera.UpdateConfig( genicam_config_filename.toUtf8().constData() );
            if (camera.IsHardwareTriggered())
            {
                camera.SetupHardwareTrigger();
            }
            else
            {
                camera.SetupSoftwareTrigger();
            }
        }
        QFileInfo genicam_file( genicam_config_filename );
        mpTreeModel->UpdateConfigFileInfo( genicam_file.fileName() );
        mpUi->treeView->setModel( NULL );
        mpUi->treeView->setModel( mpTreeModel );
        mpUi->treeView->show();

        RefreshPreviewWindows();

        //update Hz info in record dialog
        UpdateHzInfo();
        mpUi->treeView->expandAll();
    }
}

void MainWindow::UpdateProjectSettings()
{
    mpCamList->SetRetrievalTimeout( mpProjDialog->ImageRetrieveTimeout() );
}

void MainWindow::UpdateState()
{
    if( NULL != mpCamCalDialog)
    {
        HideOptionsUnopenedProject( false );
        if ( mpCamCalDialog->FinishedCalibrationExists() )
        {
            mpUi->recordButton->setEnabled( true );
        }
    }
}

void MainWindow::on_recordButton_clicked(){

    if (mpRecordDialog != NULL)
    {
        delete mpRecordDialog;
    }
    mpRecordDialog = CreateRecordDialog( DATA, false );
    mpRecordDialog->show();
}


void MainWindow::on_getMaskButton_clicked()
{
    if (mpRecordDialog != NULL)
    {
        delete mpRecordDialog;
    }
    mpRecordDialog = CreateRecordDialog( MASK, false );
    mpRecordDialog->show();

}

RecordDialog* MainWindow::CreateRecordDialog( const RecordType& iRecordType, bool iXmlAlreadyExists )
{
    RecordDialog* tmp = new RecordDialog( mpCamList,
                                          const_cast<const ProjectDialog*&>(mpProjDialog),
                                          iRecordType,
                                          mCamHerz,
                                          iXmlAlreadyExists,
                                          this );
    tmp->setModal( true );

    return  tmp;
}


void MainWindow::on_camConfigButton_2_clicked()
{
    on_camConfigButton_clicked();
}

void MainWindow::on_refreshCamerasButton_clicked()
{
    RefreshCameras();
}

void MainWindow::on_previewButton_2_clicked()
{
    on_previewButton_clicked();
}
