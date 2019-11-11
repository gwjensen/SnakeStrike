#include <QFileInfo>
#include <QDir>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <QCloseEvent>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <sys/stat.h>
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include "rapidjson/error/en.h"

#include "ui_ProjectDialog.h"
#include "ProjectDialog.h"

ProjectDialog::ProjectDialog(QWidget *ipParent)
    : QDialog(ipParent),
      mpUi(new Ui::ProjectDialog),
      mCalibrationImageOrigLoc( DEFAULT_CALIB_IMAGE_DIR + QDir::separator() + DEFAULT_CALIB_IMAGE ),
      mCalibrationImageProjLoc( DEFAULT_CALIB_IMAGE_DIR + QDir::separator() + DEFAULT_CALIB_IMAGE ),
      mTriggeringScriptOrigLoc( DEFAULT_SCRIPT_DIR + QDir::separator() + DEFAULT_TRIGGER_SCRIPT ),
      mTriggeringScriptProjLoc( DEFAULT_SCRIPT_DIR + QDir::separator() + DEFAULT_TRIGGER_SCRIPT ),
      mExportConvertScriptLoc( DEFAULT_SCRIPT_DIR + QDir::separator() + DEFAULT_EXPORT_CONVERT_SCRIPT ),
      mSaveDataDir( DEFAULT_DATA_SAVE_DIR ),
      mSaveDataFilenameBase( DEFAULT_DATA_SAVE_NAME_BASE ),
      mSaveCalibDir( DEFAULT_CALIB_IMAGE_DIR ),
      mSaveMaskDir( DEFAULT_MASK_SAVE_DIR ),
      mCamLibPath( DEFAULT_BASLER_CAMERA_LIBRARY ),
      mRetrieveImageTimeout( DEFAULT_IMAGE_RETRIEVE_TIMEOUT ),
      mProjDir(""),
      mName("None Selected"),
      mSaveTriangDir( DEFAULT_TRIANG_SAVE_DIR ),
      mSaveTriangFilenameBase( DEFAULT_TRIANG_FILE_BASE ),
      mHLeftBound(0),
      mSLeftBound(0),
      mVLeftBound(0),
      mHRightBound(0),
      mSRightBound(0),
      mVRightBound(0),
      mFilterSize(3),
      mFilterIterations(1),
      mFilterThreshold(120),
      mNumPointsToTrack(0),
      mThresholdParmsChanged(false),
      mRetrieveImageTimeoutChanged( false ),
      mCalibImageChanged( false ),
      mTriggerScriptChanged( false ),
      mProjDirChanged( false ),
      mCamLibPathChanged( false ),
      mProjOpened( false ),
      mUseMask(false),
      mSaveUndistort(false)
{
    mpUi->setupUi(this);
    mpUi->projectBaseDir_lineEdit->setText( mProjDir );
    mpUi->calibImage_lineEdit->setText( mCalibrationImageProjLoc );
    mpUi->triggerScript_lineEdit->setText( mTriggeringScriptProjLoc );
    mpUi->retrieveImageTimeout_spinBox->setValue( mRetrieveImageTimeout );
    mpUi->defaultCalibImage_checkBox->setChecked( true );
    mpUi->defaultRetrieveImageTimeout_checkBox->setChecked( true );
    mpUi->defaultTriggerScript_checkBox->setChecked( true );
    mpUi->defaultCamLibrary_checkBox->setChecked( true );

    setModal( true );
    setWindowTitle( "Edit Project Settings");

}

ProjectDialog::~ProjectDialog()
{
    delete mpUi;
}

void ProjectDialog::closeEvent(QCloseEvent *event)
{
    event->ignore();
    hide();
}

void ProjectDialog::SetupNewProject( const QString& iDirectory )
{
    QDir new_proj( iDirectory );
    if ( new_proj.exists() )
    {
        QFile existing_proj_file( iDirectory + QDir::separator() + DEFAULT_PROJECT_FILE);

        if ( !existing_proj_file.exists())
        {
            mProjDir = iDirectory;

            QDir proj( mProjDir );
            mName = proj.dirName();
            mpUi->projectBaseDir_lineEdit->setText( mProjDir );
            mpUi->calibImage_lineEdit->setText( mProjDir +
                                                QDir::separator() +
                                                mCalibrationImageProjLoc );
            mpUi->triggerScript_lineEdit->setText( mProjDir +
                                                   QDir::separator() +
                                                   mTriggeringScriptProjLoc );
            WriteConfigFile();

            QDir().mkdir(mProjDir + QDir::separator() + DEFAULT_CALIB_IMAGE_DIR);
            QDir().mkdir(mProjDir + QDir::separator() + DEFAULT_SCRIPT_DIR);
            QDir().mkdir(mProjDir + QDir::separator() + DEFAULT_MASK_SAVE_DIR);
            QDir().mkdir(mProjDir + QDir::separator() + DEFAULT_DATA_SAVE_DIR);
            QDir().mkdir(mProjDir + QDir::separator() + DEFAULT_TRIANG_SAVE_DIR);

            //Copy QRC files into project so that the projects are standalone.
            QFile::copy( ":/" + DEFAULT_CALIB_IMAGE,
                         mProjDir +
                         QDir::separator() +
                         DEFAULT_CALIB_IMAGE_DIR +
                         QDir::separator() +
                         DEFAULT_CALIB_IMAGE );

            QFile::copy( ":/" + DEFAULT_TRIGGER_SCRIPT,
                         mProjDir +
                         QDir::separator() +
                         DEFAULT_SCRIPT_DIR +
                         QDir::separator() +
                         DEFAULT_TRIGGER_SCRIPT );

            QFile::copy( ":/" + DEFAULT_EXPORT_CONVERT_SCRIPT,
                         mProjDir +
                         QDir::separator() +
                         DEFAULT_SCRIPT_DIR +
                         QDir::separator() +
                         DEFAULT_EXPORT_CONVERT_SCRIPT );
            mProjOpened = true;
        }
        else
        {
            QMessageBox msg_box;
            msg_box.setText( "There is already a project in that directory. "
                            "Please choose another or delete the project.");
            msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
            msg_box.setWindowTitle("Error!");
            msg_box.exec();
            std::cerr << "There is already a project in that directory. "
                         "Please choose another or delete the project." << std::endl;
        }


    }
    else{
        std::cerr << "Proposed directory for project doesn't exist." << std::endl;
    }
}

void ProjectDialog::WriteConfigFile( ) const
{
    QString filename = mProjDir + QDir::separator() + DEFAULT_PROJECT_FILE;
    QFile::copy( filename, filename + ".backup" );

    std::stringstream ss_date;
    time_t t = time(0);
    struct tm * now = localtime(&t);
    ss_date << (now->tm_year + 1900) << '-' << (now->tm_mon + 1) << '-'
            << now->tm_mday;
    rapidjson::StringBuffer s;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(s);
    writer.StartObject();
    writer.Key("version");
    writer.String("1.0");

    writer.Key("ProjectDirectory");
    writer.String(mProjDir.toStdString().c_str());

    writer.Key("CalibrationImage");
    writer.String(mCalibrationImageProjLoc.toStdString().c_str());

    writer.Key("CalibrationImage_OrigLoc");
    writer.String(mCalibrationImageOrigLoc.toStdString().c_str());

    writer.Key("HardwareTriggerScript");
    writer.String(mTriggeringScriptProjLoc.toStdString().c_str());

    writer.Key("HardwareTriggerScript_OrigLoc");
    writer.String(mTriggeringScriptOrigLoc.toStdString().c_str());

    writer.Key("RetrieveImageTimeout");
    writer.Int( mRetrieveImageTimeout );

    writer.Key("CamLibPath");
    writer.String(mCamLibPath.toStdString().c_str());

    writer.Key("HLeftBound");
    writer.Int( mHLeftBound );
    writer.Key("SLeftBound");
    writer.Int( mSLeftBound );
    writer.Key("VLeftBound");
    writer.Int( mVLeftBound );

    writer.Key("HRightBound");
    writer.Int( mHRightBound );
    writer.Key("SRightBound");
    writer.Int( mSRightBound );
    writer.Key("VRightBound");
    writer.Int( mVRightBound );

    writer.Key("FilterSize");
    writer.Int( mFilterSize );
    writer.Key("FilterIterations");
    writer.Int( mFilterIterations );
    writer.Key("FilterThreshold");
    writer.Int( mFilterThreshold );

    writer.Key("NumPointsToTrack");
    writer.Int( mNumPointsToTrack );

    writer.EndObject();
    std::ofstream of(filename.toStdString());
    of << s.GetString();
    if (!of.good())
    {
        throw std::runtime_error("Can't write the JSON string to the file!");
    }
}

void ProjectDialog::ReadConfigFile( const QString& iProjConfig )
{
    QString config_file = iProjConfig;
    if (config_file.isEmpty())
    {
        config_file = mProjDir + QDir::separator() + DEFAULT_PROJECT_FILE;
    }
    std::fprintf( stderr, "Reading in Project Config file...\n" );
    std::ifstream file(config_file.toStdString());
    std::stringstream buffer;
    if (file)
    {
        buffer << file.rdbuf();
    }
    else
    {
        std::cerr << "File could not be opened!\n"; // Report error
        std::cerr << "File was \"" << config_file.toStdString() << "\"" << std::endl;
        std::cerr << "Error code: " << strerror(errno); // Get some info as to why
    }
    std::string buf_str = buffer.str();
    std::fprintf( stderr, "\nConfig file:\n %s\n\n", buf_str.c_str() );

    // 1. Parse a JSON config file into DOM.
    const char* json = buf_str.c_str();
    rapidjson::Document doc;


    //ParseResult result = doc.Parse(json);
    if (doc.Parse( json ).HasParseError())
    {
        fprintf(stderr,
                "\nError(offset %u): %s\n",
                (unsigned)doc.GetErrorOffset(),
                rapidjson:: GetParseError_En(doc.GetParseError()));
        exit( -1 );
    }

    assert( doc.HasMember("ProjectDirectory") );
    rapidjson::Value& tmp_val = doc["ProjectDirectory"];
    assert(tmp_val.IsString());
    mProjDir = tmp_val.GetString();

    QDir proj( mProjDir );
    mName = proj.dirName();
    mpUi->projectBaseDir_lineEdit->setText( mProjDir );

    assert( doc.HasMember("CalibrationImage") );
    tmp_val = doc["CalibrationImage"];
    assert(tmp_val.IsString());
    mCalibrationImageProjLoc = tmp_val.GetString();
    mpUi->calibImage_lineEdit->setText( mCalibrationImageProjLoc );

    const QString cal_img_loc = DEFAULT_CALIB_IMAGE_DIR + QDir::separator() + DEFAULT_CALIB_IMAGE;
    if (cal_img_loc == mCalibrationImageProjLoc)
    {
        mpUi->defaultCalibImage_checkBox->setChecked( true );
    }
    else
    {
        mpUi->defaultCalibImage_checkBox->setChecked( false );
    }

    assert( doc.HasMember("CalibrationImage_OrigLoc") );
    tmp_val = doc["CalibrationImage_OrigLoc"];
    assert(tmp_val.IsString());
    mCalibrationImageOrigLoc = tmp_val.GetString();

    assert( doc.HasMember("HardwareTriggerScript") );
    tmp_val = doc["HardwareTriggerScript"];
    assert(tmp_val.IsString());
    mTriggeringScriptProjLoc = tmp_val.GetString();
    mpUi->triggerScript_lineEdit->setText( mTriggeringScriptProjLoc );

    const QString trig_img_loc = DEFAULT_SCRIPT_DIR + QDir::separator() + DEFAULT_TRIGGER_SCRIPT;
    if (trig_img_loc == mTriggeringScriptProjLoc)
    {
        mpUi->defaultTriggerScript_checkBox->setChecked( true );
    }
    else {
        mpUi->defaultTriggerScript_checkBox->setChecked( false );
    }

    assert( doc.HasMember("HardwareTriggerScript_OrigLoc") );
    tmp_val = doc["HardwareTriggerScript_OrigLoc"];
    assert(tmp_val.IsString());
    mTriggeringScriptOrigLoc = tmp_val.GetString();

    assert( doc.HasMember("RetrieveImageTimeout") );
    assert( doc["RetrieveImageTimeout"].IsInt() );
    mRetrieveImageTimeout = doc["RetrieveImageTimeout"].GetInt();
    mpUi->retrieveImageTimeout_spinBox->setValue( mRetrieveImageTimeout );

    if (DEFAULT_IMAGE_RETRIEVE_TIMEOUT == mRetrieveImageTimeout)
    {
        mpUi->defaultRetrieveImageTimeout_checkBox->setChecked( true );
    }
    else
    {
        mpUi->defaultRetrieveImageTimeout_checkBox->setChecked( false );
    }

    if (doc.HasMember("CamLibPath"))
    {
        tmp_val = doc["CamLibPath"];
        assert(tmp_val.IsString());
        mCamLibPath = tmp_val.GetString();
    }
    else
    {
        mCamLibPath = DEFAULT_BASLER_CAMERA_LIBRARY;
    }

    mpUi->camLibrary_lineEdit->setText( mCamLibPath );
    if (DEFAULT_BASLER_CAMERA_LIBRARY == mCamLibPath)
    {
        mpUi->defaultCamLibrary_checkBox->setChecked( true );
    }
    else
    {
        mpUi->defaultCamLibrary_checkBox->setChecked( false );
    }

    mRetrieveImageTimeoutChanged = false;
    mCalibImageChanged = false;
    mTriggerScriptChanged = false;
    mProjDirChanged = false;
    mProjOpened = true;
    mCamLibPathChanged = false;

    assert( doc.HasMember("HLeftBound") );
    assert( doc["HLeftBound"].IsInt() );
    mHLeftBound = doc["HLeftBound"].GetInt();

    assert( doc.HasMember("SLeftBound") );
    assert( doc["SLeftBound"].IsInt() );
    mSLeftBound = doc["SLeftBound"].GetInt();

    assert( doc.HasMember("VLeftBound") );
    assert( doc["VLeftBound"].IsInt() );
    mVLeftBound = doc["VLeftBound"].GetInt();

    assert( doc.HasMember("HRightBound") );
    assert( doc["HRightBound"].IsInt() );
    mHRightBound = doc["HRightBound"].GetInt();

    assert( doc.HasMember("SRightBound") );
    assert( doc["SRightBound"].IsInt() );
    mSRightBound = doc["SRightBound"].GetInt();

    assert( doc.HasMember("VRightBound") );
    assert( doc["VRightBound"].IsInt() );
    mVRightBound = doc["VRightBound"].GetInt();

    assert( doc.HasMember("FilterSize") );
    assert( doc["FilterSize"].IsInt() );
    mFilterSize = doc["FilterSize"].GetInt();

    assert( doc.HasMember("FilterIterations") );
    assert( doc["FilterIterations"].IsInt() );
    mFilterIterations = doc["FilterIterations"].GetInt();

    assert( doc.HasMember("FilterThreshold") );
    assert( doc["FilterThreshold"].IsInt() );
    mFilterThreshold = doc["FilterThreshold"].GetInt();

    assert( doc.HasMember("NumPointsToTrack") );
    assert( doc["NumPointsToTrack"].IsInt() );
    mNumPointsToTrack = doc["NumPointsToTrack"].GetInt();
}

void ProjectDialog::on_calibImageBrowse_pushButton_clicked()
{

    assert( mpUi->defaultCalibImage_checkBox->isChecked() == false );

    QString file_name = QFileDialog::getOpenFileName(this, tr("Calibration Image File"),
                                                   "",
                                                    tr("Images (*.png *.bmp)"));
    file_name = QDir::toNativeSeparators(file_name);
    mCalibrationImageOrigLoc = file_name;
    mpUi->calibImage_lineEdit->setText( file_name );

    if (mpUi->calibImage_lineEdit->text() != mCalibrationImageOrigLoc )
    {
        mCalibImageChanged = true;
    }
}

void ProjectDialog::on_triggerScriptBrowse_pushButton_clicked()
{

    assert( mpUi->defaultTriggerScript_checkBox->isChecked() == false );

    QString file_name = QFileDialog::getOpenFileName(this, tr("Hardware Trigger Script"),
                                                   "",
                                                    tr("Scripts (*.py *.sh)"));
    file_name = QDir::toNativeSeparators(file_name);
    mTriggeringScriptOrigLoc = file_name;
    mpUi->triggerScript_lineEdit->setText( file_name );

    if( mpUi->triggerScript_lineEdit->text() != mTriggeringScriptOrigLoc )
    {
        mTriggerScriptChanged = true;
    }
}

void ProjectDialog::on_projectBaseBrowse_pushButton_clicked()
{
    // :NOTE: We don't currently allow the user to change the project base inside the application.
}


void ProjectDialog::on_buttonBox_accepted()
{
    bool save_changes = false;
    if ( mCalibImageChanged ||
         mTriggerScriptChanged ||
         mRetrieveImageTimeoutChanged ||
         mCamLibPathChanged )
    {
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(this, "Exiting", "Save changes to project settings?",
                                        QMessageBox::Yes|QMessageBox::No);
        save_changes = reply == QMessageBox::Yes;
    }

    if (save_changes)
    {
        if (mCalibImageChanged)
        {
            if (mpUi->calibImage_lineEdit->text() != DEFAULT_CALIB_IMAGE_DIR +
                                                     QDir::separator() +
                                                     DEFAULT_CALIB_IMAGE)
            {
                //Copying the file selected into the project hierarchy. If the file already exists,
                //we modify the name and save it. This gives us a history of
                //changes the the scripts.
                QFileInfo new_file( mCalibrationImageOrigLoc );
                QFileInfo file_copy_loc( DEFAULT_SCRIPT_DIR +
                                         QDir::separator() +
                                         new_file.completeBaseName() );
                if (file_copy_loc.exists())
                {
                    bool free_filename_found = false;
                    uint32_t count = 1;
                    while (!free_filename_found)
                    {
                        QFileInfo tmp_file( DEFAULT_SCRIPT_DIR +
                                            QDir::separator() +
                                            new_file.baseName() +
                                            "_copy" +
                                            QString(count) +
                                            new_file.suffix() );
                        if (tmp_file.exists())
                        {
                            ++count;
                        }
                        else
                        {
                            free_filename_found = true;
                        }
                        assert( count < 10 ); //10 is just a magic number chosen at whim

                    }
                    QString short_name = DEFAULT_SCRIPT_DIR +
                                         QDir::separator() +
                                         new_file.baseName() +
                                         "_copy" +
                                         QString(count) +
                                         new_file.suffix();
                    QFile::copy( mCalibrationImageOrigLoc, short_name );
                    mTriggeringScriptProjLoc = short_name;

                }
                else
                {
                    QFile::copy( mCalibrationImageOrigLoc, file_copy_loc.absoluteFilePath() );
                    mTriggeringScriptProjLoc = file_copy_loc.absoluteFilePath();

                }
            }
            else
            {
                mCalibrationImageOrigLoc = DEFAULT_CALIB_IMAGE_DIR +
                                           QDir::separator() +
                                           DEFAULT_CALIB_IMAGE;
                mCalibrationImageProjLoc = DEFAULT_CALIB_IMAGE_DIR +
                                           QDir::separator() +
                                           DEFAULT_CALIB_IMAGE;
            }
            mCalibImageChanged = false;
        }

        if (mTriggerScriptChanged)
        {
            if (mpUi->triggerScript_lineEdit->text() != DEFAULT_SCRIPT_DIR +
                                                        QDir::separator() +
                                                        DEFAULT_TRIGGER_SCRIPT)
            {
                //Copying the file selected into the project hierarchy. If the file already
                //exists, we modify the name and save it. This gives us a history of
                //changes the the scripts.
                QFileInfo new_file( mTriggeringScriptOrigLoc );
                QFileInfo file_copy_loc( DEFAULT_SCRIPT_DIR +
                                         QDir::separator() +
                                         new_file.completeBaseName() );
                if (file_copy_loc.exists())
                {
                    bool free_filename_found = false;
                    uint32_t count = 1;
                    while (!free_filename_found)
                    {
                        QFileInfo tmp_file( DEFAULT_SCRIPT_DIR +
                                            QDir::separator() +
                                            new_file.baseName() +
                                            "_copy" +
                                            QString(count) +
                                            new_file.suffix() );
                        if ( tmp_file.exists() )
                        {
                            ++count;
                        }
                        else{
                            free_filename_found = true;
                        }
                        assert( count < 10 );

                    }
                    QString short_name = DEFAULT_SCRIPT_DIR +
                                         QDir::separator() +
                                         new_file.baseName() +
                                         "_copy" +
                                         QString(count) +
                                         new_file.suffix();
                    QFile::copy( new_file.absoluteFilePath(), short_name );
                    mTriggeringScriptProjLoc = short_name;

                }
                else
                {
                    QFile::copy( mTriggeringScriptOrigLoc, file_copy_loc.absoluteFilePath() );
                    mTriggeringScriptProjLoc = file_copy_loc.absoluteFilePath();
                }
            }
            else
            {
                mTriggeringScriptOrigLoc = DEFAULT_SCRIPT_DIR +
                                           QDir::separator() +
                                           DEFAULT_TRIGGER_SCRIPT;

                mTriggeringScriptProjLoc = DEFAULT_SCRIPT_DIR +
                                           QDir::separator() +
                                           DEFAULT_TRIGGER_SCRIPT;
            }
            mTriggerScriptChanged = false;
        }

        if (mCamLibPathChanged)
        {
            mCamLibPath = mpUi->camLibrary_lineEdit->text();
            mCamLibPathChanged = false;
        }

        WriteConfigFile();
        hide();
    }
    else
    {
        on_buttonBox_rejected();
    }
}

void ProjectDialog::on_buttonBox_rejected()
{
    //Load the project settings from the config file to get rid of any changes made.
    ReadConfigFile( );
    hide();

}

void ProjectDialog::on_defaultCalibImage_checkBox_stateChanged(int iArg)
{
    if (mpUi->defaultCalibImage_checkBox->isChecked())
    {
        mpUi->calibImage_lineEdit->setEnabled( false );
        mpUi->calibImage_lineEdit->setReadOnly( true );
        mpUi->calibImageBrowse_pushButton->setEnabled( false );
        mpUi->calibImage_lineEdit->setText( DEFAULT_CALIB_IMAGE_DIR + QDir::separator() + DEFAULT_CALIB_IMAGE );
    }
    else
    {
        mpUi->calibImage_lineEdit->setEnabled( true );
        mpUi->calibImage_lineEdit->setReadOnly( false );
        mpUi->calibImageBrowse_pushButton->setEnabled( true );
    }

}


void ProjectDialog::on_defaultTriggerScript_checkBox_stateChanged(int iArg)
{
    if (mpUi->defaultTriggerScript_checkBox->isChecked())
    {
        mpUi->triggerScript_lineEdit->setEnabled( false );
        mpUi->triggerScript_lineEdit->setReadOnly( true );
        mpUi->triggerScriptBrowse_pushButton->setEnabled( false );
        mpUi->triggerScript_lineEdit->setText( DEFAULT_SCRIPT_DIR + QDir::separator() + DEFAULT_TRIGGER_SCRIPT );
        if (mpUi->triggerScript_lineEdit->text() != mTriggeringScriptProjLoc)
        {
            mTriggerScriptChanged = true;
        }
    }
    else
    {
        mpUi->triggerScript_lineEdit->setEnabled( true );
        mpUi->triggerScript_lineEdit->setReadOnly( false );
        mpUi->triggerScriptBrowse_pushButton->setEnabled( true );
    }
}

void ProjectDialog::on_defaultRetrieveImageTimeout_checkBox_stateChanged(int iArg)
{
    if (mpUi->defaultRetrieveImageTimeout_checkBox->isChecked())
    {
        mpUi->retrieveImageTimeout_spinBox->setEnabled( false );
        mpUi->retrieveImageTimeout_spinBox->setValue( DEFAULT_IMAGE_RETRIEVE_TIMEOUT );
    }
    else
    {
        mpUi->retrieveImageTimeout_spinBox->setEnabled( true );
    }

    //:NOTE: Don't need to do an update on the value or the flag for the change of this
    // as setting the value of the spinbox automatically triggers a check.
}

void ProjectDialog::on_retrieveImageTimeout_spinBox_valueChanged(int iNewVal)
{
    if (iNewVal >= 0 && static_cast<uint32_t>(iNewVal) != mRetrieveImageTimeout)
    {
        mRetrieveImageTimeoutChanged = true;
        mRetrieveImageTimeout = iNewVal;
    }
}


void ProjectDialog::on_defaultCamLibrary_checkBox_stateChanged(int iArg)
{
    if (mpUi->defaultCamLibrary_checkBox->isChecked())
    {
        mpUi->camLibrary_lineEdit->setEnabled( false );
        mpUi->camLibraryBrowse_pushButton->setEnabled( false );
        mpUi->camLibrary_lineEdit->setText( DEFAULT_BASLER_CAMERA_LIBRARY );
    }
    else
    {
        mpUi->camLibrary_lineEdit->setEnabled( true );
        mpUi->camLibraryBrowse_pushButton->setEnabled( true );
    }

    if (mpUi->camLibrary_lineEdit->text() != mCamLibPath)
    {
        mCamLibPathChanged = true;
    }
}

void ProjectDialog::on_camLibraryBrowse_pushButton_clicked()
{
    assert( mpUi->defaultCamLibrary_checkBox->isChecked() == false );

    mpUi->camLibraryBrowse_pushButton->setEnabled( true );
    mpUi->camLibrary_lineEdit->setEnabled( true );
    mpUi->camLibrary_lineEdit->setReadOnly( false );

    QString file_name = QFileDialog::getOpenFileName(this, tr("Camera Library"),
                                                   "",
                                                    tr("Shared Libraries (*.so)"));
    file_name = QDir::toNativeSeparators(file_name);
    mpUi->camLibrary_lineEdit->setText( file_name );
    mpUi->camLibrary_lineEdit->setReadOnly( true );

    if (mpUi->camLibrary_lineEdit->text() != mCamLibPath)
    {
        mCamLibPathChanged = true;
    }

}
