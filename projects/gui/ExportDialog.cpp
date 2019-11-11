#include <QFileDialog>
#include <QCloseEvent>
#include <QMessageBox>
#include <QProcess>
#include <iostream>
#include <QDirIterator>
#include <QDialogButtonBox>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ui_ExportDialog.h"
#include "ExportDialog.h"

ExportDialog::ExportDialog(ProjectDialog*& iProjDetails, QWidget* ipParent)
    : QDialog(ipParent),
      mpUi(new Ui::ExportDialog),
      mpProjDetails( iProjDetails)
{
    mpUi->setupUi(this);
    mpUi->spinBox_endTimestep->setMaximum( 0 );
    mpUi->spinBox_endTimestep->setValue( 0 );
    mpUi->spinBox_startTimestep->setMaximum( 0 );
    mpUi->spinBox_startTimestep->setValue( 0 );
}

ExportDialog::~ExportDialog()
{
    delete mpUi;
}

void ExportDialog::PopulateTrackingFiles()
{
    mpUi->fileSelectionBox->clear();

    QRegExp rx( mpProjDetails->TriangNameBase() + "\\_\\d+\\.xml");
    QString directory(mpProjDetails->ProjectDir() + QDir::separator() + mpProjDetails->TriangDir());
    QStringList xml_files;
    QDirIterator it(directory, QStringList() << "*.xml", QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext())
    {
        xml_files << it.next();
    }
    xml_files.sort();

    if (0 == xml_files.size())
    {
        QMessageBox msg_box;
        msg_box.setText( "No triangulation files found in project.");
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Export");
        msg_box.exec();
        return;
    }

    foreach (QString filename, xml_files)
    {
        std::cerr << "XML filename: " << filename.toStdString() << std::endl;
        QFileInfo file( filename );
        if (rx.exactMatch( file.fileName() ))
        {
            mpUi->fileSelectionBox->addItem( filename.remove( mpProjDetails->ProjectDir() +
                                                              QDir::separator() +
                                                              mpProjDetails->TriangDir() +
                                                              QDir::separator() ) );
        }
    }
}

void ExportDialog::closeEvent(QCloseEvent* iEvent)
{
    iEvent->ignore();
    hide();
}

void ExportDialog::on_pushButton_cancel_clicked()
{
    hide();
}

void ExportDialog::on_pushButton_export_clicked()
{
    FreezeDialog( true );

    QProcess* process = new QProcess(this);
    process->setProcessChannelMode(QProcess::MergedChannels);

    QString shellfilename = mpProjDetails->ProjectDir() +
                            QDir::separator() +
                            DEFAULT_SCRIPT_DIR +
                            QDir::separator() +
                            DEFAULT_EXPORT_CONVERT_SCRIPT;

    if ("" == mpUi->fileSelectionBox->currentText())
    {
        QMessageBox msg_box;
        msg_box.setText( "No source file selected.");
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Export Error");
        msg_box.exec();
        FreezeDialog( false );
        return;
    }

    if ("" == mpUi->lineEdit_saveFilename->text())
    {
        QMessageBox msg_box;
        msg_box.setText( "No output file given.");
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Export Error");
        msg_box.exec();
        FreezeDialog( false );
        return;
    }

    QFile dest_dir( mpUi->lineEdit_saveFilename->text() );
    if (dest_dir.exists())
    {
        QMessageBox msg_box;
        msg_box.setText( "The destination file already exists. Please choose another name.");
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Export Error");
        msg_box.exec();
        FreezeDialog( false );
        return;
    }

    QStringList args;
    QString input_file( mpProjDetails->ProjectDir() +
                        QDir::separator() +
                        mpProjDetails->TriangDir() +
                        QDir::separator() +
                        mpUi->fileSelectionBox->currentText() );
    QString out_file( mpUi->lineEdit_saveFilename->text() );
    QFileInfo out_file_info( mpUi->lineEdit_saveFilename->text() );
    if ("csv" != out_file_info.suffix())
    {
        out_file += ".csv";
    }
    args << shellfilename
         << input_file
         << out_file
         << QString::number(mpUi->spinBox_startTimestep->value())
         << QString::number( mpUi->spinBox_endTimestep->value()) ;
    process->start( "python", args );

    process->waitForFinished(-1);
    FreezeDialog( false );

    QMessageBox msg_box;
    msg_box.setText( "The selected file has been exported.");
    msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
    msg_box.setWindowTitle("Export Complete");
    msg_box.exec();
}

void ExportDialog::on_fileSelectionBox_currentTextChanged(const QString& iText)
{
    if ("" != iText)
    {
        cv::FileStorage f((mpProjDetails->ProjectDir() +
                           QDir::separator() +
                           mpProjDetails->TriangDir() +
                           QDir::separator() +
                           iText).toStdString().c_str(),
                          cv::FileStorage::READ);
        cv::FileNode fs = f["SimplePointTracking"];

        if (fs.isNone())
        {
            QMessageBox msg_box;
            msg_box.setText( "Selected file is not a valid triangulated points file.");
            msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
            msg_box.setWindowTitle("Error!");
            msg_box.exec();
        }
        else
        {
            int max_timesteps;
            fs["MaxTimestep"] >> max_timesteps;
            if (0 == max_timesteps)
            {
                fprintf( stderr, "MaxTimesteps not found when reading XML File.\n" );
            }
            else
            {
                mpUi->spinBox_endTimestep->setMaximum( max_timesteps );
                mpUi->spinBox_endTimestep->setValue( max_timesteps );
                mpUi->spinBox_startTimestep->setMaximum( max_timesteps );
                mpUi->spinBox_startTimestep->setValue( 0 );
            }
        }
        f.release();
    }
    else
    {
        mpUi->spinBox_endTimestep->setMaximum( 0 );
        mpUi->spinBox_endTimestep->setValue( 0 );
        mpUi->spinBox_startTimestep->setMaximum( 0 );
        mpUi->spinBox_startTimestep->setValue( 0 );
    }
}

void ExportDialog::on_pushButton_triSave_clicked()
{
    QString file_name = QFileDialog::getSaveFileName(this, tr("Save File"),
                               mpUi->lineEdit_saveFilename->text(),
                               tr("CSV Files (*.csv)") );
    if (file_name.size() > 0)
    {
        mpUi->lineEdit_saveFilename->setText( file_name );
    }
}


void ExportDialog::FreezeDialog( bool iFreeze )
{
    mpUi->lineEdit_saveFilename->setEnabled( !iFreeze );
    mpUi->spinBox_startTimestep->setEnabled( !iFreeze );
    mpUi->spinBox_endTimestep->setEnabled( !iFreeze );
    mpUi->pushButton_export->setEnabled( !iFreeze );
}


void ExportDialog::on_spinBox_startTimestep_editingFinished()
{
    if (mpUi->spinBox_startTimestep->value() > mpUi->spinBox_endTimestep->value())
    {
        mpUi->spinBox_startTimestep->setValue( mpUi->spinBox_endTimestep->value() );
    }
}

void ExportDialog::on_spinBox_endTimestep_editingFinished()
{
    if (mpUi->spinBox_startTimestep->value() > mpUi->spinBox_endTimestep->value())
    {
        mpUi->spinBox_endTimestep->setValue( mpUi->spinBox_startTimestep->value() );
    }
}
