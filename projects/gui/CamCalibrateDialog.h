#ifndef CamCalibrateDialog_h
#define CamCalibrateDialog_h

#include <QDialog>
#include <QProcess>
#include <QMutex>
#include <QMutexLocker>

#include "ProjectDialog.h"
#include "RecordDialog.h"

namespace Ui {
class CamCalibrateDialog;
}

class CamCalibrateDialog : public QDialog
{
    Q_OBJECT

    public:
        explicit CamCalibrateDialog(const ProjectDialog*& iProjDetails, QWidget* ipParent = 0);
        ~CamCalibrateDialog();

        bool CalibrationImagesXMLExists();
        bool FinishedCalibrationExists();

    public slots:
        void on_calibrateCameraButton_clicked();
        void on_stopButton_clicked();
        void on_checkBox_verboseoutput_toggled();
        void on_lineEdit_inputXML_textChanged();
        void on_ProcessFinished(int iErrCode);

        void PullNewVerboseOutput();
        void UpdateXMLFromRecord( const QString& iFilename );


    protected:
        void closeEvent(QCloseEvent* iEvent);
        void FreezeDialog( bool iFreezeDialog );
        bool ProcessXMLFile( const QString& iXmlFile );

        private slots:
        void on_checkBox_verboseoutput_clicked();

        void on_checkBox_useDefaultMinMatches_clicked();

        void on_recordCalibImagesButton_clicked();

        private:
        Ui::CamCalibrateDialog *mpUi;
        QString     mCalibrationImage;
        QString     mImagesXml;
        int         mNumCams;
        QProcess*   mpProcess;
        QWidget*    mpParent;
        const ProjectDialog* mpProjDetails;
        RecordDialog*        mpRecordDialog;

        QMutex  mMutex;
        QString mShellFilename;
};

#endif // CamCalibrateDialog_h
