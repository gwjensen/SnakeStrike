#ifndef RecordDialog_h
#define RecordDialog_h

#include <QDialog>
#include <QFileDialog>
#include <QProcess>
#include <QMessageBox>
#include <QString>

#include "GuiCamArray.h"
#include "BufferingDialog.h"
#include "ProjectDialog.h"

namespace Ui {
class RecordDialog;
}

enum RecordType
{
    CALIBRATION = 0x01,
           MASK = 0x02,
           DATA = 0x03
};

class RecordDialog : public QDialog
{
    Q_OBJECT

public:
    explicit RecordDialog(GuiCamArray*& iCamList,
                          const ProjectDialog*& iProjDetails,
                          const RecordType& iRecordAction,
                          uint32_t iCamHerz,
                          bool iXmlAlreadyExists,
                          QWidget* ipParent = 0);
    ~RecordDialog();

signals:
    void SignalCreateCVConfigFile( QString iOutputDir );
    void SignalConfigImageXmlFile( QString iFullFilePath );

public slots:
     void on_startRecordingButton_clicked();
     void on_stopRecordingButton_clicked();
     void on_numImages_valueChanged(int arg1);
     void on_radioButton_hardwareTrigger_clicked();
     void on_radioButton_softwareTrigger_clicked();

     void UpdateCamDoneGrabbing();
     void UpdateCamStartGrabbing();
     void RecordingDone();

private slots:
     void CreateCVConfigFile( QString iOutputDir );
     void on_checkBox_makeVideo_stateChanged();

protected:
    void keyPressEvent ( QKeyEvent * event );
    void closeEvent(QCloseEvent *event);
    void FreezeDialog( bool iFreeze );
    void ConfigureDialogForType();


private:
    Ui::RecordDialog*       mpUi;
    const ProjectDialog*    mpProjDetails;
    QFileDialog*            mpFileDialog;
    BufferingDialog*        mpBufDialog;
    GuiCamArray*            mpCamList;
    QMessageBox*            mpCapturingMsg;
    QProcess*               mpProcess;

    int         mRunCount;
    int         mBackUpImageCount;
    bool        mCancelled;
    bool        mRecording;
    bool        mKeyPressed;
    uint32_t    mCamHerz;
    QString     mNewCapturePath;
    uint32_t    mCaptureCount;
    bool        mXmlAlreadyExists;
    QMutex      mMutex;

    const RecordType mType;
};

#endif // RecordDialog_h
