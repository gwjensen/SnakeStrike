#ifndef TrackingDialog_h
#define TrackingDialog_h

#include <QDialog>
#include <QProcess>
#include <QMutex>
#include <QFile>
#include <QMutexLocker>
#include <QEventLoop>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "common_types.h"

#include "TriangulationTask.h"
#include "CorrespondenceDialog.h"
#include "ProjectDialog.h"

namespace Ui {
class TrackingDialog;
}

class TrackingDialog : public QDialog
{
    Q_OBJECT

public:
    explicit TrackingDialog(ProjectDialog*& iProjInfo, QWidget* ipParent = 0);
    ~TrackingDialog();
    void SetThresholdBounds( uint32_t iLH,
                             uint32_t iLS,
                             uint32_t iLV,
                             uint32_t iRH,
                             uint32_t iRS,
                             uint32_t iRV );
    uint32_t FilterSize();
    uint32_t NumIterations();
    uint32_t LowerNoiseThreshold();
    uint32_t MaxNumPoints();
    void MaxNumPoints(const uint32_t iNewVal);

    cv::Scalar Leftbound();
    cv::Scalar Rightbound();

    void SetFilterSize( uint32_t iSize );
    void SetNumIterations( uint32_t iNum );
    void SetLowerNoiseThreshold( uint32_t iThresh);
    void RefreshFileInfo();
    void PopulateDataFiles();
    void PromptSaveChanges();
    bool PopulateTrackerConfig( TrackerConfigFile& oConfig, uint32_t& oCaptureCount, std::string& oTriDir );


public slots:
    void on_cancelTrackingButton_clicked();
    void on_startTrackingButton_clicked();
    void on_pushButton_leftbound_clicked();
    void on_pushButton_rightbound_clicked();
    void on_pushButton_previewThreshold_clicked();

    void on_spinBox_HLeftbound_editingFinished();
    void on_spinBox_HRightbound_editingFinished();
    void on_spinBox_SLeftbound_editingFinished();
    void on_spinBox_SRightbound_editingFinished();
    void on_spinBox_VLeftbound_editingFinished();
    void on_spinBox_VRightbound_editingFinished();


    void on_checkBox_undistortImages_toggled();
    void on_checkBox_saveUndistortedImages_toggled();
    void on_checkBox_vizUndistortedImages_toggled();
    void on_checkBox_vizThresholds_toggled();
    void on_checkBox_vizPointCorrespondences_toggled();
    void on_checkBox_vizCameraPose_toggled();
    void on_checkBox_mask_toggled();
    void on_ProcessFinished(int iErrCode);

    void PullNewVerboseOutput();
    void UserCorrespondenceHelp( TriangulationTask* iTaskWaitLoop,
                                std::vector< SmtImage > iTimestepImages,
                                const int iTimestepValue,
                                std::vector< std::vector< SmtPixel > > iTimestepMarkersFound,
                                std::pair<unsigned long, std::set<unsigned long> > iCamsToExclude,
                                const int iNumMarkersToFind);
    void WriteMarkedPoints();


protected:
    void closeEvent(QCloseEvent *event);
    void UpdateLeftboundButtonColor();
    void UpdateRightboundButtonColor();
    bool ProcessXMLFile( QString iXmlFile );
    void FreezeDialog( bool iFreezeDialog );

private:
    Ui::TrackingDialog* mpUi;
    QWidget*            mpParent;
    ProjectDialog*      mpProjInfo;
    QProcess*           mpProcess;
    QFile*              mpOutputFile;

    QMutex  mMutex;
    bool    mRunning;
    int     mNumCams;
    uint32_t          mLastCaptureCount;
    TrackerConfigFile mLastTrackerConfig;
    std::string       mLastTriDir;
    CorrespondenceDialog* mpCorresDialog;
    TriangulationTask* mpTriTask;

};

#endif // TrackingDialog_h
