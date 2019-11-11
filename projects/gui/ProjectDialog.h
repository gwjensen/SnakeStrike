#ifndef ProjectDialog_h
#define ProjectDialog_h

#include <QDialog>
#include <QDir>


const uint32_t DEFAULT_IMAGE_RETRIEVE_TIMEOUT = 200;
const QString DEFAULT_DATA_SAVE_DIR = "data";
const QString DEFAULT_DATA_SAVE_NAME_BASE = "capture";
const QString DEFAULT_CALIB_IMAGE = "default_calibration_pattern.png";
const QString DEFAULT_CALIB_IMAGE_DIR = "calib";
const QString DEFAULT_CALIB_IMAGES_XML = "calibration_images.xml";
const QString DEFAULT_CALIB_DONE_XML = "multi-camera-results.xml";
const QString DEFAULT_MASK_IMAGES_XML = "mask_images.xml";
const QString DEFAULT_DATA_IMAGES_XML = "data_images.xml";
const QString DEFAULT_MASK_SAVE_DIR = "mask";
const QString DEFAULT_TRIGGER_SCRIPT = "LabJackDigitalOutputCycle.py";
const QString DEFAULT_EXPORT_CONVERT_SCRIPT = "ConvertCVDataXMLToCSV.py";
const QString DEFAULT_SCRIPT_DIR = "scripts";
const QString DEFAULT_PROJECT_FILE = "proj_config.spt";
const QString DEFAULT_TRIANG_SAVE_DIR = "triangulation";
const QString DEFAULT_TRIANG_FILE_BASE = "TriangulatedPoints";
const QString DEFAULT_SPT_CONFIG_NAME = "SPTConfig";
const QString DEFAULT_UNDISTORTED_DATA_NAME_BASE = "capture-undistorted";
const QString DEFAULT_BASLER_CAMERA_LIBRARY = "libBaslerCams.so";

class MainWindow;

namespace Ui {
    class ProjectDialog;
}

class ProjectDialog : public QDialog
{
        Q_OBJECT

    public:
        explicit ProjectDialog(QWidget *parent = nullptr);
        ~ProjectDialog();
        void SetupNewProject( const QString& iDirectory );

        void WriteConfigFile( ) const;
        void ReadConfigFile( const QString& iProjDir="" );

        const QString Name() const{ return mName;}

        const QString ProjectDir() const { return mProjDir; }
        const QString CalibDir() const { return mSaveCalibDir; }
        const QString MaskDir() const { return mSaveMaskDir; }
        const QString DataDir() const { return mSaveDataDir; }
        const QString DataFileNameBase() const { return mSaveDataFilenameBase; }
        const QString ProjectCalibImage() const { return mCalibrationImageProjLoc; }
        const QString ProjectTriggerScript() const { return mTriggeringScriptProjLoc; }
        const QString ProjectMaskXml() const { return mSaveMaskDir + QDir::separator() + DEFAULT_MASK_IMAGES_XML; }
        //const QString ProjectDataXml() const { return mSaveDataDir + QDir::separator() + DEFAULT_DATA_IMAGES_XML; }
        const QString ProjectCalibXml() const { return mSaveCalibDir + QDir::separator() + DEFAULT_CALIB_IMAGES_XML; }
        const QString FinishedCalibXml() const { return mSaveCalibDir + QDir::separator() + DEFAULT_CALIB_DONE_XML; }
        const QString TriangDir() const { return mSaveTriangDir; }
        const QString TriangNameBase() const { return mSaveTriangFilenameBase; }
        const QString CamLibPath() const { return mCamLibPath; }
        static const QString DefaultCamLibPath() { return DEFAULT_BASLER_CAMERA_LIBRARY; }
        uint64_t ImageRetrieveTimeout() const { return mRetrieveImageTimeout; }

        bool& UseMask() { return mUseMask;}
        bool UseMask() const { return mUseMask;}
        bool& SaveUndistort() { return mSaveUndistort;}
        bool SaveUndistort() const { return mSaveUndistort; }

        bool ProjectOpened() const { return mProjOpened; }
        uint32_t HLeftBound() const { return mHLeftBound; }
        uint32_t SLeftBound() const { return mSLeftBound; }
        uint32_t VLeftBound() const { return mVLeftBound; }
        uint32_t HRightBound() const { return mHRightBound; }
        uint32_t SRightBound() const { return mSRightBound; }
        uint32_t VRightBound() const { return mVRightBound; }
        uint32_t FilterSize() const { return mFilterSize; }
        uint32_t FilterIterations() const { return mFilterIterations; }
        uint32_t FilterThreshold() const { return mFilterThreshold; }
        uint32_t NumPointsToTrack() const { return mNumPointsToTrack; }

        uint32_t& HLeftBound()  { mThresholdParmsChanged = true; return mHLeftBound; }
        uint32_t& SLeftBound()  { mThresholdParmsChanged = true; return mSLeftBound; }
        uint32_t& VLeftBound()  { mThresholdParmsChanged = true; return mVLeftBound; }
        uint32_t& HRightBound()  { mThresholdParmsChanged = true; return mHRightBound; }
        uint32_t& SRightBound()  { mThresholdParmsChanged = true; return mSRightBound; }
        uint32_t& VRightBound()  { mThresholdParmsChanged = true; return mVRightBound; }
        uint32_t& FilterSize()  { mThresholdParmsChanged = true; return mFilterSize; }
        uint32_t& FilterIterations()  { mThresholdParmsChanged = true; return mFilterIterations; }
        uint32_t& FilterThreshold()  { mThresholdParmsChanged = true; return mFilterThreshold; }
        uint32_t& NumPointsToTrack()  { mThresholdParmsChanged = true; return mNumPointsToTrack; }



    public slots:
         //void on_outputBrowseButton_clicked();
         //void on_numImages_valueChanged(int arg1);



    private slots:
        void on_buttonBox_accepted();
        void on_buttonBox_rejected();
        void on_calibImageBrowse_pushButton_clicked();
        void on_triggerScriptBrowse_pushButton_clicked();
        void on_projectBaseBrowse_pushButton_clicked();
        void on_defaultCalibImage_checkBox_stateChanged(int iArg);
        void on_defaultTriggerScript_checkBox_stateChanged(int iArg);
        void on_defaultRetrieveImageTimeout_checkBox_stateChanged(int iArg);
        void on_retrieveImageTimeout_spinBox_valueChanged(int iArg);
        void on_defaultCamLibrary_checkBox_stateChanged(int iArg);
        void on_camLibraryBrowse_pushButton_clicked();

    signals:
        void SignalRetrieveImageTimeoutChanged();


    protected:
        void closeEvent(QCloseEvent *event);

    private:
        Ui::ProjectDialog* mpUi;
        QString  mCalibrationImageOrigLoc;
        QString  mCalibrationImageProjLoc;
        QString  mTriggeringScriptOrigLoc;
        QString  mTriggeringScriptProjLoc;
        QString  mExportConvertScriptLoc;
        QString  mSaveDataDir;
        QString  mSaveDataFilenameBase;
        QString  mSaveCalibDir;
        QString  mSaveMaskDir;
        QString  mCamLibPath;
        uint64_t mRetrieveImageTimeout;

        QString mProjDir;
        QString mName;
        QString mSaveTriangDir;
        QString mSaveTriangFilenameBase;

        uint32_t       mHLeftBound;
        uint32_t       mSLeftBound;
        uint32_t       mVLeftBound;
        uint32_t       mHRightBound;
        uint32_t       mSRightBound;
        uint32_t       mVRightBound;
        uint32_t       mFilterSize;
        uint32_t       mFilterIterations;
        uint32_t       mFilterThreshold;
        uint32_t       mNumPointsToTrack;
        QVector<int>   mCamsToNotInclude;

        bool mThresholdParmsChanged;
        bool mRetrieveImageTimeoutChanged;
        bool mCalibImageChanged;
        bool mTriggerScriptChanged;
        bool mProjDirChanged;
        bool mCamLibPathChanged;

        bool mProjOpened;
        bool mUseMask;
        bool mSaveUndistort;
};

#endif // ProjectDialog_h
