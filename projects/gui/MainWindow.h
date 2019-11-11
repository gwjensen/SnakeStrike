#ifndef MainWindow_h
#define MainWindow_h

#include <QMainWindow>
#include <QMdiSubWindow>
#include <QFileDialog>

#include "CamPreviewWindow.h"
#include "CamPreviewCanvas.h"
#include "CamCalibrateDialog.h"
#include "TrackingDialog.h"
#include "RecordDialog.h"
#include "VisualizationDialog.h"
#include "CountdownDialog.h"
#include "CamInfoTree.h"
#include "ProjectDialog.h"
#include "ExportDialog.h"
#include "GuiCamArray.h"
#include "HelpDialog.h"

const uint32_t DEFAULT_HERZ_CALIB_UPPER_END = 15;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
        Q_OBJECT

    public:
        explicit MainWindow(QWidget *parent = 0);
        ~MainWindow();
        bool GetLabel( const int iLabelIdx, CamPreviewCanvas*& oLabelPtr );
        uint32_t CamHz( ) const { return mCamHerz; }
        RecordDialog* CreateRecordDialog(const RecordType& iRecType, bool iXmlAlreadyExists );
        void UpdateState();
        void RefreshPreviewWindows();


    protected:
        void closeEvent(QCloseEvent *event) override;

    private slots:
        void on_openProjectButton_clicked();
        void on_createProjectButton_clicked();
        void on_editProjectSettingsButton_clicked();
        void on_previewButton_clicked();
        void on_recordButton_clicked();
        void on_trackingButton_clicked();
        void on_calibrateButton_clicked();
        void on_pushButton_export_clicked();
        void on_visualisationButton_clicked();
        void on_camConfigButton_clicked();
        void on_getMaskButton_clicked();
        void on_camConfigButton_2_clicked();
        void on_refreshCamerasButton_clicked();

        CamPreviewWindow* CreatePreviewWindow(int iCamIdx);
        void UpdateCamPreviewWindow(int iCamIdx, QPixmap image);
        void RefreshCameras();
        void UpdateProjectSettings();

        void on_previewButton_2_clicked();

    private:
        CamPreviewWindow* ActivePreviewWindow() const;
        CamPreviewWindow* FindPreviewWindow( const int iCamIdx ) const;
        void RefreshCameraTree(const QStringList& iCameraNames);
        void HideOptionsUnopenedProject( bool iHide );
        void InitDependentDialogs();

        void UpdateHzInfo();

        Ui::MainWindow*         mpUi;
        CamCalibrateDialog*     mpCamCalDialog;
        ProjectDialog*          mpProjDialog;
        RecordDialog*           mpRecordDialog;
        TrackingDialog*         mpTrackDialog;
        QFileDialog*            mpFileDialog;
        CameraInfoTree*         mpTreeModel;
        VisualizationDialog*    mpVis3dDialog;
        ExportDialog*           mpExportDialog;
        HelpDialog*             mpHelpDialog;
        CorrespondenceDialog*   mpCorresDialog;

        QDir mDir;
        QVector< CamPreviewCanvas* > mCamPreviewList;
        QVector< CamPreviewWindow* > mCamWindowList;
        GuiCamArray* mpCamList;
        bool mPlay;
        uint32_t mCamHerz;

};

#endif // MainWindow_h
