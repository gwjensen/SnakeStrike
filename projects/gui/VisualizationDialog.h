#ifndef VisualizationDialog_h
#define VisualizationDialog_h

#include <QDialog>
#include <QMutex>
#include <QReadWriteLock>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include "ProjectDialog.h"

namespace Ui {
class VisualizationDialog;
}

class VisualizationDialog : public QDialog
{
    Q_OBJECT

    public:
        explicit VisualizationDialog(ProjectDialog*& iProjDetails, QWidget* ipParent = 0);
        ~VisualizationDialog();
        bool PopulateTrackingFiles();


    public slots:
        void on_pushButton_view_clicked();
        void on_pushButton_cancel_clicked();
        void on_pushButton_triPointFileNew_clicked();
        void on_pushButton_resetView_clicked();
        void on_pushButton_viewClear_clicked();

        void on_doubleSpinBox_pointNoise_editingFinished();
        void on_doubleSpinBox_updateMultiplier_editingFinished();
        void on_spinBox_numTrajPoints_editingFinished();
        void on_spinBox_trajTailLength_editingFinished();
        void on_spinBox_viewTimestep_editingFinished();

        void on_checkBox_smooth_toggled();
        void on_checkBox_trace_toggled();
        void on_checkBox_renderOffset_toggled();
        void on_checkBox_triPointFileNew_toggled();
        void on_checkBox_centerX_toggled();
        void on_checkBox_centerY_toggled();
        void on_checkBox_centerZ_toggled();
        void on_fileSelectionBox_currentTextChanged(const QString& iNewFile);


    protected:
        void closeEvent(QCloseEvent* iEvent);
//        void ReadInPolyHull( QString iDirectory,
//                            uint32_t iTotalNumTimesteps,
//                            std::vector< vtkSmartPointer<vtkPolyData> >& oPolyData );


    private:
        Ui::VisualizationDialog* mpUi;
        ProjectDialog*           mpProjDetails;
        uint32_t    mMaxNumPoints;
        int         mMaxTimesteps;
        int         mNumMissingTimesteps;
        double      mPointNoise;
        double      mUpdateMultiplier;
        double      mOffsetX;
        double      mOffsetY;
        double      mOffsetZ;
        bool        mStopCalled;
        bool        mViewRunning;
        int         mCurTimestep;

        QMutex          mMutex;
        QReadWriteLock  mRwLock;
        std::vector< std::vector< cv::Point3d > >       mTriangulatedPointsCV;
        std::vector< std::vector< pcl::PointXYZRGB > >  mTriangulatedPointsPCL;
        pcl::PointCloud< pcl::PointXYZRGB >::Ptr        mCloud;

        boost::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;
       // boost::shared_ptr<pcl::visualization::PCLVisualizer> mHullviewer;
       // boost::shared_ptr<opensource::EPVHInterface>         mVisualHullInterface;
       // std::vector< vtkSmartPointer<vtkPolyData> >          mPolyDataVec;

        pcl::PolygonMesh::Ptr mHullmesh;

        bool ProcessXMLFile( QString iXmlFile );
        bool ProcessCSVFile( QString iCsvFile );
        void FreezeDialog( bool iFreeze );

        void VisualizePoints( const std::vector< std::vector< pcl::PointXYZRGB > >& iTriangulatedPoints,
                              const int iNumTracePoints,
                              const int iLengthTraceHistory,
                              const bool iCenterX = false,
                              const bool iCenterY = false,
                              const bool iCenterZ = false,
                              int iStartTimeStep = 0,
                              int iEndTimeStep = -1 );
        void UpdateStartButtonText( bool iPauseText );
};

#endif // VisualizationDialog_h
