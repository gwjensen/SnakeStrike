#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <QCloseEvent>
#include <QDirIterator>
#include <QFileDialog>
#include <QMessageBox>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>
#include <QThread>
#include <QDirIterator>

//#include "EPVH/epvhinterface.h"

#include "ui_Visualization.h"
#include "VisualizationDialog.h"

VisualizationDialog::VisualizationDialog(ProjectDialog*& iProjDetails, QWidget* ipParent) :
    QDialog(ipParent),
    mpUi(new Ui::VisualizationDialog),
    mpProjDetails( iProjDetails ),
    mMaxNumPoints(0),
    mMaxTimesteps(0),
    mNumMissingTimesteps(0),
    mPointNoise(0),
    mUpdateMultiplier(0),
    mOffsetX(0),
    mOffsetY(0),
    mOffsetZ(0),
    mStopCalled(false),
    mViewRunning(false),
    mCurTimestep(0)
{
    mpUi->setupUi(this);

    mCloud.reset( new pcl::PointCloud< pcl::PointXYZRGB > );
    mCloud->width = 4; //pixelsTriangulated;
    mCloud->height = 1;
    mCloud->is_dense = false;
    mCloud->points.resize ( 30 );

    mViewer.reset( new pcl::visualization::PCLVisualizer("Simple Point Tracking", false) );
    mViewer->addCoordinateSystem( 50.0, "axis", 0 );
    mViewer->setBackgroundColor( 255, 255, 255, 0 );	// Setting background to a white
    mpUi->qvtkwidget->SetRenderWindow( mViewer->getRenderWindow() );
    mViewer->setupInteractor( mpUi->qvtkwidget->GetInteractor(),
                              mpUi->qvtkwidget->GetRenderWindow() );
    mpUi->qvtkwidget->update();

    //Add connections for signals on button changes
    mViewer->addPointCloud( mCloud, "spt");
    mViewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                               5,
                                               "spt" );

    mViewer->resetCamera();
    mpUi->qvtkwidget->update();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);

    //Setup viewer for visual hull
    //mVisualHullInterface.reset( new opensource::EPVHInterface() );
    //mHullviewer.reset( new pcl::visualization::PCLVisualizer("Hull Viewer", false) );
    //mHullviewer->setupInteractor( mpUi->qvtkwidget->GetInteractor(),
    //                              mpUi->qvtkwidget->GetRenderWindow());

    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
}

VisualizationDialog::~VisualizationDialog()
{
    delete mpUi;
}

bool VisualizationDialog::PopulateTrackingFiles()
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
        msg_box.setWindowTitle("Visualize");
        msg_box.exec();
        return false;
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
    return true;
}

void VisualizationDialog::VisualizePoints( const std::vector< std::vector< pcl::PointXYZRGB > >& iTriangulatedPoints,
                                                            const int iNumTracePoints,
                                                            const int iLengthTraceHistory,
                                                            const bool iOffsetX,
                                                            const bool iOffsetY,
                                                            const bool iOffsetZ,
                                                            int iStartTimeStep,
                                                            int iEndTimeStep)
{
    if (iEndTimeStep > 0 && iStartTimeStep > iEndTimeStep)
    {
        mMutex.lock();
        mViewRunning = false;
        mStopCalled = false;
        mMutex.unlock();
        UpdateStartButtonText( false );

        std::fprintf(stderr, "The starting Timestep must be smaller than the ending timestep.\n");
        return;
    }
    mMutex.lock();
    mViewRunning = true;
    mMutex.unlock();
    std::vector< std::vector< pcl::PointXYZRGB > >::const_iterator timestep_iter = iTriangulatedPoints.begin();
    std::vector< std::vector< pcl::PointXYZRGB > >::const_iterator timestep_iter_end = iTriangulatedPoints.end();
    std::advance( timestep_iter, iStartTimeStep );

    float check_val = 0.123456;
    std::vector< pcl::PointXYZ > lineOldPointList;
    std::vector< pcl::PointXYZ >lineNewPointList;

    pcl::PointXYZ offset( 0, 0, 0 );
    if (iOffsetX)
    {
        offset.x = static_cast<float>(mOffsetX);
    }

    if (iOffsetY)
    {
        offset.y = static_cast<float>(mOffsetY);
    }

    if (iOffsetZ)
    {
        offset.z = static_cast<float>(mOffsetZ);
    }

    for (int i=0; i < iNumTracePoints; ++i)
    {
        pcl::PointXYZ old_point( check_val, check_val, check_val);
        lineOldPointList.push_back( old_point );

        pcl::PointXYZ new_point(check_val, check_val,check_val );
        lineNewPointList.push_back( new_point );
    }
    if (iEndTimeStep >= 0)
    {
        timestep_iter_end = iTriangulatedPoints.begin();
        std::advance( timestep_iter_end, iEndTimeStep );
    }
    int sleep_time = mpUi->spinBox_waitTime->value();
    int timestep = iStartTimeStep;
    //bool show_vis_hull = mpUi->checkBox_showVisualHull->isChecked();
    for (; timestep_iter != timestep_iter_end; ++timestep_iter, ++timestep)
    {
//        if (show_vis_hull && mPolyDataVec.size() > 0)
//        {
//            mHullviewer->removeAllShapes();
//            mHullviewer->addModelFromPolyData( mPolyDataVec[timestep],"poly_data", 0 );

//            QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
//        }
        if (mStopCalled)
        {
            break;
        }

        if (timestep_iter->size() > mMaxNumPoints)
        {
            std::fprintf(stderr, "Should not have more points than the max.\n");
            assert(false);
        }

        QThread::msleep( sleep_time );
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);

        if (0 == timestep_iter->size())
        {
            mViewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY,
                                                       0.0,
                                                       "spt" );
            continue;
        }
        else
        {
            mViewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY,
                                                      1.0,
                                                      "spt" );
        }
        mpUi->qvtkwidget->update();

        if (timestep >= iLengthTraceHistory && iLengthTraceHistory > 0)
        {
            for (int i =0; i < iNumTracePoints; ++i)
            {
                if (mStopCalled)
                {
                    break;
                }

                std::string line_name = "line" +
                                        std::to_string(i) +
                                        "_" +
                                        std::to_string(timestep - iLengthTraceHistory);
                mViewer->removeShape( line_name );
            }
        }

        if (timestep > 0)
        {
            for (uint32_t i =0; i < timestep_iter->size(); ++i)
            {
                if (0 == i)
                {
                    //viewer->removeShape("sphere");
                }
                else{
                    std::string segmentName = "point" +
                                              std::to_string( i - 1 ) +
                                              "_to_point" +
                                              std::to_string( i );
                    mViewer->removeShape( segmentName );
                }
            }
        }

        for (uint32_t i = 0; i < timestep_iter->size(); ++i)
        { //i is cluster pixel location number
            mCloud->points[i].x = (*timestep_iter)[i].x - offset.x;
            mCloud->points[i].y = (*timestep_iter)[i].y - offset.y;
            mCloud->points[i].z = (*timestep_iter)[i].z - offset.z;

            if (i != 0)
            {
                std::string segmentName = "point" +
                                          std::to_string( i - 1 ) +
                                          "_to_point" +
                                          std::to_string( i );
                mViewer->addLine( mCloud->points[ i -1 ],
                                  mCloud->points[ i ],
                                  255, 0 , 0, segmentName  );
            }
            else
            {
                //Making the firs point easier to see.
                //viewer->addSphere (cloud->points[0], 4, 255, 0, 0.0, "sphere");
            }

            switch( i % 10 )
            {
                case 0:
                    mCloud->points[i].r = (uint8_t)255;
                    mCloud->points[i].g = (uint8_t)0;
                    mCloud->points[i].b = (uint8_t)0;
                    break;
                case 1:
                    mCloud->points[i].r = (uint8_t)0;
                    mCloud->points[i].g = (uint8_t)255;
                    mCloud->points[i].b = (uint8_t)0;
                    break;
                case 2:
                    mCloud->points[i].r = (uint8_t)0;
                    mCloud->points[i].g = (uint8_t)0;
                    mCloud->points[i].b = (uint8_t)255;
                    break;
                case 3:
                    mCloud->points[i].r = (uint8_t)255;
                    mCloud->points[i].g = (uint8_t)0;
                    mCloud->points[i].b = (uint8_t)255;
                    break;
                case 4:
                    mCloud->points[i].r = (uint8_t)255;
                    mCloud->points[i].g = (uint8_t)255;
                    mCloud->points[i].b = (uint8_t)0;
                    break;
                case 5:
                    mCloud->points[i].r = (uint8_t)0;
                    mCloud->points[i].g = (uint8_t)255;
                    mCloud->points[i].b = (uint8_t)255;
                    break;
                case 6:
                    mCloud->points[i].r = (uint8_t)128;
                    mCloud->points[i].g = (uint8_t)0;
                    mCloud->points[i].b = (uint8_t)0;
                    break;
                case 7:
                    mCloud->points[i].r = (uint8_t)128;
                    mCloud->points[i].g = (uint8_t)128;
                    mCloud->points[i].b = (uint8_t)0;
                    break;
                case 8:
                    mCloud->points[i].r = (uint8_t)0;
                    mCloud->points[i].g = (uint8_t)128;
                    mCloud->points[i].b = (uint8_t)0;
                    break;
                case 9:
                    mCloud->points[i].r = (uint8_t)0;
                    mCloud->points[i].g = (uint8_t)0;
                    mCloud->points[i].b = (uint8_t)128;
                    break;
                default:
                    std::fprintf(stderr, "Should never get here.\n");
                    assert(false);
            }

            if (i < lineNewPointList.size())
            {
                lineNewPointList[i].x = (*timestep_iter)[i].x - offset.x;
                lineNewPointList[i].y = (*timestep_iter)[i].y - offset.y;
                lineNewPointList[i].z = (*timestep_iter)[i].z - offset.z;
            }
        }

        mViewer->updatePointCloud( mCloud, "spt" );

        for (uint32_t i =0; i < lineOldPointList.size(); ++i)
        {
            if (lineOldPointList[i].x != check_val ||
                lineOldPointList[i].y != check_val ||
                lineOldPointList[i].z != check_val )
            {
                std::string line_name = "line" + std::to_string(i) + "_" + std::to_string(timestep);
                mMutex.lock();
                if (mStopCalled)
                {
                    mMutex.unlock();
                    break;
                }
                mViewer->addLine( lineOldPointList[i], lineNewPointList[i], line_name  );
                mMutex.unlock();
                QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
            }
            else
            {
               fprintf(stderr, "*********Skipping first point*************\n");
            }
            lineOldPointList[i] = lineNewPointList[i];
        }
        mpUi->qvtkwidget->update();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
    }

    mMutex.lock();
    mViewRunning = false;
    mStopCalled = false;
    if (iEndTimeStep < 0 || iEndTimeStep - iStartTimeStep > 1)
    {
        //don't update timestep spinbox if it was only a single timestep view update
        mpUi->spinBox_viewTimestep->setValue(timestep);
        mCurTimestep = timestep;
    }
    mMutex.unlock();
    UpdateStartButtonText( false );
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
}

void VisualizationDialog::UpdateStartButtonText( bool iPauseText )
{
    mMutex.lock();
    if (iPauseText)
    {
       mpUi->pushButton_view->setText(" Stop ");
    }
    else
    {
       mpUi->pushButton_view->setText(" Start ");
    }

    mMutex.unlock();
}

void VisualizationDialog::on_pushButton_viewClear_clicked()
{
    mMutex.lock();
    mViewer->removeAllShapes();
    mpUi->qvtkwidget->update();
    mMutex.unlock();
}

void VisualizationDialog::FreezeDialog( bool iFreeze )
{
    mpUi->spinBox_waitTime->setEnabled( !iFreeze );
    mpUi->spinBox_viewTimestep->setEnabled( !iFreeze );
    mpUi->fileSelectionBox->setEnabled( !iFreeze );
    if (mpUi->checkBox_triPointFileNew->isChecked())
    {
        mpUi->lineEdit_triPointFileNew->setEnabled( !iFreeze );
        mpUi->pushButton_triPointFileNew->setEnabled( !iFreeze);
    }
    //mpUi->checkBox_smooth->setEnabled( !iFreeze );

//    if (mpUi->checkBox_smooth->isChecked())
//    {
//        mpUi->doubleSpinBox_pointNoise->setEnabled( !iFreeze );
//        mpUi->doubleSpinBox_updateMultiplier->setEnabled( !iFreeze );
//    }
    mpUi->checkBox_trace->setEnabled( !iFreeze );

    if (mpUi->checkBox_trace->isChecked())
    {
        mpUi->spinBox_numTrajPoints->setEnabled( !iFreeze );
        mpUi->spinBox_trajTailLength->setEnabled( !iFreeze );
    }
    mpUi->checkBox_renderOffset->setEnabled( !iFreeze );

    if (mpUi->checkBox_renderOffset->isChecked())
    {
        mpUi->checkBox_centerX->setEnabled( !iFreeze );
        mpUi->checkBox_centerY->setEnabled( !iFreeze );
        mpUi->checkBox_centerZ->setEnabled( !iFreeze );
    }
}

void VisualizationDialog::on_pushButton_view_clicked()
{
    //stop any view already running
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);

    mMutex.lock();
    if (mViewRunning)
    {
       mpUi->pushButton_view->setText(" Start ");
       mStopCalled = true;
       mMutex.unlock();
       FreezeDialog( false );
       return;
    }
    else
    {
        mMutex.unlock();
        FreezeDialog( true );
    }

    mpUi->pushButton_view->setText(" Stop ");
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
    //Do a check of required fields

    //Read in points from file
    if (mTriangulatedPointsCV.size() == 0 ||
        mTriangulatedPointsPCL.size() != mTriangulatedPointsCV.size())
    {
        //processXML didn't read any points
        QMessageBox msg_box;
        msg_box.setText( "There are no triangulated points to show. There must be a problem with the given XML file.");
        msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
        msg_box.setWindowTitle("Error!");
        msg_box.exec();
        FreezeDialog( false );

        return;
    }

    //Check if smoothing selected
      //if smoothing then process points in file using MultiSmoother3d

    //set up PCL code for visualization
    int num_trace_points = 0;
    int length_trace_history = 0;
    if (mpUi->checkBox_trace->isChecked())
    {
        num_trace_points = mpUi->spinBox_numTrajPoints->value();
        length_trace_history = mpUi->spinBox_trajTailLength->value();
    }

    if (mpUi->checkBox_renderOffset->isChecked())
    {
        VisualizePoints( mTriangulatedPointsPCL,
                         num_trace_points,
                         length_trace_history,
                         mpUi->checkBox_centerX->isChecked(),
                         mpUi->checkBox_centerY->isChecked(),
                         mpUi->checkBox_centerZ->isChecked(),
                         mCurTimestep);
    }
    else
    {
        VisualizePoints( mTriangulatedPointsPCL,
                         num_trace_points,
                         length_trace_history,
                         false,
                         false,
                         false,
                         mCurTimestep);
    }
    FreezeDialog( false );
}

void VisualizationDialog::closeEvent(QCloseEvent* iEvent)
{
    iEvent->ignore();
    hide();
    //mVisualHullInterface->setWindowViewable( false );
}

void VisualizationDialog::on_pushButton_cancel_clicked()
{
    hide();
    //mVisualHullInterface->setWindowViewable( false );
}

//void VisualizationDialog::ReadInPolyHull( QString iDirectory,
//                                          uint32_t iTotalNumTimesteps,
//                                          std::vector< vtkSmartPointer<vtkPolyData> >& oPolyData )
//{
//    QDirIterator dir_iter( iDirectory, QDirIterator::Subdirectories );
//    while (dir_iter.hasNext())
//    {
//        dir_iter.next();
//        if (QFileInfo(dir_iter.filePath()).isFile())
//        {
//            if (QFileInfo(dir_iter.filePath()).suffix() == "vtp")
//            {
//                if (oPolyData.size() != iTotalNumTimesteps)
//                {
//                    oPolyData.resize( iTotalNumTimesteps );
//                }
//                QString filename = dir_iter.fileName();
//                filename.replace( ".vtp", "" );

//                oPolyData[ filename.toInt() ] = mVisualHullInterface->readSavedPolygonsVTK( dir_iter.filePath().toStdString() ) ;
//            }
//        }
//    }
//}

bool VisualizationDialog::ProcessCSVFile( QString iCsvFile )
{
    QFile file( iCsvFile );
    if (!file.open(QIODevice::ReadOnly))
    {
        std::fprintf(stderr, "%s\n", file.errorString().toStdString().c_str() );
        return false;
    }
    mMutex.lock();
    cv::Point3d mean(0,0,0);
    int num_all_points = 0;
    mTriangulatedPointsCV.clear();
    mTriangulatedPointsPCL.clear();
    int num_timesteps = 0;
    int smallest_timestep = INT_MAX;
    int largest_timestep = -1;

    while (!file.atEnd())
    {
        QByteArray line = file.readLine();
        QStringList word_list = QString::fromUtf8(line.data()).split(',');
        if (word_list.size() == 0)
        {
            continue;
        }
        else if (word_list[word_list.size() -1] == "\n")
        {
            word_list.pop_back();
        }
        bool is_int;
        int value = word_list[0].toInt( &is_int );
        if (is_int)
        {
            if (mTriangulatedPointsCV.size() == 0)
            {
                if (mMaxTimesteps > 0)
                {
                    mTriangulatedPointsCV.resize( mMaxTimesteps );
                    mTriangulatedPointsPCL.resize( mMaxTimesteps );
                }
            }
            if (value > largest_timestep)
            {
                largest_timestep = value;
            }
            if (value < smallest_timestep)
            {
                smallest_timestep = value;
            }
            //add timestep to list of timesteps
            std::vector <cv::Point3d> point_list;
            for (int i = 1; i < word_list.size(); ++i)
            {
                QStringList point = word_list[i].split('|');
                double x = point[0].toDouble();
                double y = point[1].toDouble();
                double z = point[2].toDouble();

                point_list.push_back( cv::Point3d(x,y,z));
            }

            mTriangulatedPointsCV[value] = point_list;
            std::vector< pcl::PointXYZRGB > pcl_point_list;
            for (uint32_t p = 0; p < point_list.size(); ++p, ++num_all_points)
            {
                mean.x += point_list[p].x;
                mean.y += point_list[p].y;
                mean.z += point_list[p].z;
                pcl::PointXYZRGB point( 0, 0, 0);
                point.x = point_list[p].x;
                point.y = point_list[p].y;
                point.z = point_list[p].z;
                pcl_point_list.push_back( point );
            }

            mTriangulatedPointsPCL[value] = pcl_point_list;
        }
        else if (word_list[0] == "ModifiedNumTimesteps")
        {
            num_timesteps = word_list[1].toInt();
        }
        else if (word_list[0] == "MaxNumPoints" && word_list.size() == 2)
        {
            mMaxNumPoints = word_list[1].toInt();
        }
        else if (word_list[0] == "MaxTimestep" && word_list.size() == 2)
        {
            mMaxTimesteps = word_list[1].toInt();
            mTriangulatedPointsCV.resize( mMaxTimesteps );
            mTriangulatedPointsPCL.resize( mMaxTimesteps );
        }
        else if (word_list[0] == "NumMissingTimesteps" && word_list.size() == 2)
        {
            mNumMissingTimesteps = word_list[1].toInt();
        }
    }

    if (num_timesteps == 0)
    {
        num_timesteps = largest_timestep - smallest_timestep + 1;
    }

    mOffsetX = mean.x / num_all_points;
    mOffsetY = mean.y / num_all_points;
    mOffsetZ = mean.z  /num_all_points;

    mpUi->spinBox_viewTimestep->setMaximum( largest_timestep );
    mpUi->spinBox_viewTimestep->setMinimum( smallest_timestep );
    mpUi->spinBox_trajTailLength->setMaximum( num_timesteps - 2);
    mpUi->spinBox_numTrajPoints->setMaximum( mMaxNumPoints );
    //update timestep ranges of the viewer to fit the correct timesteps

    mMutex.unlock();

    return true;
}

bool VisualizationDialog::ProcessXMLFile( QString iXmlFile )
{
    cv::FileStorage f(iXmlFile.toStdString().c_str(), cv::FileStorage::READ);
    cv::FileNode fs = f["SimplePointTracking"];

    mMutex.lock();
    int tmp_max_num_points = 0;
    fs["MaxNumPoints"] >> tmp_max_num_points;
    if (0 == tmp_max_num_points)
    {
        std::cerr << "MaxNumPoints not found." << std::endl;
    }
    else
    {
        assert( tmp_max_num_points > 0);
        mMaxNumPoints = static_cast<uint32_t>(tmp_max_num_points);
    }

    fs["MaxTimestep"] >> mMaxTimesteps;
    if (0 == mMaxTimesteps)
    {
        std::cerr << "MaxTimesteps not found." << std::endl;
    }

    fs["NumMissingTimesteps"] >> mNumMissingTimesteps;
    if (0 == mNumMissingTimesteps)
    {
        std::cerr << "NumMissingTimesteps not found." << std::endl;
    }
    mMutex.unlock();

    cv::FileNode smoothing = fs["Smoothing"];
    if (NULL != smoothing.node)
    {
        mMutex.lock();
        (smoothing["PointNoise"]) >> mPointNoise;
        (smoothing["UpdateMultiplier"]) >> mUpdateMultiplier;

        mpUi->checkBox_smooth->setChecked( true );
        if (0 != mPointNoise)
        {
            mpUi->doubleSpinBox_pointNoise->setValue( mPointNoise );
            mpUi->doubleSpinBox_pointNoise->setEnabled( true );
            mpUi->label_pointNoise->setEnabled( true );
        }
        if (0 != mUpdateMultiplier)
        {
            mpUi->doubleSpinBox_updateMultiplier->setValue( mUpdateMultiplier );
            mpUi->doubleSpinBox_updateMultiplier->setEnabled( true );
            mpUi->label_updateMultiplier->setEnabled( true );
        }
        mMutex.unlock();
    }
    else
    {
        std::cerr << "No previous smoothing info found." << std::endl;
        mMutex.lock();
        mpUi->checkBox_smooth->setChecked( false );
        mpUi->doubleSpinBox_pointNoise->setEnabled( false );
        mpUi->label_pointNoise->setEnabled( false );
        mpUi->doubleSpinBox_updateMultiplier->setEnabled( false );
        mpUi->label_updateMultiplier->setEnabled( false );
        mMutex.unlock();
    }

    cv::FileNode offset = fs["RenderOffset"];
    //This check commented out because this hasn't yet been implemented.
//    if (NULL != offset.node)
//    {
        mMutex.lock();
        (offset["OffsetX"]) >> mOffsetX;
        (offset["OffsetY"]) >> mOffsetY;
        (offset["OffsetZ"]) >> mOffsetZ;

        if (mOffsetX > 0 || mOffsetY > 0 || mOffsetZ > 0)
        {
            mpUi->checkBox_renderOffset->setChecked( true );
            mpUi->checkBox_centerX->setEnabled( true );
            mpUi->checkBox_centerY->setEnabled( true );
            mpUi->checkBox_centerZ->setEnabled( true );
        }

        if (mOffsetX > 0)
        {
            mpUi->checkBox_centerX->setChecked( true );
        }

        if (mOffsetY > 0)
        {
            mpUi->checkBox_centerY->setChecked( true );
        }

        if (mOffsetZ > 0)
        {
            mpUi->checkBox_centerZ->setChecked( true );
        }
        mMutex.unlock();

//    }
//    else
//    {
//        std::cerr << "No previous render offset info found." << std::endl;
//        mMutex.lock();
//        mpUi->checkBox_renderOffset->setChecked( false );
//        mpUi->checkBox_centerX->setEnabled( false );
//        mpUi->checkBox_centerY->setEnabled( false );
//        mpUi->checkBox_centerZ->setEnabled( false );
//        mMutex.unlock();
//    }

    mMutex.lock();
    cv::Point3d mean(0,0,0);
    int num_all_points = 0;
    std::string tag_name = "Timestep";
    mTriangulatedPointsCV.clear();
    mTriangulatedPointsPCL.clear();
    for (int i = 0; i < mMaxTimesteps; ++i)
    {
        std::string step_name = tag_name + std::to_string(i);
        cv::FileNode offset = fs[step_name];

        std::vector <cv::Point3d> point_list;
        offset >> point_list;
        mTriangulatedPointsCV.push_back( point_list );
        std::vector< pcl::PointXYZRGB > pcl_point_list;
        for (uint32_t p = 0; p < point_list.size(); ++p, ++num_all_points)
        {
            mean.x += point_list[p].x;
            mean.y += point_list[p].y;
            mean.z += point_list[p].z;
            pcl::PointXYZRGB point( 0, 0, 0);
            point.x = point_list[p].x;
            point.y = point_list[p].y;
            point.z = point_list[p].z;
            pcl_point_list.push_back( point );
        }
        mTriangulatedPointsPCL.push_back( pcl_point_list );
    }
    mOffsetX = mean.x / num_all_points;
    mOffsetY = mean.y / num_all_points;
    mOffsetZ = mean.z  /num_all_points;

    mpUi->spinBox_viewTimestep->setMaximum( mMaxTimesteps - 1 );
    mpUi->spinBox_trajTailLength->setMaximum( mMaxTimesteps - 2);
    mpUi->spinBox_numTrajPoints->setMaximum( mMaxNumPoints );
    mMutex.unlock();

    f.release();
    return true;
}

void VisualizationDialog::on_fileSelectionBox_currentTextChanged(const QString& iNewFile)
{

    if (iNewFile.size() > 0)
    {
        QString file_name = mpProjDetails->ProjectDir() +
                            QDir::separator() +
                            mpProjDetails->TriangDir() +
                            QDir::separator() +
                            iNewFile;

        QFileInfo fi( file_name );
        QString suffix = fi.completeSuffix();
        if (suffix == "xml")
        {
            if (!ProcessXMLFile( file_name ))
            {
                QMessageBox msg_box;
                msg_box.setText( "The file you selected is not valid Triangulated Points XML file.");
                msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
                msg_box.setWindowTitle("Error!");
                msg_box.exec();
                mMutex.lock();
                mpUi->fileSelectionBox->setCurrentText("");
                mMutex.unlock();
            }
        }
        else
        {
            if (!ProcessCSVFile( file_name ))
            {
                QMessageBox msg_box;
                msg_box.setText( "The file you selected is not valid Triangulated Points CSV file.");
                msg_box.setWindowIcon(QIcon("snake_icon_inv.jpg"));
                msg_box.setWindowTitle("Error!");
                msg_box.exec();
                mMutex.lock();
                mpUi->fileSelectionBox->setCurrentText("");
                mMutex.unlock();
            }
        }
//        if (mpUi->checkBox_showVisualHull->isChecked())
//        {
//            int32_t timestep_max = mpUi->spinBox_viewTimestep->maximum();
//            assert(timestep_max > 0);
//            ReadInPolyHull( fi.path(), static_cast<uint32_t>(timestep_max+1), mPolyDataVec );

//            mHullviewer->addCoordinateSystem( 50.0,
//                                              static_cast<float>(mOffsetX),
//                                              static_cast<float>(mOffsetY),
//                                              static_cast<float>(mOffsetZ),
//                                              "axis",
//                                              0 );
//            mHullviewer->resetCamera();
//            mHullviewer->resetCameraViewpoint();

//        }
    }
}

void VisualizationDialog::on_pushButton_triPointFileNew_clicked()
{

}

void VisualizationDialog::on_pushButton_resetView_clicked()
{
    mMutex.lock();
    mViewer->resetCamera();
    mViewer->resetCameraViewpoint();
    mpUi->qvtkwidget->update();
    mMutex.unlock();
}


void VisualizationDialog::on_doubleSpinBox_pointNoise_editingFinished()
{
    mMutex.lock();
    mPointNoise = mpUi->doubleSpinBox_pointNoise->value();
    mMutex.unlock();
}

void VisualizationDialog::on_doubleSpinBox_updateMultiplier_editingFinished()
{
    mMutex.lock();
    mUpdateMultiplier = mpUi->doubleSpinBox_updateMultiplier->value();
    mMutex.unlock();
}

void VisualizationDialog::on_spinBox_numTrajPoints_editingFinished()
{
    mMutex.lock();
    mViewer->removeAllShapes();
    mpUi->qvtkwidget->update();
    mMutex.unlock();
}

void VisualizationDialog::on_spinBox_trajTailLength_editingFinished()
{
    mMutex.lock();
    mViewer->removeAllShapes();
    mpUi->qvtkwidget->update();
    mMutex.unlock();
}

void VisualizationDialog::on_spinBox_viewTimestep_editingFinished()
{
    if (!mViewRunning)
    {
        mMutex.lock();
        mCurTimestep = mpUi->spinBox_viewTimestep->value();
        mMutex.unlock();

        int num_trace_points = 0;
        int length_trace_history = 0;
        mMutex.lock();
        if (mpUi->checkBox_trace->isChecked())
        {
            num_trace_points = mpUi->spinBox_numTrajPoints->value();
            length_trace_history = mpUi->spinBox_trajTailLength->value();
            if (0 == length_trace_history)
            {
                length_trace_history = mMaxTimesteps - 1; // -1 for zero based counting
            }
        }
        mMutex.unlock();
        if (mpUi->checkBox_renderOffset->isChecked())
        {
            VisualizePoints( mTriangulatedPointsPCL,
                             num_trace_points,
                             length_trace_history,
                             mpUi->checkBox_centerX->isChecked(),
                             mpUi->checkBox_centerY->isChecked(),
                             mpUi->checkBox_centerZ->isChecked(),
                             mCurTimestep,
                             mCurTimestep+1);
        }
        else
        {
            VisualizePoints( mTriangulatedPointsPCL,
                             num_trace_points,
                             length_trace_history,
                             false,
                             false,
                             false,
                             mCurTimestep,
                             mCurTimestep+1);
        }
    }
}

void VisualizationDialog::on_checkBox_smooth_toggled()
{
    if (mpUi->checkBox_smooth->isChecked())
    {
        mpUi->doubleSpinBox_pointNoise->setEnabled( true );
        mpUi->doubleSpinBox_updateMultiplier->setEnabled( true );
        mpUi->label_pointNoise->setEnabled( true );
        mpUi->label_updateMultiplier->setEnabled( true );

        mpUi->checkBox_triPointFileNew->setEnabled( true );
        if (mpUi->checkBox_triPointFileNew->isChecked())
        {
            mpUi->label_triPointFileNew->setEnabled( true );
            mpUi->lineEdit_triPointFileNew->setEnabled( true );
            mpUi->pushButton_triPointFileNew->setEnabled( true );
        }
        else
        {
            mpUi->label_triPointFileNew->setEnabled( false );
            mpUi->lineEdit_triPointFileNew->setEnabled( false );
            mpUi->pushButton_triPointFileNew->setEnabled( false );
        }
    }
    else
    {
        mpUi->doubleSpinBox_pointNoise->setEnabled( false );
        mpUi->doubleSpinBox_updateMultiplier->setEnabled( false );
        mpUi->label_pointNoise->setEnabled( false );
        mpUi->label_updateMultiplier->setEnabled( false );

        mpUi->checkBox_triPointFileNew->setEnabled( false );
        mpUi->label_triPointFileNew->setEnabled( false );
        mpUi->lineEdit_triPointFileNew->setEnabled( false );
        mpUi->pushButton_triPointFileNew->setEnabled( false );
    }
}

void VisualizationDialog::on_checkBox_trace_toggled()
{
    if (mpUi->checkBox_trace->isChecked())
    {
        mpUi->spinBox_numTrajPoints->setEnabled( true );
        mpUi->spinBox_trajTailLength->setEnabled( true );
        mpUi->label_trajTailLength->setEnabled( true );
        mpUi->label_numTrajPoints->setEnabled( true );
        mpUi->label_trajTailLengthNote->setEnabled( true );
        mpUi->spinBox_waitTime->setEnabled( false );
    }
    else
    {
        mpUi->spinBox_numTrajPoints->setEnabled( false );
        mpUi->spinBox_trajTailLength->setEnabled( false );
        mpUi->label_trajTailLength->setEnabled( false );
        mpUi->label_numTrajPoints->setEnabled( false );
        mpUi->label_trajTailLengthNote->setEnabled( false );
        mpUi->spinBox_waitTime->setEnabled( true );
    }
}

void VisualizationDialog::on_checkBox_renderOffset_toggled()
{
    if (mpUi->checkBox_renderOffset->isChecked())
    {
        mpUi->checkBox_centerX->setEnabled( true );
        mpUi->checkBox_centerY->setEnabled( true );
        mpUi->checkBox_centerZ->setEnabled( true );
    }
    else
    {
        mpUi->checkBox_centerX->setEnabled( false );
        mpUi->checkBox_centerY->setEnabled( false );
        mpUi->checkBox_centerZ->setEnabled( false );
    }
}

void VisualizationDialog::on_checkBox_triPointFileNew_toggled()
{
    if (mpUi->checkBox_triPointFileNew->isChecked())
    {
        mpUi->lineEdit_triPointFileNew->setEnabled( true );
        mpUi->pushButton_triPointFileNew->setEnabled( true );
        mpUi->label_triPointFileNew->setEnabled( true );
    }
    else
    {
        mpUi->lineEdit_triPointFileNew->setEnabled( false );
        mpUi->pushButton_triPointFileNew->setEnabled( false );
        mpUi->label_triPointFileNew->setEnabled( false );
    }
}

void VisualizationDialog::on_checkBox_centerX_toggled()
{

}

void VisualizationDialog::on_checkBox_centerY_toggled()
{

}

void VisualizationDialog::on_checkBox_centerZ_toggled()
{

}

