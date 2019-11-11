#include <QColor>
#include <QEventLoop>
#include <QMutexLocker>
#include <set>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <malloc.h>


#include "image/ImageSet.h"
#include "image/PixelSet.h"
#include "TrackingDialog.h"

#include "CorrespondenceDialog.h"
#include "CorrespondenceImage.h"
#include "ui_CorrespondenceDialog.h"


/* Generic function to find if an element of any type exists in list
 */
template <typename T>
bool contains(std::list<T> & listOfElements, const T & element)
{
    // Find the iterator if element in list
    auto it = std::find(listOfElements.begin(), listOfElements.end(), element);
    //return if iterator points to end or not. It points to end then it means element
    // does not exists in list
    return it != listOfElements.end();
}

template <typename T>
bool contains(std::set<T> & listOfElements, const T & element)
{
    // Find the iterator if element in list
    auto it = std::find(listOfElements.begin(), listOfElements.end(), element);
    //return if iterator points to end or not. It points to end then it means element
    // does not exists in list
    return it != listOfElements.end();
}


CorrespondenceDialog::CorrespondenceDialog(QWidget* iParent,
                                            std::vector< SmtImage > iTimestepImages,
                                           const int iTimestepValue,
                                            std::vector< std::vector< SmtPixel > > iTimestepMarkersFound,
                                            std::pair<unsigned long, std::set<unsigned long> > iCamsToExclude,
                                           const int iNumMarkersToFind)
    :
    QDialog(iParent),
    mpParent(iParent),
    mpUi(new Ui::CorrespondenceDialog),
    mIsCancelled(false),
    mClickCoords(-1, -1, -1),
    mTimestep(iTimestepValue),
    mCurCamIdx(-1),
    mCurPointNum(-1),
    mMaxPointNum(iNumMarkersToFind),
    mIsComplete(false),
    mIsUpdating(false)
{
    mpUi->setupUi(this);
    setModal(true);
    mpUi->undoClickButton->setEnabled(false);

    connect( mpUi->Image, SIGNAL(SignalClick(int,int)), this, SLOT(SavePixelValueAndUpdateImage( int, int )), Qt::QueuedConnection);
    connect( this, SIGNAL(UpdateImage(QPixmap,int,int)), mpUi->Image, SLOT(UpdateImage(QPixmap,int,int)), Qt::QueuedConnection);
    //this->setWindowFlags(Qt::WindowStaysOnTopHint|Qt::X11BypassWindowManagerHint);
    //this->setWindowFlags(Qt::WindowStaysOnTopHint);

    mFoundMarkers.resize( iTimestepMarkersFound.size() );
    for (uint32_t j = 0; j < iTimestepMarkersFound.size(); ++j)
    {
        for (uint32_t i = 0; i < iTimestepMarkersFound[j].size(); ++i)
        {
            mFoundMarkers[j].push_back( iTimestepMarkersFound[j][i]);
        }
    }

    for (uint32_t i = 0; i < iTimestepImages.size(); ++i)
    {
        mTimestepImages.push_back( iTimestepImages[i]);
    }

    //populate the camera list with the camera numbers we can use.
    for ( uint32_t i = 0; i < iCamsToExclude.first; ++i)
    {
        if (0 == iCamsToExclude.second.count( i ))
        {
            mCamList.push_back( i );
        }
    }

    //We populate this list with the indexes of the points we have. We then know later when
    //trying to calculate closest point, which points have already been used.
    mPointIdxLeft.resize( iCamsToExclude.first);
    for (unsigned long c = 0; c < iCamsToExclude.first; ++c)
    {
        if (0 == iCamsToExclude.second.count( c ))
        {
            std::list<unsigned long> tmp_list;
            for (unsigned long t = 0; t < mMaxPointNum; ++t)
            {
                tmp_list.push_back( t );
            }
            mPointIdxLeft[ c ] = tmp_list;
        }
    }

    //Populate the user marked points list with dummy points that will later be changed.
    mMarkedPoints.resize( iCamsToExclude.first );
    for (uint32_t i = 0; i< iCamsToExclude.first; ++i)
    {
        if (0 == iCamsToExclude.second.count( i ))
        {
            mMarkedPoints[i].resize( mMaxPointNum );
        }
    }

    //holds the original point indexes in the order they were clicked by the user.
    for (uint32_t t = 0; t < mMaxPointNum; ++t)
    {
        std::vector< int32_t > point_idx_list;
        point_idx_list.resize( iCamsToExclude.first );
        mPointIdxList.push_back( point_idx_list );
    }

//    connect( this,
//             SIGNAL(SignalGrabbedPixelValue(int,int)),
//             this,
//             SLOT(SavePixelValueAndUpdateImage(int,int)) );


    SavePixelValueAndUpdateImage( -1, -1 );
}

CorrespondenceDialog::~CorrespondenceDialog()
{
    std::cerr << "Deleting CorrespondenceDialog..." << std::endl;
    {
        std::vector< SmtImage > delete_vector;
        delete_vector.swap(mTimestepImages);
    }
    {
        std::vector< std::vector< SmtPixel> > delete_vector;
        delete_vector.swap(mFoundMarkers);
    }
    delete mpUi;
    //::INFO:: we have to ask politely for the C memory allocator to return memory to OS
    //otherwise it looks like we have a memory leak, when we don't.
    malloc_trim(0);
}

//void CorrespondenceDialog::SignalGrabbedPixelValue( int iX, int iY )
//{
//    emit GrabbedPixelValue(iX, iY);
//}
void CorrespondenceDialog::closeEvent(QCloseEvent *event)
{
    emit Cancelled();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
    event->accept();
}

cv::Scalar GetRGBInfo( int iIdx )
{
    switch( iIdx % 10 )
    {
        case 0:
            return cv::Scalar( unsigned(255), unsigned(0), unsigned(0) );
        case 1:
            return cv::Scalar( unsigned(0), unsigned(255), unsigned(0) );
        case 2:
            return cv::Scalar( unsigned(0), unsigned(0), unsigned(255) );
        case 3:
            return cv::Scalar( unsigned(255), unsigned(0), unsigned(255) );
        case 4:
            return cv::Scalar( unsigned(255), unsigned(255), unsigned(0) );
        case 5:
            return cv::Scalar( unsigned(0), unsigned(255), unsigned(255) );
        case 6:
            return cv::Scalar( unsigned(128), unsigned(0), unsigned(0) );
        case 7:
            return cv::Scalar( unsigned(128), unsigned(128), unsigned(0) );
        case 8:
            return cv::Scalar( unsigned(0), unsigned(128), unsigned(0) );
        case 9:
            return cv::Scalar( unsigned(0), unsigned(0), unsigned(128) );
        default:
            std::fprintf( stderr, "Should never get here.\n");
            assert( false );
    }
    return cv::Scalar( 0, 0, 0 );
}

const std::vector< std::vector< std::vector< int32_t > > > CorrespondenceDialog::BestFitInfoForPoints()
{
    std::vector< std::vector< std::vector< int32_t > > > tmpVec;
    tmpVec.push_back(mPointIdxList);
    return tmpVec;
}

void CorrespondenceDialog::SavePixelValueAndUpdateImage( int iX, int iY)
{
    if (mIsUpdating)
    {
        std::cerr << "Skipping SavePixelValueAndUpdateImage as it is already currently processing a click." << std::endl;
        return;
    }
    std::cerr << "In SavePixelValueAndUpdateImage.." << std::endl;
    //mLock.lock();
    mIsUpdating = true;
    //mLock.unlock();
    if ( -1 == iX && -1 == iY && -1 == mCurCamIdx && -1 == mCurPointNum )
    {
        //Initializing the window
        mCurCamIdx = 0;
        mCurPointNum = 1; //don't use zero based counting for the points
        mpUi->TimestepLabel->setText("Timestep: " + QString::number(mTimestep));
    }
    else if ( -2 == iX && -2 == iY)
    { // user wants to undo a click made

            if (mCurCamIdx > 0)
            {
                --mCurCamIdx;
            }
            else
            {
                if (mCurPointNum > 1)
                {
                    --mCurPointNum;
                    mCurCamIdx = mCamList.size() - 1;
                }
            }
            mPointIdxLeft[mCamList[mCurCamIdx]].push_back( mPointIdxList[mCurPointNum-1][mCamList[mCurCamIdx]] );

            if (1 == mCurPointNum && 0 == mCurCamIdx)
            {
                mpUi->undoClickButton->setEnabled(false);
            }
    }
    else
    {
        //User clicked point on the window, save coords and get closest found marked point
        mpUi->undoClickButton->setEnabled(true);
        mClickCoords.x = iX;
        mClickCoords.y = iY;

        ///@todo :TODO:
        //This method assumes the user can somewhat accurately click on the points to be tracked.
        //Furthermore, the user can't currently click on places where no point was found, or use one
        //point more than once in the case of merged points.
        double best_dist = std::numeric_limits<double>::max();
        int8_t index = std::numeric_limits<int8_t>::max();
        for (uint32_t px = 0; px < mPointIdxLeft[mCamList[mCurCamIdx]].size(); ++px)
        {
            std::list< unsigned long >::iterator tmp_iter = mPointIdxLeft[mCamList[mCurCamIdx]].begin();
            std::advance( tmp_iter, px );
            int8_t idx = *tmp_iter;
            double dist = sqrt( pow( mFoundMarkers[mCamList[mCurCamIdx]][ idx ].x - mClickCoords.x, 2) +
                                pow( mFoundMarkers[mCamList[mCurCamIdx]][ idx ].y - mClickCoords.y, 2 ) );
            if (dist < best_dist)
            {
                best_dist = dist;
                index =  *tmp_iter;
            }
        }
        mMarkedPoints[mCamList[mCurCamIdx]][mCurPointNum-1].x = mFoundMarkers[mCamList[mCurCamIdx]][index].x;
        mMarkedPoints[mCamList[mCurCamIdx]][mCurPointNum-1].y = mFoundMarkers[mCamList[mCurCamIdx]][index].y;
        mPointIdxLeft[mCamList[mCurCamIdx]].remove( index );
        mPointIdxList[mCurPointNum-1][mCamList[mCurCamIdx]] = index ;


        // forward to the next image
        if (mCurCamIdx < mCamList.size() -1)
        {
            mCurCamIdx++;
        }
        else if (mCurCamIdx == (mCamList.size() -1) && mCurPointNum < mMaxPointNum)
        {
            mCurCamIdx = 0;
            mCurPointNum++;
        }
        else
        {
            assert( mCurCamIdx < mCamList.size() );
            assert( mCurPointNum == mMaxPointNum );
            mIsComplete = true;
            emit ProcessingDone();
            QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
            hide();
            return;
        }
    }
    mpUi->CamLabel->setText("Cam: " + QString::number(mCamList[mCurCamIdx]));
    mpUi->PointLabel->setText("Point: " + QString::number(mCurPointNum));

    cv::Mat cur_image = mTimestepImages[mCurCamIdx].clone();

    for (uint32_t pix = 0; pix < mFoundMarkers[mCamList[mCurCamIdx]].size(); ++pix)
    {
        cv::drawMarker( cur_image,
                        mFoundMarkers[mCamList[mCurCamIdx]][pix],
                        cv::Scalar( 255, 105, 180 ),
                        cv::MARKER_TRIANGLE_DOWN,
                        8/*size*/,
                        2 /*thickness*/ );
    }

    for (uint32_t mark = 0; mark < mCurPointNum-1; ++mark)
    {
        cv::drawMarker( cur_image,
                mMarkedPoints[mCamList[mCurCamIdx]][mark],
                GetRGBInfo( mark ),
                cv::MARKER_STAR,
                6/*size*/,
                3/*thickness*/ );
    }

    QPixmap pix= QPixmap::fromImage(QImage((unsigned char*) cur_image.data,
                                           cur_image.cols,
                                           cur_image.rows,
                                           QImage::Format_RGB888));
    emit UpdateImage( pix, cur_image.cols, cur_image.rows);
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
    std::cerr << "Finished current SavePixelValueAndUpdaeImage Processing, back in ready state." << std::endl;
    //mLock.lock();
    mIsUpdating = false;
    //mLock.unlock();
}



void CorrespondenceDialog::on_undoClickButton_clicked()
{
    SavePixelValueAndUpdateImage( -2, -2 );
}

void CorrespondenceDialog::on_CancelButton_clicked()
{
    emit Cancelled();
    hide();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);


//    if (typeid(TrackingDialog*) == typeid(mpParent))
//    {
//        TrackingDialog* pParentDiag = (TrackingDialog *)&mpParent;
//        pParentDiag->on_cancelTrackingButton_clicked();
//    }
}
