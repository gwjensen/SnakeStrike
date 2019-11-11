#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>
#include <vector>
#include <algorithm>

#include "opensource/munkres/Munkres.h"
#include "opensource/eigen_cv_conversions.h"
#include "visualization/opencv_viz.h"

#include "MultiPointTracker.h"

MultiPointTracker::MultiPointTracker( uint8_t iNumPoints )
    :mNumPoints( iNumPoints )
{

}
MultiPointTracker::~MultiPointTracker()
{
    for (uint32_t i = 0; i < mKFilterPtrs.size(); ++i)
    {
        delete mKFilterPtrs[i];
    }
}


void MultiPointTracker::Initialize( std::vector< SmtPixel > iStartingPoints )
{


    //initalize KalmanFilters with these points
    int state_size = 4;
    int meas_size = 2;
    int contr_size = 0;
    unsigned int type = CV_64F;

    for (uint32_t i = 0; i < iStartingPoints.size(); ++i)
    {
        cv::KalmanFilter* kf = new cv::KalmanFilter( state_size, meas_size, contr_size, type );

        cv::Mat state( state_size, 1, type );  // [x,y,v_x,v_y]
        cv::Mat meas( meas_size, 1, type );    // [z_x,z_y]
        //cv::Mat procNoise(state_size, 1, type)
        // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

        // Transition State Matrix A
        // Note: set dT at each processing step!
        // [ 1 0 dT 0  ]
        // [ 0 1 0  dT ]
        // [ 0 0 1  0  ]
        // [ 0 0 0  1  ]
        cv::setIdentity( kf->transitionMatrix );

        // Measure Matrix H
        // [ 1 0 0 0 ]
        // [ 0 1 0 0 ]
        kf->measurementMatrix = cv::Mat::zeros( meas_size, state_size, type );
        kf->measurementMatrix.at<double>(0) = 1.0;
        kf->measurementMatrix.at<double>(5) = 1.0;


        // Process Noise Covariance Matrix Q
        // [ Ex   0   0     0     ]
        // [ 0    Ey  0     0     ]
        // [ 0    0   Ev_x  0     ]
        // [ 0    0   0     Ev_y  ]
        cv::setIdentity( kf->processNoiseCov, cv::Scalar::all( 1e-2) );
        

        kf->processNoiseCov.at<float>(0) = 0;//1e-2;
        kf->processNoiseCov.at<float>(5) = 0;//1e-2;
        //kf->processNoiseCov.at<float>(10) = 1e-1;//2.0f;
        //kf->processNoiseCov.at<float>(15) = 1e-1;

        // Measures Noise Covariance Matrix R
        cv::setIdentity( kf->measurementNoiseCov, cv::Scalar::all( 1e-2 ) );

        //Update model with first point data
        cv::setIdentity( kf->errorCovPost, cv::Scalar::all( 1e-2) );
        //cv::setIdentity(kf->errorCovPre, cv::Scalar::all(.1));
        //kf->errorCovPost.at<float>(0) = 1e-2; // px
        //kf->errorCovPost.at<float>(5) = 1e-2; // px
        //kf->errorCovPost.at<float>(10) = 1e-2;
        //kf->errorCovPost.at<float>(15) = 1e-2;

        //not sure if these errorCov matrices need to be pre-populated
        //kf->errorCovPre.at<float>(0) = 1e-2; // px
        //kf->errorCovPre.at<float>(5) = 1e-2; // px
        //kf->errorCovPre.at<float>(10) = 1e-2;
        //kf->errorCovPre.at<float>(15) = 1e-2;


        state.at<double>(0) = iStartingPoints[i].x;
        state.at<double>(1) = iStartingPoints[i].y;
        state.at<double>(2) = 1e-4;//.1f;
        state.at<double>(3) = 1e-4;//.1f;

        kf->statePost = state;
        //kf->statePre = state;

        mKFilterPtrs.push_back( kf );
        mKFilterTimestep.push_back( 0 );
    }
}

bool MultiPointTracker::Next( const std::vector< SmtPixel >& iUnorderedPoints, uint32_t iTimestep, std::vector< SmtPixel >& oCorresPoints )
{
    if (iUnorderedPoints.empty())
    {
        return false;
    }
    cv::Mat_<int> cost_mat = GetCostMat( iUnorderedPoints, iTimestep );
    //printMat( cost_mat );

    if (cost_mat.rows == mNumPoints)
    {

        std::vector< std::pair<int, int> > ordered_idxs = OrderCorrespondingPoints( cost_mat );

        std::vector< int > ordered_idx_index;
        ordered_idx_index.resize( ordered_idxs.size() );
        for (uint32_t i = 0; i < ordered_idxs.size(); ++i)
        {
            //index ordered_idx_index by the index of the tracker and set that spot to the index of the associated point
            ordered_idx_index[ ordered_idxs[i].first ] = ordered_idxs[i].second;
        }

        for (uint32_t i = 0; i < ordered_idx_index.size(); ++i)
        {
            //iUnorderedPoints[ordered_idx_index[i]].x -
            //if( iUnorderedPoints[ordered_idx_index[i]])
            //use the index of the associated tracker to get the index of the corresponding value.
            oCorresPoints.push_back( iUnorderedPoints[ ordered_idx_index[i] ] );
        }

        //Update the Kalman Filters with new info
        for (uint32_t i = 0; i < mKFilterPtrs.size(); ++i)
        {
            cv::Mat meas= cv::Mat( 2, 1, CV_64F );
            meas.at<double>(0) = oCorresPoints[i].x;
            meas.at<double>(1) = oCorresPoints[i].y;
            //fprintf( stderr, "Next():\n%d statePre\n", iTimestep );
            //printMat( kFilterPtrs[i]->statePre );
            //fprintf( stderr, "%d errorCovPre\n", iTimestep );
            //printMat( kFilterPtrs[i]->errorCovPre );
            cv::Mat pred_state = mKFilterPtrs[i]->statePost;
            //fprintf( stderr, "%d errorCovPost\n", iTimestep );
            //printMat( kFilterPtrs[i]->errorCovPost );
            //fprintf( stderr, "%d statePost\n", iTimestep );
            //printMat( kFilterPtrs[i]->statePost );

            //fprintf( stderr, "%d measurementMatrix\n", iTimestep );
            //printMat( kFilterPtrs[i]->measurementMatrix );
            //fprintf( stderr, "measurement\n");
            //printMat( meas );
            cv::Mat update = mKFilterPtrs[i]->correct( meas );
            //fprintf( stderr, "corrected measurement\nEnd Next()\n");
            //printMat( update );
        }
    }
    else
    {
        return false;
    }

    return true;
}

/*bool MultiPointTracker::nextGreedy( const std::vector< SmtPixel >& iUnorderedPoints, uint32_t iTimestep, std::vector< SmtPixel >& oCorresPoints )
 {
    if (iUnorderedPoints.empty())
    {
        return false;
    }
    cv::Mat_<int> costMat = getCostMat( iUnorderedPoints, iTimestep );
    //printMat( costMat );

    if (costMat.rows == numPoints)
    {

        std::vector< std::pair<int, int> > orderedIdxs = orderCorrespondingPoints( costMat );

        std::vector< int > orderedIdxIndex;
        orderedIdxIndex.resize( orderedIdxs.size() );
        for (uint32_t i = 0; i < orderedIdxs.size(); ++i)
        {
            //index orderedIdxIndex by the index of the tracker and set that spot to the index of the associated point
            orderedIdxIndex[ orderedIdxs[i].first ] = orderedIdxs[i].second;
        }

        for (uint32_t i = 0; i < orderedIdxIndex.size(); ++i)
        {
            //use the index of the associated tracker to get the index of the corresponding value.
            oCorresPoints.push_back( iUnorderedPoints[ orderedIdxIndex[i] ] );
        }

        //Update the Kalman Filters with new info
        for (uint32_t i = 0; i < kFilterPtrs.size(); ++i)
        {
            cv::Mat meas= cv::Mat( 2, 1, CV_64F );
            meas.at<double>(0) = oCorresPoints[i].x;
            meas.at<double>(1) = oCorresPoints[i].y;
            fprintf( stderr, "%d statePre\n", iTimestep );
            printMat( kFilterPtrs[i]->statePre );
            fprintf( stderr, "%d errorCovPre\n", iTimestep );
            printMat( kFilterPtrs[i]->errorCovPre );
            cv::Mat predState = kFilterPtrs[i]->statePost;
            fprintf( stderr, "%d errorCovPost\n", iTimestep );
            printMat( kFilterPtrs[i]->errorCovPost );
            fprintf( stderr, "%d statePost\n", iTimestep );
            printMat( kFilterPtrs[i]->statePost );

            fprintf( stderr, "%d measurementMatrix\n", iTimestep );
            printMat( kFilterPtrs[i]->measurementMatrix );
            fprintf( stderr, "measurement\n");
            printMat( meas );
            cv::Mat update = kFilterPtrs[i]->correct( meas );
            fprintf( stderr, "corrected measurement\n" );
            printMat( update );
        }
    }
    else
    {
        return false;
    }
    return true;
}*/

bool MultiPointTracker::GetPredictedStates( const uint32_t iTimestep, std::vector< cv::Point2d > & oPredictedLocations )
{

    oPredictedLocations.clear();
    for (uint32_t i = 0; i < mKFilterPtrs.size(); ++i)
    {
        double dT = (iTimestep - mKFilterTimestep[i])/100.0;

        mKFilterPtrs[i]->transitionMatrix.at<double>(2) = dT;
        mKFilterPtrs[i]->transitionMatrix.at<double>(7) = dT;
        //fprintf( stderr, "%d transitionMatrix\n", iTimestep );
        //printMat( kFilterPtrs[i]->transitionMatrix );
        //fprintf( stderr, "%d gain\n", iTimestep );
        //printMat( kFilterPtrs[i]->gain );


        cv::Mat pred_state = mKFilterPtrs[i]->predict();
        //cv::Mat predState = kFilterPtrs[i]->statePost;

        //fprintf( stderr, "Predicted States():\n%d statePre\n", iTimestep );
        //printMat( kFilterPtrs[i]->statePre );
        //fprintf( stderr, "%d errorCovPre\n", iTimestep );
        //printMat( kFilterPtrs[i]->errorCovPre );
        //cv::Mat predState = kFilterPtrs[i]->statePost;
        //fprintf( stderr, "%d errorCovPost\n", iTimestep );
        //printMat( kFilterPtrs[i]->errorCovPost );
        //fprintf( stderr, "%d statePost\n", iTimestep );
        //printMat( kFilterPtrs[i]->statePost );

        //fprintf( stderr, "%d measurementMatrix\nEnd Predicted States()\n", iTimestep );
        //printMat( kFilterPtrs[i]->measurementMatrix );
        cv::Point2d pred_point;
        pred_point.x = pred_state.at<double>(0);
        pred_point.y = pred_state.at<double>(1);
        //fprintf( stderr, "New Predicted State:\n" );
        //printMat( predState );
        //fprintf( stderr, "End New Predicted State\n" );

        oPredictedLocations.push_back( pred_point );
    }
    return true;
}

bool MultiPointTracker::FillInMissingPointsFromPredictions( const std::vector< SmtPixel >& iIncompleteList, const uint32_t iTimestep, std::vector< SmtPixel >&  oFilledInList )
{
    std::vector< cv::Point2d > predicted_locations;
    if (!GetPredictedStates( iTimestep, predicted_locations ))
    {
        std::fprintf( stderr, "Error in retrieving predicted states from filter.\n" );
        return false;
    }

    //uint32_t columnLength = iIncompleteList.size();
    //cv::Mat_<int> costMat( cv::Size( predictedLocations.size(), columnLength/*cols*/ ), std::numeric_limits<int>::max() );
    uint32_t column_length = predicted_locations.size();
    cv::Mat_<int> cost_mat( cv::Size( iIncompleteList.size(), column_length/*cols*/ ),
                            std::numeric_limits<int>::max() );


    //this matrix is used in an integer programming solver, so we need to make everything integers
    int smallest_num = std::numeric_limits<int>::max();
    for (int32_t row = 0; row < cost_mat.rows; ++row)
    {
        for (uint32_t col = 0; col < column_length; ++col)
        { //if we only get 3 points passed in, but expect 4, then those values for the 4th point default to very large.
            //if the point doesn't exist
            if (col < iIncompleteList.size())
            {
                cv::Point2d diff = iIncompleteList[col%column_length] - predicted_locations[row];
                int dist = round( sqrt( pow( diff.x, 2 ) + pow( diff.y, 2 ) ) * 1000 );
                if (dist < smallest_num)
                {
                    smallest_num = dist;
                }
                cost_mat.at<int>(row, col) = dist;
                //printMat( costMat );
            }
        }
    }
    //fprintf( stderr, "Matrix Full\n");
    //printMat( costMat );
    cost_mat += abs(smallest_num);
    //printMat( costMat );

    std::vector< std::pair<int, int> > ordered_idxs = OrderCorrespondingPoints( cost_mat );

    std::vector< int > indexes_used;
    for (uint32_t i = 0; i < iIncompleteList.size(); ++i)
    {
        //index orderedIdxIndex by the index of the tracker and set that spot to the index of the associated point
        indexes_used.push_back( ordered_idxs[i].second );
    }

    std::vector< unsigned int > indexes_not_used;
    for (unsigned int x = 0; x < predicted_locations.size(); ++x)
    {
        if (std::find( indexes_used.begin(), indexes_used.end(), x) == indexes_used.end() )
        {
            indexes_not_used.push_back( x );
        }
    }

    oFilledInList = iIncompleteList;
    for (unsigned int i = 0; i < indexes_not_used.size(); ++i)
    {
        oFilledInList.push_back( SmtPixel(predicted_locations[ i ], oFilledInList[0].Cam() ) );
    }
    return true;
}

cv::Mat_<int> MultiPointTracker::GetCostMat( const std::vector< SmtPixel >& iUnorderedPoints, uint32_t iTimestep )
{
    std::vector< cv::Point2d > predicted_locations;
    if (!GetPredictedStates( iTimestep, predicted_locations ))
    {
        std::fprintf( stderr, "Error in retrieving predicted states from filter.\n");
        cv::Mat empty_cost_mat;
        return empty_cost_mat;
    }
     ///@todo think of a way to solve this problem
    //We could probably handle 1 point swing on each side, but its complicated... better to not try right now.
    if (iUnorderedPoints.size() - mNumPoints != 0)
    {
        cv::Mat empty_cost_mat;
        return empty_cost_mat;
    }
    else
    {
        for (uint32_t i = 0; i < mKFilterTimestep.size(); ++i)
        {
            mKFilterTimestep[i] += iTimestep - mKFilterTimestep[i];
        }
    }
    uint32_t column_length = mNumPoints ;//std::max( numPoints, iUnorderedPoints.size() );
    cv::Mat_<int> cost_mat( cv::Size(mNumPoints/*rows*/, column_length/*cols*/),
                            std::numeric_limits<int>::max() );

    //this matrix is used in an integer programming solver, so we need to make everything integers
    int smallest_num = std::numeric_limits<int>::max();
    for (int32_t row = 0; row < cost_mat.rows; ++row)
    {
        for (uint32_t col = 0; col < column_length; ++col)
        { //if we only get 3 points passed in, but expect 4, then those values for the 4th point default to very large.
            //if the point doesn't exist
            if (col < iUnorderedPoints.size())
            {
                cv::Point2d diff = iUnorderedPoints[col%column_length] - predicted_locations[row];
                int dist = round( sqrt( pow( diff.x, 2 ) + pow( diff.y, 2 ) ) * 1000 );
                if (dist < smallest_num)
                {
                    smallest_num = dist;
                }
                cost_mat.at<int>(row, col) = dist;
                //printMat( costMat );
            }
        }
    }
    //fprintf( stderr, "Matrix Full\n");
    //printMat( costMat );
    cost_mat += abs( smallest_num );
    //printMat( costMat );
    return cost_mat;
}

std::vector< std::pair<int, int> > MultiPointTracker::OrderCorrespondingPoints(const cv::Mat_<int>& iCostMat)
{
    ///@todo should probably just make the cost matrix using an Eigen Mat instead of CV...
    ///
    Eigen::Matrix<int, Dynamic, Dynamic, RowMajor> eigen_mat;
    opensource::cv2eigen( iCostMat, eigen_mat );

    opensource::Munkres hungarianAlg( eigen_mat.array() );
    std::vector< std::pair<int, int> > orderedIdxs = hungarianAlg.run();

    return orderedIdxs;
}
