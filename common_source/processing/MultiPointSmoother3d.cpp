#include "MultiPointSmoother3d.h"

MultiPointSmoother3d::MultiPointSmoother3d( uint8_t iNumPoints )
    :MultiPointTracker( iNumPoints ), mIsInitialized( false )
{

}
MultiPointSmoother3d::~MultiPointSmoother3d()
{

}


void MultiPointSmoother3d::Initialize(uint32_t iFirstMatchTimestep, std::vector< cv::Point3d > iStartingPoints )
{

    mIsInitialized = true;
    //initalize KalmanFilters with these points
    int state_size = 6;
    int meas_size = 3;
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
        // [ 1 0 0  dT 0  0 ]
        // [ 0 1 0  0  dT 0 ]
        // [ 0 0 1  0  0  dT]
        // [ 0 0 0  1  0  0 ]
        // [ 0 0 0  0  1  0 ]
        // [ 0 0 0  0  0  1 ]
        cv::setIdentity(kf->transitionMatrix);

        // Measure Matrix H
        // [ 1 0 0 0 0 0]
        // [ 0 1 0 0 0 0]
        // [ 0 0 1 0 0 0]
        kf->measurementMatrix = cv::Mat::zeros( meas_size, state_size, type );
        kf->measurementMatrix.at<double>(0) = 1.0;
        kf->measurementMatrix.at<double>(7) = 1.0;
        kf->measurementMatrix.at<double>(14) = 1.0;


        // Process Noise Covariance Matrix Q
        // [ Ex   0   0     0    0    0 ]
        // [ 0    Ey  0     0    0    0 ]
        // [ 0    0   Ez    0    0    0 ]
        // [ 0    0   0     Ev_x 0    0 ]
        // [ 0    0   0     0    Ev_y 0 ]
        // [ 0    0   0     0    0    Ev_z  ]
        cv::setIdentity( kf->processNoiseCov, cv::Scalar::all( 1e-2 ) );

        kf->processNoiseCov.at<float>(0) = 0;//1e-2;
        kf->processNoiseCov.at<float>(7) = 0;//1e-2;
        kf->processNoiseCov.at<float>(14) = 0;
        //kf->processNoiseCov.at<float>(10) = 1e-1;//2.0f;
        //kf->processNoiseCov.at<float>(15) = 1e-1;

        // Measures Noise Covariance Matrix R
        cv::setIdentity( kf->measurementNoiseCov, cv::Scalar::all( 1e-2) );

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
        state.at<double>(2) = iStartingPoints[i].z;
        state.at<double>(3) = 1e-4;
        state.at<double>(4) = 1e-4;
        state.at<double>(5) = 1e-4;

        kf->statePost = state;

        mKFilterPtrs.push_back( kf );
        mKFilterTimestep.push_back( iFirstMatchTimestep );
    }
}

bool MultiPointSmoother3d::Next( const std::vector< cv::Point3d >& iMeasuredPoints, uint32_t iTimestep, std::vector< cv::Point3d >& oSmoothedPoints )
{
    if (iMeasuredPoints.empty() || iMeasuredPoints.size() < mNumPoints)
    {
        return false;
    }
    oSmoothedPoints.clear();

    for (uint32_t i = 0; i < mKFilterPtrs.size(); ++i)
    {
        double dT = (iTimestep - mKFilterTimestep[i])/100.0;

        mKFilterPtrs[i]->transitionMatrix.at<double>(3) = dT;
        mKFilterPtrs[i]->transitionMatrix.at<double>(10) = dT;
        mKFilterPtrs[i]->transitionMatrix.at<double>(17) = dT;
        //fprintf( stderr, "%d transitionMatrix\n", iTimestep );
        //printMat( kFilterPtrs[i]->transitionMatrix );
        //fprintf( stderr, "%d gain\n", iTimestep );
        //printMat( kFilterPtrs[i]->gain );


        cv::Mat pred_state = mKFilterPtrs[i]->predict();
    }


    for (uint32_t i = 0; i < mNumPoints/*kFilterTimestep.size()*/; ++i)
    {
        mKFilterTimestep[i] += iTimestep - mKFilterTimestep[i];
    }



    //Update the Kalman Filters with new info
    for (uint32_t i = 0; i < mNumPoints/*kFilterPtrs.size()*/; ++i)
    {
        cv::Mat meas= cv::Mat( 3, 1, CV_64F );
        meas.at<double>(0) = iMeasuredPoints[i].x;
        meas.at<double>(1) = iMeasuredPoints[i].y;
        meas.at<double>(2) = iMeasuredPoints[i].z;
        //fprintf( stderr, "%d statePre\n", iTimestep );
        //printMat( kFilterPtrs[i]->statePre );
        //fprintf( stderr, "%d errorCovPre\n", iTimestep );
        //printMat( kFilterPtrs[i]->errorCovPre );
        //cv::Mat predState = kFilterPtrs[i]->statePost;
        //fprintf( stderr, "%d errorCovPost\n", iTimestep );
        //printMat( kFilterPtrs[i]->errorCovPost );
        //fprintf( stderr, "%d statePost\n", iTimestep );
        //printMat( kFilterPtrs[i]->statePost );

        //fprintf( stderr, "%d measurementMatrix\n", iTimestep );
        //printMat( kFilterPtrs[i]->measurementMatrix );
        //fprintf( stderr, "measurement\n");
        //printMat( meas );
        cv::Mat_<double> update = mKFilterPtrs[i]->correct( meas );
        //fprintf( stderr, "corrected measurement\n");
        //printMat( update );
        cv::Point3d newPos( update.at<double>(0),
                            update.at<double>(1),
                            update.at<double>(2) );
        oSmoothedPoints.push_back( newPos );
    }

    return true;

}
