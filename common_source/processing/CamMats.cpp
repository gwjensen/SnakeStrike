#include "eigen_cv_conversions.h"

#include "CamMats.h"

//Have to initialize this in a cpp file otherwise we get a linking problem
CamMats* CamMats::mpsInstance = nullptr;

//This is a static function
CamMats* CamMats::Instance()
{
    if (NULL == CamMats::mpsInstance)
    {
        CamMats::mpsInstance = new CamMats;
    }
    return CamMats::mpsInstance;
}

void CamMats::Init( const std::string& iPathToInputFile )
{
    if ( isInit() )
    {
        ClearData();
    }
    //CamMats* cam_matrices = CamMats.Instance();
    //if (!cam_matrices->isInit())
    //{
    //    assert("Must initialize CamMats object before use.");
    //}
    std::fprintf( stderr, "Loading Matrices from %s\n", iPathToInputFile.c_str() );
    cv::FileStorage fs( iPathToInputFile, cv::FileStorage::READ );

    int num_cameras = 0;
    fs["nCameras"] >> num_cameras;

    std::fprintf( stderr, "We expect %d cameras.\n", num_cameras );
    //init the vectors with empty matrices for the correct size of the vector


    std::fprintf( stderr, "Getting intrinsic matrices\n" );
    //Fill vector with intrinsic matrices
    std::string prefix = "camera_matrix_";
    //std::fprintf( stderr, "   using %s \n", prefix.c_str() );

    cv::Mat_<double> project_homogenous = cv::Mat::zeros( 3, 4, CV_64FC1 );
    project_homogenous.at<double>(0,0) = 1;
    project_homogenous.at<double>(1,1) = 1;
    project_homogenous.at<double>(2,2) = 1;
    project_homogenous.at<double>(2,3) = 1;
    cv::Mat_<double> project_homogenous_transpose;
    cv::transpose( project_homogenous, project_homogenous_transpose );
    //ProjectHomogenous.at<double>(2,3) = 0;

    for (int i = 0; i< num_cameras; ++i)
    {
        std::string name = prefix + std::to_string(i);
        //std::fprintf( stderr,"            matrix %i\n", i );
        cv::Mat tmp_32f;
        cv::Mat_<double> tmp_64f;
        fs[name] >> tmp_32f;
        tmp_32f.convertTo( tmp_64f, CV_64F );
        mK.push_back( tmp_64f );
        cv::Mat_<double> k_h = mK[i] * project_homogenous;
        k_h.at<double>(2,3) = 1; //have to convert from 3d back to homogenous
        mKH.push_back( k_h );
        cv::Mat_<double> k_inv_h = mK[i].inv() * project_homogenous;
        k_inv_h.at<double>(2,3) = 1; //have to convert from 3d back to homogenous
        mKInvH.push_back( k_inv_h );
        //printMat( iCamMatrices.K[i] );
        //printMat( iCamMatrices.KH[i] );
        //printMat( iCamMatrices.KinvH[i] );


    }
    //std::fprintf( stderr, "\nGetting extrinsic matrices \n");

    //Fill vector with extrinsic matrices
    prefix = "camera_pose_";
    //pose is the inverse of extrinsic
    //std::fprintf( stderr, "   using %s \n", prefix.c_str() );
    for (int i = 0; i< num_cameras; ++i)
    {
        std::string name = prefix + std::to_string( i );
        //std::fprintf( stderr,"____matrix %i\n", i );
        cv::Mat tmp_32f, tmp_64f;
        fs[name] >> tmp_32f;
        tmp_32f.convertTo( tmp_64f, CV_64F );

        //extrinsic is the inverse of the pose matrix and created lower down in this function
        mPose.push_back( tmp_64f( cv::Range(0,4), cv::Range(0,4) ) );
        //printMat( iCamMatrices.pose[i] );

    }

    std::fprintf( stderr, "\nGetting rotational matrices  \n");

    //Fill vector with rotational matrices, assume 0 based counting
    for (int i = 0; i< num_cameras; ++i)
    {
        cv::Mat_<double> rotation = cv::Mat::eye( 4, 4, CV_64FC1 );
        cv::Mat_<double> small_rot( rotation, cv::Rect( 0,0, 3, 3 ) );
        cv::Mat_<double> tmp_pose = mPose[i].inv();
        tmp_pose( cv::Range( 0, 3 ), cv::Range( 0, 3 ) ).copyTo( small_rot );
        //std::fprintf( stderr, "\nRotation Matrix\n");
        //printMat( Rotation );
        //std::fprintf( stderr,"____matrix %i\n", i);
        mR.push_back( rotation );

        //transpose of a rotation is the inverse
        mRInv.push_back( rotation.inv() );
        //printMat( iCamMatrices.R[i] );
        //printMat( iCamMatrices.Rinv[i] );
    }
    std::fprintf( stderr, "\nGetting translational matrices \n" );

    //Fill vector with translation matrices, assume 0 based counting
    for (int i = 0; i< num_cameras; ++i)
    {
        cv::Point3d c;
        cv::Mat_<double> tmpC = mPose[i].inv();
        c.x = tmpC.at<double>( 0, 3 );
        c.y = tmpC.at<double>( 1, 3 );
        c.z = tmpC.at<double>( 2, 3 );

        mC.push_back( c );

        cv::Mat_<double> c_mat = cv::Mat::zeros( 3, 1, CV_64FC1 );
        c_mat.at<double>(0,0) = mPose[i].at<double>( 0, 3 ) * -1;
        c_mat.at<double>(1,0) = mPose[i].at<double>( 1, 3 ) * -1;
        c_mat.at<double>(2,0) = mPose[i].at<double>( 2, 3 ) * -1;

        cv::Mat_<double> c_coords = cv::Mat::eye( 4, 4, CV_64FC1 );
        cv::Mat_<double> pose_inv = mPose[i].inv();
        cv::Mat_<double> rc( pose_inv, cv::Rect( 0,0, 3, 3 ) );
        cv::Mat_<double> t = rc * c_mat;
        //printMat(iCamMatrices.pose[i]);
        //printMat(iCamMatrices.pose[i].inv());
        //printMat(Rc);
        //printMat(CMat);
        //printMat(t);
        c_coords.at<double>(0,3) = t.at<double>( 0, 0 ) ;
        c_coords.at<double>(1,3) = t.at<double>( 1, 0 ) ;
        c_coords.at<double>(2,3) = t.at<double>( 2, 0 ) ;

        //std::fprintf( stderr,"____matrix %i\n", i );

        cv::Mat_<double> c_coords_inv = cv::Mat::eye( 4, 4, CV_64FC1 );
        c_coords_inv.at<double>(0,3) = t.at<double>( 0, 0 ) * -1;
        c_coords_inv.at<double>(1,3) = t.at<double>( 1, 0 ) * -1;
        c_coords_inv.at<double>(2,3) = t.at<double>( 2, 0 ) * -1;

        mT.push_back( c_coords );
        mTInv.push_back( c_coords_inv );
        //printMat( cv::Mat( iCamMatrices.T[i] ) );
        //printMat( cv::Mat( iCamMatrices.Tinv[i] ) );


    }


    //extrinsic is the inverse of the pose
    for (unsigned int i = 0; i < mPose.size(); ++i)
    {
        mExtrinsic.push_back( mTInv[i] * mRInv[i] );
        //std::fprintf( stderr, "Extrinsic %d\n", i);
        //printMat( iCamMatrices.extrinsic[i] );
        //std::fprintf( stderr, "Rinv * Tinv %d\n", i);
        //printMat( iCamMatrices.Rinv[i] * iCamMatrices.Tinv[i] );
        //std::fprintf( stderr, "Tinv * Rinv %d\n", i);
        //printMat( iCamMatrices.Tinv[i] * iCamMatrices.Rinv[i] );

    }

    std::fprintf( stderr, "\nGetting distortion matrices \n" );

    //Get the distortion matrices for each camera
    prefix = "camera_distortion_";
    for (int i = 0; i< num_cameras; ++i)
    {
        std::string name = prefix + std::to_string(i);
        //std::fprintf( stderr,"____matrix %i\n", i );
        cv::Mat tmp_32f, tmp_64f;
        fs[name] >> tmp_32f;
        tmp_32f.convertTo( tmp_64f, CV_64F );
        mDistortion.push_back( tmp_64f );
        //printMat( iCamMatrices.distortion[i] );
    }


    //Get the Projective Matrices
    std::fprintf( stderr, "Getting the projective Matrices\n");
    for (int i = 0; i < num_cameras; ++i)
    {
        cv::Mat_<double> m_inv_mat;
        cv::Mat_<double> m_mat;

        m_inv_mat = mT[i] * mR[i] * project_homogenous_transpose * mK[i].inv() ;

        //MMat = iCamMatrices.K[i] * ProjectHomogenous * iCamMatrices.Tinv[i] * iCamMatrices.Rinv[i] ;
        m_mat =  mK[i] * project_homogenous;
        //MMat.at<double>(2,3) = 1;
        //std::fprintf( stderr, "MMat Matrix before extrinsic mult:\n");
        //printMat(MMat);
        //cv::transpose( MMat, MMat );
        //MMat = MMat * iCamMatrices.Tinv[i] * iCamMatrices.Rinv[i];
        m_mat = m_mat * mRInv[i] * mTInv[i];

        cv::transpose( m_inv_mat, m_inv_mat ); //4x3 to 3x4
        mMInv.push_back( m_inv_mat );
        mM.push_back( m_mat );

        //create an Eigen representation of the projection matrices for use with EPVH
        Eigen::Matrix< double, 3, 4 > tmpEigen;
        opensource::cv2eigen( mM[i], tmpEigen );
        mEigenM.push_back( tmpEigen );
        //printMat( iCamMatrices.M[i] );
    }
    mIsInit = true;
}

void CamMats::ClearData()
{
    //::TRICKY:: We have to do this in order to make sure the cv::Mat objects get deleted.
    // Before we might have tried to just do a clear() on the vectors, but that doesn't seem to work
    {
        std::vector< cv::Mat_< double > > delete_vector;
        delete_vector.swap(mK);
    }
    {
        std::vector< cv::Mat_< double > > delete_vector;
        delete_vector.swap(mKH);
    }
    {
        std::vector< cv::Mat_< double > > delete_vector;
        delete_vector.swap(mKInvH);
    }
    {
        std::vector< cv::Mat_< double > > delete_vector;
        delete_vector.swap(mPose);
    }
    {
        std::vector< cv::Mat_< double > > delete_vector;
        delete_vector.swap(mExtrinsic);
    }
    {
        std::vector< cv::Mat_< double > > delete_vector;
        delete_vector.swap(mR);
    }
    {
        std::vector< cv::Mat_< double > > delete_vector;
        delete_vector.swap(mRInv);
    }
    {
        std::vector< cv::Mat_< double > > delete_vector;
        delete_vector.swap(mT);
    }
    {
        std::vector< cv::Mat_< double > > delete_vector;
        delete_vector.swap(mTInv);
    }
    {
        std::vector<cv::Point3d> delete_vector;
        delete_vector.swap(mC);
    }
    {
        std::vector< cv::Mat_< double > > delete_vector;
        delete_vector.swap(mDistortion);
    }
    {
        //camera projection matrix
        std::vector< cv::Mat_< double > > delete_vector;
        delete_vector.swap(mM);
    }
    {
        std::vector< cv::Mat_< double > > delete_vector;
        delete_vector.swap(mMInv);
    }
    {
        std::vector< Eigen::Matrix< double, 3, 4 > > delete_vector;
        delete_vector.swap(mEigenM);
    }

    mIsInit = false;
}

void CamMats::GetPrincipalPointInfo( const int& iCamIdx, double& oFocalLength, cv::Point2d& oOpticalCenter ) const
{
    //std::fprintf( stderr, "oFocalLength = (iCamMatrix.K[iCamIdx].at< double >(0,0) + iCamMatrix.K[iCamIdx].at< double >(1,1))/2 iCamIdx:%d\n",iCamIdx);
    //std::fprintf( stderr, "oFocalLength = (           %d                ", iCamMatrix.K[iCamIdx].at< double >(0,0));
    //std::fprintf( stderr, "+      %d      )/2 camIdx:%d\n", iCamMatrix.K[iCamIdx].at< double >(1,1));
    oFocalLength = (mK[iCamIdx].at< double >(0,0) + mK[iCamIdx].at< double >(1,1))/2 ;
    oOpticalCenter = cv::Point2d( mK[iCamIdx].at< double >(0,2), mK[iCamIdx].at< double >(1,2) );
}

void CamMats::GetPrincipalPointInfo( std::vector<double>& oFocalLengths, std::vector< cv::Point2d >& oOpticalCenters) const
{
    for( unsigned int cam = 0; cam < mK.size(); ++cam ){
        oFocalLengths.push_back( (mK[cam].at< double >(0,0) + mK[cam].at< double >(1,1))/2 );
        oOpticalCenters.push_back( cv::Point2d( mK[cam].at< double >(0,2), mK[cam].at< double >(1,2) ) );
    }
}

