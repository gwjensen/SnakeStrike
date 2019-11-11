#include <opencv2/core.hpp>
#include <opencv2/sfm.hpp>
#include <iostream>
#include <Eigen/Dense>

#include "opensource/Multiview3dRecon-KanataniBook/srcs/MultipleViewTriangulation.h"
#include "opensource/eigen_cv_conversions.h"
#include "opensource/distance_3d_seg.h"
#include "ThreadPool.h"
#include "common_types.h" 
#include "visualization/opencv_viz.h"

#include "Triangulation.h"

cv::Point2d operator*(cv::Mat_<double> iM, const cv::Point2d& iP)
{
    cv::Mat_<double> src(3/*rows*/,1 /* cols */);

    src(0,0)=iP.x;
    src(1,0)=iP.y;
    src(2,0)=1.0;

    cv::Mat_<double> dst = iM*src; //USE MATRIX ALGEBRA
    return cv::Point2d(dst(0,0),dst(1,0));
}


////Creates a ray with arbitrary specified length from the camera through the image pixel specified.
void CalculateTransformedPosition( const int& iCamIdx,
                                   const std::vector< SmtPixel >& iCandidateEndPoints,
                                   const CamMats& iCamMatrix,
                                   cv::Point3d& oStart,
                                   std::vector< cv::Point3d >& oEnds,
                                   float iExtendLength )
{
    double focal_length = 0;
    cv::Point2d optical_center;
    iCamMatrix.GetPrincipalPointInfo( iCamIdx, focal_length, optical_center );
    //std::fprintf( stderr, "The optical center dimensions are x: %lf, y: %lf\n", opticalCenter.x, opticalCenter.y );

    //cv::Mat_< double > startPointMat(3,1,CV_64FC1);

    oStart = iCamMatrix.C()[iCamIdx];

    //std::fprintf( stderr, "\nThe startpoint is at (%lf,%lf,%lf)\n", oStart.x, oStart.y, oStart.z );

    for (unsigned int i = 0; i < iCandidateEndPoints.size(); ++i)
    {
        std::vector< double > cand_point_array = {(iCandidateEndPoints[i].x ), ( iCandidateEndPoints[i].y ), 1};

        cv::Mat_<double> cand_point( cand_point_array ); //creates a 3x1 matrix
        //cv::transpose( candPoint, candPoint );// 1x3
        //candPoint = candPoint * iCamMatrix.Minv[iCamIdx];
        //cv::transpose( candPoint, candPoint );// 3x1
        //---- DEBUG
        std::vector< double > test_cand_point_array = {( iCandidateEndPoints[i].x ), ( iCandidateEndPoints[i].y ), 1};

        //std::fprintf( stderr,
        //              "candPointArray before translations: (%lf,%lf,%lf)\n",
        //               candPointArray[0], candPointArray[1], candPointArray[2] );
        cand_point =  iCamMatrix.K()[iCamIdx].inv() * cand_point ;//+ iCamMatrix.Proj[iCamIdx](Range(0,3), Range(3,4));

        //std::fprintf( stderr, "\nCandpoint after K matrix mult:\n");
        PrintMat( cand_point );

        cand_point = iCamMatrix.R()[iCamIdx](cv::Range(0,3), cv::Range(0,3)) * cand_point;

        //std::fprintf( stderr, "\nCandpoint after R matrix mult:\n");
        PrintMat( cand_point );

        cand_point = cand_point + cv::Mat( iCamMatrix.C()[iCamIdx] );

        //std::fprintf( stderr, "\nCandpoint after T matrix add:\n");
        PrintMat( cand_point );

        //std::fprintf( stderr,
        //              "candPointArray used for init of candPoint(after all ops) (%lf,%lf,%lf)\n",
        //              candPointArray[0], candPointArray[1], candPointArray[2] );
        //std::fprintf( stderr,
        //              "candPointArray used for init of testPoint(before any ops) (%lf,%lf,%lf)\n",
        //              testCandPointArray[0], testCandPointArray[1], testCandPointArray[2] );

        //__Test Matrices___________________________________________________________
        //cv::Mat_<double> testPoint(3, 1, CV_64FC1, &candPointArray);
        cv::Mat_<double> test_point( test_cand_point_array ); //creates a 3x1 matrix
        //std::fprintf( stderr, "\ntestPoint matrix\n");
        PrintMat( test_point );


        //std::fprintf( stderr, "\n K Matrix\n");
        PrintMat( iCamMatrix.K()[iCamIdx].inv() );

        cv::Mat_<double> projection_mat = cv::Mat::zeros( 3, 4, CV_64FC1 ); //projection
        projection_mat.at<double>(0,0) = 1;
        projection_mat.at<double>(1,1) = 1;
        projection_mat.at<double>(2,2) = 1;
        projection_mat.at<double>(2,3) = 1; //have to convert from 3d back to homogenous
        //std::fprintf( stderr, "\nProjection Matrix\n");
        //printMat( Projection );

        cv::Mat_<double> rotation_mat = iCamMatrix.R()[iCamIdx].clone();//rotation

        //std::fprintf( stderr, "\nRotation Matrix\n");
        //printMat( Rotation );

        cv::Mat_<double> translation_mat = iCamMatrix.T()[iCamIdx].clone(); //translation

        //std::fprintf( stderr, "\nTranslation Matrix\n");
        //printMat( Translation );


        cv::Mat_<double> k_inv;
        cv::Mat_<double> k_inv_2;
        cv::Mat_<double> k_inv_3;
        cv::transpose( projection_mat, projection_mat);
        k_inv = translation_mat * rotation_mat * projection_mat * iCamMatrix.K()[iCamIdx].inv() ;

        cv::Mat_<double> output = k_inv * test_point  ;

        //std::fprintf( stderr,
        //              "\n testPoint after Translation* Rotation * Projection * iCamMatrix.K[iCamIdx].inv() * point matrix mult:\n");
        //printMat( output );

        cv::transpose( test_point, test_point ); //1x3
        cv::transpose(  k_inv , k_inv );

        k_inv = test_point * k_inv ;
        //std::fprintf( stderr, "\n testPoint after testPoint^T * Kinv^T  testPoint^T matrix mult:\n");
        //printMat( Kinv );

        k_inv_2 =   test_point * iCamMatrix.MInv()[iCamIdx];
        //printMat( Kinv2 );

        cv::Mat_<double> project_homogenous = cv::Mat::zeros( 3, 4, CV_64FC1 );
        project_homogenous.at<double>(0,0) = 1;
        project_homogenous.at<double>(1,1) = 1;
        project_homogenous.at<double>(2,2) = 1;
        project_homogenous.at<double>(2,3) = 1;
        cv::transpose( project_homogenous, project_homogenous );
        cv::transpose( test_point, test_point ); //3,1
        cv::Mat_<double> m_mat = (project_homogenous * iCamMatrix.K()[iCamIdx].inv()) * test_point;
        cv::transpose( test_point, test_point ); //1x3
        //std::fprintf( stderr, "MMat After K Matrix and H:\n");
        //printMat(MMat);
        //cv::transpose( MMat, MMat );
        //cv::Mat_<double> m_mat_2 = iCamMatrix.R()[iCamIdx] * m_mat;

        //std::fprintf( stderr, "MMat After T Matrix:\n");
        //printMat( iCamMatrix.R[iCamIdx] );
        //printMat(MMat2);
        //cv::Mat_<double> MMat3 = iCamMatrix.T()[iCamIdx] * m_mat_2;
        //std::fprintf( stderr, "MMat After R Matrix:\n");
        //printMat( iCamMatrix.T[iCamIdx] );
        //printMat(MMat3);


        //cv::transpose( testPoint, testPoint); //1x3
        k_inv_3 =  test_point * iCamMatrix.MInv()[iCamIdx];
        std::fprintf( stderr, "\n testPoint after Kinv3 matrix mult:\n");
        cv::transpose( k_inv_3, k_inv_3 );
        //printMat( Kinv3 );

        std::fprintf( stderr,
                      "\n----------------------\nTest output endpoint (%lf, %lf, %lf) compared to (%lf, %lf, %lf) /\n",
                      (output.at<double>(0,0)/output.at<double>(0,3)),
                      (output.at<double>(0,1)/output.at<double>(0,3)),
                      (output.at<double>(0,2)/output.at<double>(0,3)),
                      cand_point.at<double>(0,0),
                      cand_point.at<double>(1,0),
                      cand_point.at<double>(2,0) );

        std::fprintf( stderr,
                      "\n----------------------\nKinv2 Test output endpoint (%lf, %lf, %lf) compared to (%lf, %lf, %lf) \n",
                      (k_inv_2.at<double>(0,0)/k_inv_2.at<double>(0,3)),
                      (k_inv_2.at<double>(0,1)/k_inv_2.at<double>(0,3)),
                      (k_inv_2.at<double>(0,2)/k_inv_2.at<double>(0,3)),
                      cand_point.at<double>(0,0),
                      cand_point.at<double>(1,0),
                      cand_point.at<double>(2,0)  );

        std::fprintf( stderr,
                      "\n----------------------\nKinv3 Test output endpoint (%lf, %lf, %lf) compared to (%lf, %lf, %lf) \n",
                      (k_inv_3.at<double>(0,0)/k_inv_3.at<double>(0,3)),
                      (k_inv_3.at<double>(0,1)/k_inv_3.at<double>(0,3)),
                      (k_inv_3.at<double>(0,2)/k_inv_3.at<double>(0,3)),
                      cand_point.at<double>(0,0),
                      cand_point.at<double>(1,0),
                      cand_point.at<double>(2,0) );

        //__End Test Matrices____________________________________________________________

        //--END DEBUG

        cv::Point3d final_point = cv::Point3d( cand_point.at<double>(0,0),
                                               cand_point.at<double>(1,0),
                                               cand_point.at<double>(2,0) );
        //cv::Point3d finalPoint = cv::Point3d( Kinv2.at<double>(0),  Kinv2.at<double>(1),  Kinv2.at<double>(2) );

        final_point = opensource::extendRay(oStart, final_point, iExtendLength );

        oEnds.push_back( final_point );
    }
}

/* Gives you an output that has the triangulated pixels sorted into timesteps */
int TriangulatePixelsWithHistory( const PixelSet& iPixelSet,
                                  const CamMats& iCamMats,
                                  std::vector< std::vector< cv::Point3d > >& oCalcdPoints )
{
    //loop through each timestep
    int pixels_triangulated = 0;
    std::fprintf( stderr, "triangulatePixels: There are %lu timesteps.\n", iPixelSet.size() );

    unsigned int history_length = 10;
    for (unsigned int j = 0 + history_length; j < iPixelSet.size(); ++j)
    {
        if (iPixelSet[j].size() > 0)
        {
            std::vector<cv::Mat> cameras_in_timestep;
            std::vector<cv::Mat> keypoints;
            //std::fprintf( stderr, "----timestep: %d, # images: %lu \n", j, iPixelSet[j].size() );
            //add the cam matrices for each image in a timestep in the order the images are stored.
            //int totalPointsInAllImages = 0;


            for (unsigned int i = 0; i < iPixelSet[j].size(); ++i)
            {
                //std::fprintf( stderr,
                //              "Projection matrix rows = %d, cols = %d\n",
                //              iExtrinsicMats[i].rows, iExtrinsicMats[i].cols );
                int count = 0;
                int timestep_skipped = 0;
                int num_points = 1;
                cv::Mat tmp( 2, num_points *  history_length, CV_64FC1 ); //64 expected by sfm, just converting here to save time later.
                for (unsigned int h = 0; h < history_length; ++h)
                {
                    int idx = j - h;
                    if (iPixelSet[idx].size() > 0)
                    {
                        if (iPixelSet[idx][i].size() > 0)
                        {
                            //std::fprintf( stderr,
                            //              "-------number of points in this image %lu\n",
                            //              iPixelSet[idx][i].size());
                            for (unsigned int x = 0 ; x < tmp.cols/history_length; ++x, ++count)
                            {
                                tmp.at<double>(0, count) = iPixelSet[idx][i][x].x;
                                tmp.at<double>(1, count) = iPixelSet[idx][i][x].y;
                                //std::fprintf( stderr,
                                //              "---------Point(%f, %f)\n",
                                //              iPixelSet[idx][i][x].x, iPixelSet[idx][i][x].y );
                                //std::fprintf( stderr,
                                //              "-----------tmp(%lf, %lf)\n",
                                //              tmp.at<double>(1, x),  tmp.at<double>(2, x) );
                            }

                            //totalPointsInAllImages += iPixelSet[idx][i].size();

                            //camerasInTimestep.push_back( iExtrinsicMats[index](Range(0,3), Range(0,4) ));
                        }
                        else
                        {
                            std::cerr << "Error processing pixel set in triangulatePixels():no pixels" << std::endl;
                        }
                    }
                    else
                    {
                        ++timestep_skipped;
                    }
                }
                if (timestep_skipped)
                {
                    tmp = tmp.clone()( cv::Range( 0, 2 ),
                                       cv::Range( 0, (num_points *  history_length) - timestep_skipped) );
                }
                //std::fprintf( stderr, "\nThe tmp Matrix:\n" );
                //printMat( tmp );
                keypoints.push_back( tmp );

                ///@TODO This only works for the single point with guaranteed fixed number of cameras in each timestep
                int index = iPixelSet[j][i][0].Cam();
                //std::fprintf( stderr, "-------adding camera %d matrix to list.\n", index );
                cameras_in_timestep.push_back( iCamMats.Extrinsic()[index]( cv::Range( 0, 3 ), cv::Range( 0, 4 ) ) );
            }

            //std::fprintf( stderr,
            //              "Calling sfm::trangulatePoints with %d points to triangulate with each other from %lu cameras and %lu matrices.\n",
            //              totalPointsInAllImages, iPixelSet[j].size(), camerasInTimestep.size());
            std::vector< cv::Point3d > tmp_3d_points;
            cv::Mat tmp_mat;

            //std::fprintf( stderr, "points2d_tmp[0].at<double>(0, 0) = %f\n", keypoints[0].at<double>(0,0) );
            cv::sfm::triangulatePoints( keypoints, cameras_in_timestep, tmp_mat );
            //std::fprintf( stderr,
            //              "----done with sfm::triangulatePoints --> %d 3D points created, mat rows = %d, mat cols = %d, chans = %d\n",
            //              tmpMat.cols, tmpMat.rows, tmpMat.cols, tmpMat.channels() );

            //Put Mat into std::vector<cv::Point3d>
            for( int x = 0 ; x < 1; ++x ){
                cv::Point3d tmp_point( tmp_mat.at<double>(0, x), tmp_mat.at<double>(1, x), tmp_mat.at<double>(2,x) );
                tmp_3d_points.push_back( tmp_point );
                std::fprintf( stderr,
                              "-------- Adding new 3d point to output std::vector (%f, %f, %f)\n",
                              tmp_point.x, tmp_point.y, tmp_point.z );
            }
            oCalcdPoints.push_back( tmp_3d_points );
            pixels_triangulated += tmp_3d_points.size();
        }
    }


    return pixels_triangulated;
}

void TriangulatePixelsTimestepGroup(uint32_t iStartTimestep,
                                    uint32_t iEndTimestep,
                                    const PixelSet& iPixelSet,
                                    std::vector< std::vector< cv::Point3d > >& oCalcdPoints )
{
    CamMats* cam_matrices = CamMats::Instance();
    if (!cam_matrices->isInit())
    {
        assert( "Must initialize CamMats object before use." );
    }
    for (uint32_t j = iStartTimestep; j < iEndTimestep; ++j)
    {
        unsigned int count = 0;
        for (unsigned int i = 0; i < iPixelSet[j].size(); ++i)
        {
            if (iPixelSet[j][i].size() > 0)
            {
                ++count;
            }
        }
        ///@todo this check should really be fixed on the generating size of the pixelset as iPixelSet.size will just come back w/ numCams and empty pixelsets for the cams
        if (iPixelSet[j].size() > 1 && count > 1)
        {
            std::vector< cv::Mat > cameras_in_timestep;
            std::vector< cv::Mat > cameras_in_timestep_oc;

            //std::fprintf( stderr, "\n----timestep: %d, # images: %lu \n", j, iPixelSet[j].size() );
            //add the cam matrices for each image in a timestep in the order the images are stored.
            int total_points_in_all_images = 0;
            std::vector< cv::Mat > keypoints;
            int index = 0;
            //int prevIndex = -1;
            for (unsigned int i = 0; i < iPixelSet[j].size(); ++i)
            {
                //std::fprintf( stderr, "timestep = %d, cam = %d\n", j, i );
                if (iPixelSet[j][i].size() > 0)
                {
                    //std::fprintf( stderr, "-------number of points in this image %lu\n", iPixelSet[j][i].size() );
                    //for( int idx = 0; idx < iPixelSet[j][i].size(); ++idx){
                    //    std::fprintf( stderr,
                    //                  "---------Point(%f, %f)\n",
                    //                  iPixelSet[j][i][idx].x, iPixelSet[j][i][idx].y );
                    //}
                    cv::Mat tmp( 2, iPixelSet[j][i].size(), CV_64FC1 ); //64 expected by sfm, just converting here to save time later.
                    //cv::RNG rng( 0xFFFFFFFF );
                    //cv::Point noise;
                    for (int x = 0 ; x < tmp.cols; ++x)
                    {
                        //noise.x = rng.uniform(-20,20);
                        //noise.y = rng.uniform(-20,20);
                        tmp.at<double>(0, x) = iPixelSet[j][i][x].x ;//+ noise.x;

                        tmp.at<double>(1, x) = iPixelSet[j][i][x].y ;//+ noise.y;
                        //std::fprintf( stderr, "---------Point(%f, %f)\n", iPixelSet[j][i][x].x, iPixelSet[j][i][x].y );
                    //	std::fprintf( stderr, "-----------tmp(%lf, %lf)\n",  tmp.at<double>(0, x),  tmp.at<double>(1, x) );
                    }

                    keypoints.push_back( tmp );
                    index = iPixelSet[j][i][0].Cam();
                    //std::fprintf( stderr, "-------adding camera %d matrix to list.\n", index );
                    total_points_in_all_images += iPixelSet[j][i].size();

                    cameras_in_timestep.push_back( cam_matrices->M()[index] );
                    cameras_in_timestep_oc.push_back( cam_matrices->M()[index] );
                    //prevIndex = index;
                }
                else{
                    std::cerr << "Error processing pixel set in triangulatePixels(): no pixels in timestep" << std::endl;
                }
            }
            //std::fprintf( stderr,
            //              "Calling sfm::trangulatePoints with %d points to triangulate with each other from %lu cameras and %lu matrices.\n",
            //              totalPointsInAllImages, iPixelSet[j].size(), camerasInTimestep.size() );

            //Optimize point locations to solve the M-linear contraints
            std::vector< cv::Mat > corrected_keypoints;

            CorrectTimestepPoints( cameras_in_timestep, keypoints, corrected_keypoints );

            //--DEBUG-------------------

            /*std::fprintf( stderr, "\n Non-corrected vs Corrected Keypoints for Trianguation:\n");
            for( int k = 0; k < correctedKeypoints.size(); ++k ){
                printMat( keypoints[k] );
                printMat( correctedKeypoints[k] );
            }

            PixelSet correctedPixels;
            std::vector< std::vector< ImagePixel > > timestepPixels;
            for( int c = 0; c < correctedKeypoints.size(); ++c ){
                std::vector< ImagePixel > camImagePixels;
                std::fprintf( stderr, "\nCorrectedKeypoint[%d] Mat:\n", c );
                printMat( correctedKeypoints[c] );
                for( int x = 0 ; x < correctedKeypoints[c].cols; ++x ){

                    ImagePixel tmpPoint( cv::Point2d(correctedKeypoints[c].at< double >(0, x),
                                         correctedKeypoints[c].at< double >(1, x) ),
                                         iPixelSet[j][c][0].cam );
                    camImagePixels.push_back( tmpPoint );
                }
                timestepPixels.push_back( camImagePixels );
            }
            correctedPixels.push_back( timestepPixels ); //only one timestep here, but have to have it in this format.
            std::fprintf( stderr, "Viewing the corrected Keypoints.\n");
            common::convertPointsToLinesForViewing( correctedPixels, iCamMats );*/
            //--END DEBUG ---------------

            std::vector< cv::Point3d > tmp_3d_points;
            cv::Mat tmp_mat;

            //std::fprintf( stderr, "points2d_tmp[0].at<double>(0, 0) = %f\n", correctedKeypoints[0].at< double >(0,0) );
            try
            {
                cv::sfm::triangulatePoints( corrected_keypoints, cameras_in_timestep, tmp_mat );
            }
            catch (cv::Exception ex)
            {
                std::fprintf( stderr, "Exception caught '%s'\n", ex.msg.c_str() );
                assert( false );
            }

            //std::fprintf( stderr,
            //              "----done with sfm::triangulatePoints --> %d 3D points created, mat rows = %d, mat cols = %d, chans = %d\n",
            //              tmpMat.cols, tmpMat.rows, tmpMat.cols, tmpMat.channels());

            //Put Mat into std::vector<cv::Point3d>
            for (int x = 0 ; x < tmp_mat.cols; ++x)
            {
                cv::Point3d tmp_point( tmp_mat.at< double >(0, x), tmp_mat.at< double >(1, x), tmp_mat.at< double >(2,x) );
                //tmpPoint = tmpPoint * 100;
                tmp_3d_points.push_back( tmp_point );
                //std::fprintf( stderr, "-------- Adding new 3d point to output std::vector (%lf, %lf, %lf)\n", tmpPoint.x, tmpPoint.y, tmpPoint.z );
            }

            oCalcdPoints[j] = tmp_3d_points;
            //pixelsTriangulated += tmp3dPoints.size();
        }
        else{
            std::vector< cv::Point3d >  empty_set;
            oCalcdPoints[j] = empty_set;
        }
    }
}

/* Gives you an output that has the triangulated pixels sorted into timesteps */
int TriangulatePixels( const PixelSet& iPixelSet,
                       std::vector< std::vector< cv::Point3d > >& oCalcdPoints )
{
    //loop through each timestep
    int pixels_triangulated = 0;
    std::fprintf( stderr, "\n\ntriangulatePixels: There are %lu timesteps.\n", iPixelSet.size() );

    //threadpool
    ThreadPool pool(15);
    uint32_t num_timesteps_per_thread = 100;

    oCalcdPoints.clear();
    oCalcdPoints.resize( iPixelSet.size() );
    for (uint32_t j = 0; j < iPixelSet.size(); )
    { //for each timestep
        if (iPixelSet.size() - j < num_timesteps_per_thread)
        {
            num_timesteps_per_thread = iPixelSet.size() - j;
        }
        pool.enqueue( boost::bind( TriangulatePixelsTimestepGroup,
                                   j,
                                   j + num_timesteps_per_thread,
                                   boost::ref( iPixelSet ),
                                   boost::ref( oCalcdPoints ) ));
        //debug//
//            triangulatePixelsTimestepGroup( j,
//                                            j + numTimestepsPerThread,
//                                            iPixelSet,
//                                            iCamMats,
//                                            oCalcdPoints );
//            //end debug//
        j += num_timesteps_per_thread;
    }

    for( uint32_t j = 0; j < oCalcdPoints.size(); ++j ){
        pixels_triangulated += oCalcdPoints[j].size();
    }
    return pixels_triangulated;
}

void TriangulatePixelsWithKalman( const PixelSet& iPixelSet,
                       const std::pair<int, std::set<int> >& iCamIndexesToExclude,
                       MultiPointSmoother3d& ioTracker,
                       const unsigned int iNumPoints,
                       std::vector< std::vector< cv::Point3d > >& oCalcdPoints )
{
    if (iCamIndexesToExclude.first == 0 || iNumPoints == 1 ){
        //just put this here to I don't have ot change the function signature
    }
    CamMats* cam_matrices = CamMats::Instance();
    if (!cam_matrices->isInit())
    {
        assert( "Must initialize CamMats object before use." );
    }

    //loop through each timestep
    //int pixelsTriangulated = 0;
    std::fprintf( stderr, "\n\ntriangulatePixels: There are %lu timesteps.\n", iPixelSet.size() );

    oCalcdPoints.clear();
    oCalcdPoints.resize( iPixelSet.size() );
    for (uint32_t j = 0; j < iPixelSet.size(); ++j)
    { //for each timestep
        int count = 0;
        for (unsigned int i = 0; i < iPixelSet[j].size(); ++i)
        {
            if (iPixelSet[j][i].size() > 0)
            {
                ++count;
            }
        }
        ///@todo this check should really be fixed on the generating size of the pixelset as iPixelSet.size will just come back w/ numCams and empty pixelsets for the cams
        if (iPixelSet[j].size() >= 2  && count >= 2)
        {
            std::vector< cv::Mat > cameras_in_timestep;

            //std::fprintf( stderr, "\n----timestep: %d, # images: %lu \n", j, iPixelSet[j].size() );
            //add the cam matrices for each image in a timestep in the order the images are stored.
            int total_points_in_all_images = 0;
            std::vector< cv::Mat > keypoints;
            int index = 0;
            for (unsigned int i = 0; i < iPixelSet[j].size(); ++i)
            {
                //std::fprintf( stderr, "timestep = %d, cam = %d\n", j, i );
                if (iPixelSet[j][i].size() > 0)
                {
                    //std::fprintf( stderr, "-------number of points in this image %lu\n", iPixelSet[j][i].size() );
                    //for( int idx = 0; idx < iPixelSet[j][i].size(); ++idx){
                    //    std::fprintf( stderr, "---------Point(%f, %f)\n", iPixelSet[j][i][idx].x, iPixelSet[j][i][idx].y );
                    //}
                    cv::Mat tmp( 2, iPixelSet[j][i].size(), CV_64FC1 ); //64 expected by sfm, just converting here to save time later.
                    //cv::RNG rng( 0xFFFFFFFF );
                    //cv::Point noise;
                    for (int x = 0 ; x < tmp.cols; ++x)
                    {
                        //noise.x = rng.uniform(-20,20);
                        //noise.y = rng.uniform(-20,20);
                        tmp.at<double>(0, x) = iPixelSet[j][i][x].x ;//+ noise.x;

                        tmp.at<double>(1, x) = iPixelSet[j][i][x].y ;//+ noise.y;
                        //std::fprintf( stderr, "---------Point(%f, %f)\n", iPixelSet[j][i][x].x, iPixelSet[j][i][x].y );
                    //	std::fprintf( stderr, "-----------tmp(%lf, %lf)\n",  tmp.at<double>(0, x),  tmp.at<double>(1, x) );
                    }

                    keypoints.push_back( tmp );
                    index = iPixelSet[j][i][0].Cam();
                    //std::fprintf( stderr, "-------adding camera %d matrix to list.\n", index );
                    total_points_in_all_images += iPixelSet[j][i].size();

                    cameras_in_timestep.push_back( cam_matrices->M()[index] );
                }
                else
                {
                    std::cerr << "Error processing pixel set in triangulatePixels(): no pixels in timestep" << std::endl;
                }
            }
            //std::fprintf( stderr,
            //              "Calling sfm::trangulatePoints with %d points to triangulate with each other from %lu cameras and %lu matrices.\n",
            //              totalPointsInAllImages, iPixelSet[j].size(), camerasInTimestep.size() );

            //Optimize point locations to solve the M-linear contraints
            std::vector< cv::Mat > corrected_keypoints;

            CorrectTimestepPoints( cameras_in_timestep, keypoints,  corrected_keypoints );


            //--DEBUG-------------------

            /*std::fprintf( stderr, "\n Non-corrected vs Corrected Keypoints for Trianguation:\n");
            for( int k = 0; k < correctedKeypoints.size(); ++k ){
                printMat( keypoints[k] );
                printMat( correctedKeypoints[k] );
            }

            PixelSet correctedPixels;
            std::vector< std::vector< ImagePixel > > timestepPixels;
            for( int c = 0; c < correctedKeypoints.size(); ++c ){
                std::vector< ImagePixel > camImagePixels;
                std::fprintf( stderr, "\nCorrectedKeypoint[%d] Mat:\n", c );
                printMat( correctedKeypoints[c] );
                for( int x = 0 ; x < correctedKeypoints[c].cols; ++x ){

                    ImagePixel tmpPoint( cv::Point2d(correctedKeypoints[c].at< double >(0, x),
                                         correctedKeypoints[c].at< double >(1, x) ),
                                         iPixelSet[j][c][0].cam );
                    camImagePixels.push_back( tmpPoint );
                }
                timestepPixels.push_back( camImagePixels );
            }
            correctedPixels.push_back( timestepPixels ); //only one timestep here, but have to have it in this format.
            std::fprintf( stderr, "Viewing the corrected Keypoints.\n");
            common::convertPointsToLinesForViewing( correctedPixels, iCamMats );*/
            //--END DEBUG ---------------

            std::vector< cv::Point3d > tmp_3d_points;
            cv::Mat tmp_mat;

            //std::fprintf( stderr, "points2d_tmp[0].at<double>(0, 0) = %f\n", correctedKeypoints[0].at< double >(0,0) );
            try
            {
                cv::sfm::triangulatePoints( corrected_keypoints, cameras_in_timestep, tmp_mat );
            }
            catch (cv::Exception ex)
            {
                std::fprintf( stderr, "Exception caught '%s'\n", ex.msg.c_str() );
                assert( false );
            }

            //std::fprintf( stderr,
            //              "----done with sfm::triangulatePoints --> %d 3D points created, mat rows = %d, mat cols = %d, chans = %d\n",
            //              tmpMat.cols, tmpMat.rows, tmpMat.cols, tmpMat.channels());

            //Put Mat into std::vector<cv::Point3d>
            for (int x = 0 ; x < tmp_mat.cols; ++x)
            {
                cv::Point3d tmp_point( tmp_mat.at< double >(0, x), tmp_mat.at< double >(1, x), tmp_mat.at< double >(2,x) );
                //tmpPoint = tmpPoint * 100;
                tmp_3d_points.push_back( tmp_point );
                //std::fprintf( stderr,
                //              "-------- Adding new 3d point to output std::vector (%lf, %lf, %lf)\n",
                //              tmpPoint.x, tmpPoint.y, tmpPoint.z );
            }

            std::vector< cv::Point3d > kalman_corrected_3d_points;
            //Setup Kalman Filters
            if (!ioTracker.IsSetup())
            {
                //We know that the timestep being used to initialize has all of the points because when
                //the user helps with correspondences, we get rid of any timesteps that don't have all
                //the points for all the views.
                ioTracker.Initialize( j, tmp_3d_points );
            }
            else
            {
                fprintf( stderr, "Update Kalman for timestep %d\n", j );
                ioTracker.Next( tmp_3d_points, j, kalman_corrected_3d_points );
            }
            oCalcdPoints[j] = kalman_corrected_3d_points;
            //pixelsTriangulated += tmp3dPoints.size();
        }
        else{
            std::vector< cv::Point3d >  emptySet;
            oCalcdPoints[j] = emptySet;
        }


    }
}


void ConvertProjMatrix( Matrix34d iPrj[], int iCamNum, double iF0)
{
   for (int cm = 0; cm < iCamNum; cm++)
   {
      //double p11, p12, p13, p14, p21, p22, p23, p24, p31, p32, p33, p34;
      Matrix34d pt = iPrj[cm];
      Matrix3d qm, qqt_i, c, c_i, r, k;
      Vector3d qv, tr;


      // convert to Projection matrix which includes f0
      qm = pt.block<3,3>(0,0);
      qv = pt.block<3,1>(0,3);
      
      if(qm.determinant() < 0){
         qm *= -1.0;
         qv *= -1.0;
      }
      tr = -qm.inverse() * qv;
		
      qqt_i = (qm * qm.transpose()).inverse();
      LLT<Matrix3d> chol_d(qqt_i);
      c = chol_d.matrixL().transpose();
      c_i = c.inverse();
      
      r = (c * qm).transpose();
      k = c_i;
      k /= c_i(2,2);
      k(2,2) = iF0;
      iPrj[cm] << k*r.transpose(), -k*r.transpose()*tr;
   }

}

double CorrectTimestepPoints( const std::vector< cv::Mat >& iCamProj,
                              const std::vector< cv::Mat >& iPoints,
                              const std::pair<int, std::set<int> >& iCamIndexesToExclude,
                              std::vector< cv::Mat >& oPoints )
{
    IOFormat clean_fmt( 4, 0, ", ", "\n", "[", "]" );
    int cam_num_all = (int) iCamIndexesToExclude.first - iCamIndexesToExclude.second.size();
    int pt_num = (int) iPoints[0].cols;

    //std::fprintf( stderr, "correctTimestepPoints:: There are %d points and %d cameras.\n", PtNum, CamNumAll );
    MatrixXd ptc[pt_num];
    for (int i = 0; i < pt_num; ++i)
    {
        ptc[i] = Eigen::MatrixXd::Zero(2,cam_num_all );
    }
    Matrix34d proj[cam_num_all];
    MatrixXd pt[pt_num];
    double total_error[pt_num] = {0};


    //Vector3d rp[pt_num];
    MatrixXi idx(pt_num,cam_num_all);

    //Populate Matrices
    int proj_idx = 0;
    for (int i = 0; i < iCamIndexesToExclude.first; ++i)
    {
        if (iCamIndexesToExclude.second.count( i ) == 1)
        {
            continue;
        }
        opensource::cv2eigen( iCamProj[i], proj[proj_idx] );
        ++proj_idx;
        //Proj[i] = iCamProj[i].data();
        std::fprintf( stderr, "\niCamProj[%d]\n", i );
        PrintMat( iCamProj[i] );
        std::cout << "Matrix34d Proj" << std::endl;
        std::cout << proj[i].format(clean_fmt) << std::endl;
    }
    ConvertProjMatrix( proj, cam_num_all, MultipleViewTriangulation::Default_f0 );
    //for( int i = 0; i < CamNumAll; ++i ){
    //	std::cout << "Matrix34d Proj - converted" << std::endl;
    //	std::cout << Proj[i].format(CleanFmt) << std::endl;
    //}

    for (int p = 0; p < pt_num; ++p)
    {
        MatrixXd tmp( 2, cam_num_all );
        for (int i = 0; i < cam_num_all; ++i)
        {
            tmp( 0, i ) = iPoints[i].at< double >( 0, p );
            tmp( 1, i ) = iPoints[i].at< double >( 1, p );
        }
        //std::fprintf( stderr, "\niPoints[%d]\n", i );
        //printMat( iPoints[i] );
        //std::cout << "MatrixXd tmp" << std::endl;
        //std::cout << tmp.format(CleanFmt) << std::endl;
        pt[p] = tmp;
    }


    // resize each point data matrix whose size is 2 x CamNum
    ///@todo is this resize really necessary??
    for (int i = 0; i < pt_num; ++i)
    {
        pt[i].resize( 2, cam_num_all );
        ptc[i].resize( 2, cam_num_all );
        //std::cout << "pt[" << i << "]" << std::endl;
        //std::cout << pt[i].format(CleanFmt) << std::endl;
        //std::cout << "ptc[" << i << "]" << std::endl;
        //std::cout << ptc[i].format(CleanFmt) << std::endl;

    }

    // optimal correction and triangulation
    //std::cerr << "doing optimal correction ..." << std::endl;
    MatrixXd* p_ptc = (MatrixXd*)&ptc;
    Matrix34d* p_proj = (Matrix34d*)&proj;
    MatrixXd* p_pt = (MatrixXd*) &pt;
    bool oc_ret = MultipleViewTriangulation::optimal_correction_all(p_proj,
                                                                   cam_num_all,
                                                                   p_pt,
                                                                   p_ptc,
                                                                   idx,
                                                                   (double*)&total_error,
                                                                   pt_num);

    assert( oc_ret == true );

    for (int p = 0; p < pt_num; ++p)
    {
        //std::cout << "ptc[" << p << "] " << ptc[p] << std::endl;
        std::cout << "MatrixXd ptc[" << p << "] " << std::endl;
        try
        {
            std::cout << ptc[p].format(clean_fmt) << std::endl;
        }
        catch (std::exception& e)
        {
            std::cerr << "Exception catched : " << e.what() << std::endl;
        }
    }

    //for (int i = 0; i < PtNum; ++i)
    //{
     //   std::cout << "pt[" << i << "]" << std::endl;
     //   std::cout << pt[i].format(CleanFmt) << std::endl;
     //   std::cout << "ptc[" << i << "]" << std::endl;
     //   std::cout << ptc[i].format(CleanFmt) << std::endl;

    //}

    //for( int e = 0; e < PtNum; ++e ){
    //	std::fprintf( stderr, "Error for point %d is %lf\n", e, rerr[e] );
    //}

    //std::cout << "MatrixXi idx" << std::endl;
    //std::cout << idx.format(CleanFmt) << std::endl;

    //std::cerr << "done" << std::endl;

    for (int i = 0; i < cam_num_all; ++i)
    {
        cv::Mat tmp( 2, pt_num, CV_64FC1 );
        for (int p = 0; p < pt_num; ++p)
        {
            cv::Mat_<double> tmp_ptc;
            //std::cout << "ptc[" << p << "] " << ptc[p] << std::endl;
            //std::cout << "MatrixXd ptc[p]" << std::endl;
            //std::cout << ptc[p].format(CleanFmt) << std::endl;
            opensource::eigen2cv( ptc[p], tmp_ptc );

            //std::fprintf( stderr, "\tmpPTC i=%d, p=%d\n", i,p );
            //printMat( tmpPTC );
            tmp.at< double >( 0, p ) = tmp_ptc.at<double>( 0, i );
            tmp.at< double >( 1, p ) = tmp_ptc.at<double>( 1, i );
            //std::fprintf( stderr, "\tmp[%d]\n", i );
            //printMat( tmp );
        }
        oPoints.push_back( tmp );
    }

    double error = 0;
    for (int i =0; i < pt_num; ++i)
    {
        error += total_error[i];
    }
    //if( error > 500 ){
    //    std::fprintf( stderr, "Stop here to debug large OC values\n");
    //}
    std::fprintf( stderr, "The total error for this optimiation was %lf\n", error );
    return error;
}

double CorrectTimestepPoints( const std::vector< cv::Mat >& iCamProj,
                              const std::vector< cv::Mat >& iPoints,
                              std::vector< cv::Mat >& oPoints )
{
    IOFormat clean_fmt(4, 0, ", ", "\n", "[", "]");
    int cam_num_all = (int) iCamProj.size();
    int point_num = (int) iPoints[0].cols;

    //std::fprintf( stderr, "correctTimestepPoints:: There are %d points and %d cameras.\n", PtNum, CamNumAll );
    MatrixXd ptc[point_num];
    for (int i = 0; i < point_num; ++i)
    {
        ptc[i] = Eigen::MatrixXd::Zero( 2, cam_num_all );
    }
    Matrix34d proj[cam_num_all];
    MatrixXd pt[point_num];
    double total_error[point_num] = {-1};

    //Vector3d rp[point_num];
    MatrixXi idx(point_num,cam_num_all);

    //Populate Matrices
    for (int i = 0; i < cam_num_all; ++i)
    {
        opensource::cv2eigen( iCamProj[i], proj[i] );
        //Proj[i] = iCamProj[i].data();
    //	std::fprintf( stderr, "\niCamProj[%d]\n", i );
    //	printMat( iCamProj[i] );
    //	std::cout << "Matrix34d Proj" << std::endl;
    //	std::cout << Proj[i].format(CleanFmt) << std::endl;
    }
    ConvertProjMatrix( proj, cam_num_all, MultipleViewTriangulation::Default_f0);
    //for( int i = 0; i < CamNumAll; ++i ){
    //	std::cout << "Matrix34d Proj - converted" << std::endl;
    //	std::cout << Proj[i].format(CleanFmt) << std::endl;
    //}

    for (int p = 0; p < point_num; ++p)
    {
        MatrixXd tmp( 2, cam_num_all );
        for (int i = 0; i < cam_num_all; ++i)
        {
            tmp( 0, i ) = iPoints[i].at< double >( 0, p );
            tmp( 1, i ) = iPoints[i].at< double >( 1, p );
        }
        //std::fprintf( stderr, "\niPoints[%d]\n", i );
        //printMat( iPoints[i] );
        //std::cout << "MatrixXd tmp" << std::endl;
        //std::cout << tmp.format(CleanFmt) << std::endl;
        pt[p] = tmp;
    }


    // resize each point data matrix whose size is 2 x CamNum
    ///@todo is this resize really necessary??
    for (int i = 0; i < point_num; ++i)
    {
        pt[i].resize( 2, cam_num_all );
        ptc[i].resize( 2, cam_num_all );
        //std::cout << "pt[" << i << "]" << std::endl;
        //std::cout << pt[i].format(CleanFmt) << std::endl;
        //std::cout << "ptc[" << i << "]" << std::endl;
        //std::cout << ptc[i].format(CleanFmt) << std::endl;

    }

    // optimal correction and triangulation
    //std::cerr << "doing optimal correction ..." << std::endl;
    MatrixXd* p_ptc = (MatrixXd*)&ptc;
    Matrix34d* p_proj = (Matrix34d*)&proj;
    MatrixXd* p_pt = (MatrixXd*) &pt;
    bool oc_ret = MultipleViewTriangulation::optimal_correction_all(p_proj,
                                                                   cam_num_all,
                                                                   p_pt,
                                                                   p_ptc,
                                                                   idx,
                                                                   (double*)&total_error,
                                                                   point_num);

    assert( oc_ret == true );

    for (int p = 0; p < point_num; ++p)
    {
        //std::cout << "ptc[" << p << "] " << ptc[p] << std::endl;
        std::cout << "MatrixXd ptc[" << p << "] " << std::endl;
        try
        {
            std::cout << ptc[p].format(clean_fmt) << std::endl;
        }
        catch (std::exception& e)
        {
            std::cerr << "Exception catched : " << e.what() << std::endl;
        }
    }

    //for (int i = 0; i < PtNum; ++i)
    //{
     //   std::cout << "pt[" << i << "]" << std::endl;
     //   std::cout << pt[i].format(CleanFmt) << std::endl;
     //   std::cout << "ptc[" << i << "]" << std::endl;
     //   std::cout << ptc[i].format(CleanFmt) << std::endl;

    //}

    //for( int e = 0; e < PtNum; ++e ){
    //	std::fprintf( stderr, "Error for point %d is %lf\n", e, rerr[e] );
    //}

    //std::cout << "MatrixXi idx" << std::endl;
    //std::cout << idx.format(CleanFmt) << std::endl;

    //std::cerr << "done" << std::endl;

    for (int i = 0; i < cam_num_all; ++i)
    {
        cv::Mat tmp( 2, point_num, CV_64FC1 );
        for (int p = 0; p < point_num; ++p)
        {
            cv::Mat_<double> tmp_ptc;
            //std::cout << "ptc[" << p << "] " << ptc[p] << std::endl;
            //std::cout << "MatrixXd ptc[p]" << std::endl;
            //std::cout << ptc[p].format(CleanFmt) << std::endl;
            opensource::eigen2cv( ptc[p], tmp_ptc );

            //std::fprintf( stderr, "\tmpPTC i=%d, p=%d\n", i,p );
            //printMat( tmpPTC );
            tmp.at< double >( 0, p ) = tmp_ptc.at<double>( 0, i );
            tmp.at< double >( 1, p ) = tmp_ptc.at<double>( 1, i );
            //std::fprintf( stderr, "\tmp[%d]\n", i );
            //printMat( tmp );
        }
        oPoints.push_back( tmp );
    }

    double error = 0;
    for (int i =0; i < point_num; ++i)
    {
        error += total_error[i];
    }
    //if( error > 500 ){
    //    std::fprintf( stderr, "Stop here to debug large OC values\n");
    //}
    std::fprintf( stderr, "The total error for this optimiation was %lf\n", error );
    return error;
}

double CorrectTimestepPoints( const std::vector< cv::Mat >& iCamProj,
                              const std::vector< cv::Mat >& iPoints,
                              const std::vector< cv::Mat >& iLastPointLocation,
                              std::vector< cv::Mat >& oPoints )
{
    IOFormat clean_fmt(4, 0, ", ", "\n", "[", "]");
    int cam_num_all = (int) iCamProj.size();
    int point_num = (int) iPoints[0].cols;

    //std::fprintf( stderr, "correctTimestepPoints:: There are %d points and %d cameras.\n", PtNum, CamNumAll );
    MatrixXd ptc[point_num];
    for (int i = 0; i < point_num; ++i)
    {
        ptc[i] = Eigen::MatrixXd::Zero( 2, cam_num_all );
    }
    Matrix34d proj[cam_num_all];
    MatrixXd pt[point_num];
    MatrixXd pt_last_run[point_num];
    double total_error[point_num] = {-1};

    //Vector3d rp[point_num];
    MatrixXi idx( point_num, cam_num_all );

    //Populate Matrices
    for (int i = 0; i < cam_num_all; ++i)
    {
        opensource::cv2eigen( iCamProj[i], proj[i] );
        //Proj[i] = iCamProj[i].data();
    //	std::fprintf( stderr, "\niCamProj[%d]\n", i );
    //	printMat( iCamProj[i] );
    //	std::cout << "Matrix34d Proj" << std::endl;
    //	std::cout << Proj[i].format(CleanFmt) << std::endl;
    }
    ConvertProjMatrix( proj, cam_num_all, MultipleViewTriangulation::Default_f0);
    //for( int i = 0; i < CamNumAll; ++i ){
    //	std::cout << "Matrix34d Proj - converted" << std::endl;
    //	std::cout << Proj[i].format(CleanFmt) << std::endl;
    //}

    for (int p = 0; p < point_num; ++p)
    {
        MatrixXd tmp( 2, cam_num_all );
        MatrixXd tmp_last( 2, cam_num_all );
        for (int i = 0; i < cam_num_all; ++i)
        {
            tmp( 0, i ) = iPoints[i].at< double >( 0, p );
            tmp( 1, i ) = iPoints[i].at< double >( 1, p );

            tmp_last( 0, i ) = iLastPointLocation[i].at< double >( 0, p );
            tmp_last( 1, i ) = iLastPointLocation[i].at< double >( 1, p );
        }
        //std::fprintf( stderr, "\niPoints[%d]\n", i );
        //printMat( iPoints[i] );
        //std::cout << "MatrixXd tmp" << std::endl;
        //std::cout << tmp.format(CleanFmt) << std::endl;
        pt[p] = tmp;
        pt_last_run[p] = tmp_last;
    }


    // resize each point data matrix whose size is 2 x CamNum
    ///@todo is this resize really necessary??
    for (int i = 0; i < point_num; ++i)
    {
        pt[i].resize( 2, cam_num_all );
        pt_last_run[i].resize( 2, cam_num_all );
        ptc[i].resize( 2, cam_num_all );
        //std::cout << "pt[" << i << "]" << std::endl;
        //std::cout << pt[i].format(CleanFmt) << std::endl;
        //std::cout << "ptc[" << i << "]" << std::endl;
        //std::cout << ptc[i].format(CleanFmt) << std::endl;

    }

    // optimal correction and triangulation
    //std::cerr << "doing optimal correction ..." << std::endl;
    MatrixXd* p_ptc = (MatrixXd*)&ptc;
    Matrix34d* p_proj = (Matrix34d*)&proj;
    MatrixXd* p_pt = (MatrixXd*) &pt;
    MatrixXd* p_pt_last_run = (MatrixXd*) &pt_last_run;
    bool oc_ret = MultipleViewTriangulation::optimal_correction_all(p_proj,
                                                                   cam_num_all,
                                                                   p_pt,
                                                                   p_pt_last_run,
                                                                   p_ptc,
                                                                   idx,
                                                                   (double*)&total_error,
                                                                   point_num);

    assert( oc_ret == true );

    for (int p = 0; p < point_num; ++p)
    {
        //std::cout << "ptc[" << p << "] " << ptc[p] << std::endl;
        std::cout << "MatrixXd ptc[" << p << "] " << std::endl;
        try
        {
            std::cout << ptc[p].format(clean_fmt) << std::endl;
        }
        catch (std::exception& e)
        {
            std::cerr << "Exception catched : " << e.what() << std::endl;
        }
    }

    //for (int i = 0; i < PtNum; ++i)
    //{
     //   std::cout << "pt[" << i << "]" << std::endl;
     //   std::cout << pt[i].format(CleanFmt) << std::endl;
     //   std::cout << "ptc[" << i << "]" << std::endl;
     //   std::cout << ptc[i].format(CleanFmt) << std::endl;

    //}

    //for( int e = 0; e < PtNum; ++e ){
    //	std::fprintf( stderr, "Error for point %d is %lf\n", e, rerr[e] );
    //}

    //std::cout << "MatrixXi idx" << std::endl;
    //std::cout << idx.format(CleanFmt) << std::endl;

    //std::cerr << "done" << std::endl;

    for (int i = 0; i < cam_num_all; ++i)
    {
        cv::Mat tmp( 2, point_num, CV_64FC1 );
        for (int p = 0; p < point_num; ++p)
        {
            cv::Mat_<double> tmpPTC;
            //std::cout << "ptc[" << p << "] " << ptc[p] << std::endl;
            //std::cout << "MatrixXd ptc[p]" << std::endl;
            //std::cout << ptc[p].format(CleanFmt) << std::endl;
            opensource::eigen2cv( ptc[p], tmpPTC );

            //std::fprintf( stderr, "\tmpPTC i=%d, p=%d\n", i,p );
            //printMat( tmpPTC );
            tmp.at< double >( 0, p ) = tmpPTC.at<double>( 0, i );
            tmp.at< double >( 1, p ) = tmpPTC.at<double>( 1, i );
            //std::fprintf( stderr, "\tmp[%d]\n", i );
            //printMat( tmp );
        }
        oPoints.push_back( tmp );
    }

    double error = 0;
    for (int i =0; i < point_num; ++i)
    {
        error += total_error[i];
    }
    //if( error > 500 ){
    //    std::fprintf( stderr, "Stop here to debug large OC values\n");
    //}
    std::fprintf( stderr, "The total error for this optimiation was %lf\n", error );
    return error;
}


/*void correctPointsAndTriangulate(){
    Matrix34d proj[CamNumAll];
    MatrixXd pt[PtNum], ptc[PtNum];
    double rerr[PtNum];
    Vector3d rp[PtNum];
    MatrixXi idx(PtNum,CamNumAll);

    // resize each point data matrix whose size is 2 x CamNum
    for (int i = 0; i < PtNum; i++)
    {
      pt[i].resize(2,CamNumAll);
      ptc[i].resize(2,CamNumAll);
    }

    // read projection matrices
    std::cerr << "loading projection matrices ... ";
    load_proj_mat( projmatfile, proj, CamNumAll );
    std::cerr << "done" << std::endl;


    // optimal correction and triangulation
    std::cerr << "doing optimal correction ...";
    MultipleViewTriangulation::optimal_correction_all( proj,
                                                       CamNumAll,
                                                       pt,
                                                       ptc,
                                                       idx,
                                                       rerr,
                                                       PtNum);
    std::cerr << "done" << std::endl;

    std::cerr << "doing triangulation ...";
    MultipleViewTriangulation::triangulation_all(proj,
                                                 CamNumAll,
                                                 ptc,
                                                 rp,
                                                 PtNum,
                                                 idx);
    std::cerr << "done" << std::endl;

    // save data as ply format
    std::string plyfile("plyoutput.txt");
    std::cerr << "saving a ply file ...";
    save_data_as_ply(plyfile, rp, PtNum);
    std::cerr << "done" << std::endl;

    std::cerr << "all done!" << std::endl;
}*/
