#include <dlfcn.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <QPoint>

#include "io/File.h"
#include "MatchingAlgorithmBase.h"

#include "TriangulationPipeline.h"

//using namespace std;
using namespace cv;
using namespace cv::motempl;
using namespace rapidjson;

//Have to initialize this in a cpp file otherwise we get a linking problem
TriangulationPipeline* TriangulationPipeline::mpsInstance = nullptr;

//this is a static function
TriangulationPipeline* TriangulationPipeline::Instance()
{
    if (NULL == TriangulationPipeline::mpsInstance)
    {
        TriangulationPipeline::mpsInstance = new TriangulationPipeline;
    }
    return TriangulationPipeline::mpsInstance;
}

//this is a static function
void TriangulationPipeline::CloseInstance()
{
    if (NULL != TriangulationPipeline::mpsInstance)
    {
        delete TriangulationPipeline::mpsInstance;
        TriangulationPipeline::mpsInstance = NULL;
    }
}
void TriangulationPipeline::Cancel()
{
    mIsCancelled = true;
}


TriangulationPipeline::TriangulationPipeline()
{

}

TriangulationPipeline::~TriangulationPipeline()
{
    Cleanup();
}

int TriangulationPipeline::Factorial(int n)
{
  return (n == 1 || n == 0) ? 1 : Factorial(n - 1) * n;
}

//static function
bool TriangulationPipeline::CompareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
    //Order largest contour area first
    double i = fabs( contourArea(cv::Mat(contour1)) );
    double j = fabs( contourArea(cv::Mat(contour2)) );
    return ( i > j );
}

void TriangulationPipeline::Cleanup()
{
    mIsCancelled = false;
    //mTimestepImages.clear();
    //mBinaryImages.clear();
    DeleteImages();
    DeleteThresholdedImages();
    mPixelsToTrack.clear();
    mFirstMatchTimestep = 0;
}

void TriangulationPipeline::Initialize( const TrackerConfigFile& iConfigFile )
{
    mConfig = iConfigFile;
    Cleanup();

    //Get camera Matrices
    CamMats* cam_matrices = CamMats::Instance();
    cam_matrices->Init( mConfig.calibFileLocation );
    //common::getCameraMatricesFromFile(config.calibFileLocation, camMatrices);


    unsigned long maxCameraNum = cam_matrices->K().size();
    assert( maxCameraNum == mConfig.numCameras );
    std::pair<unsigned long, std::set<unsigned long> > cam_exclude_pair( maxCameraNum, mConfig.camIndexesToExclude);
    mCamIndexesToExcludePair = cam_exclude_pair;
    //Get images -- expect them to be in "camera-timestep.suffix" format where suffix can be bmp, png, etc.
    mImageBufferLength = 0;
    mImageWidth = 0;
    mImageHeight = 0;

    std::vector< std::string > image_locations;
    uint32_t cam_num = 0;
    uint64_t image_count = 0;
    ReadXMLImages( iConfigFile.dataFileLocation, iConfigFile.projDir, image_locations, cam_num, image_count );
    assert( image_locations.size() > 1 );
    std::string test_str = image_locations[0];
    if(test_str.substr(test_str.find_last_of(".") + 1) == "png")
    {
        mImagesRead = ReadImages( mConfig,
                                 mTimestepImages,
                                 TriangulationPipeline::Cancelled );
        std::fprintf( stderr,"Successfully read in %" PRId64 " images.\n", mImagesRead );
    }
    else
    { //if they aren't images, then they must be videos
        PopulateImagesFromVideos( image_locations,
                                  iConfigFile,
                                  mTimestepImages,
                                  TriangulationPipeline::Cancelled );
    }

    if (mTimestepImages.size() > 0 && mTimestepImages[0].size() > 0)
    {
        //mImageBufferLength = CalcRotatedImageBufferSize( mTimestepImages[0][0] );
        mImageWidth = mTimestepImages[0][0].cols;
        mImageHeight = mTimestepImages[0][0].rows;
    }

    int maxNumTimesteps = mTimestepImages.size();
    //Gotta clean up bounds, otherwise we might get some out of index access violations.
    if (mConfig.vizCameraPose == 0 || mConfig.vizCameraPose > maxNumTimesteps)
    {
        mConfig.vizCameraPose = maxNumTimesteps;
    }
    else if (mConfig.vizCameraPose < 0)
    {
        mConfig.vizCameraPose = 0;
    }

    if (mConfig.vizPointCorrespondences == 0 || mConfig.vizPointCorrespondences > maxNumTimesteps)
    {
        mConfig.vizPointCorrespondences = maxNumTimesteps;
    }
    else if (mConfig.vizPointCorrespondences < 0)
    {
        mConfig.vizPointCorrespondences = 0;
    }

    if (mConfig.vizThresholds == 0 || mConfig.vizThresholds > maxNumTimesteps)
    {
        mConfig.vizThresholds = maxNumTimesteps;
    }
    else if (mConfig.vizThresholds < 0)
    {
        mConfig.vizThresholds = 0;
    }

    if (mConfig.vizUndistortedImages == 0 || mConfig.vizUndistortedImages > maxNumTimesteps)
    {
        mConfig.vizUndistortedImages = maxNumTimesteps;
    }
    else if (mConfig.vizUndistortedImages < 0)
    {
        mConfig.vizUndistortedImages = 0;
    }
    mIsInit = true;
}


void TriangulationPipeline::Undistort()
{
    //first need to undistort the images that I've been given...
    if( mConfig.undistortImagesBool ){
        UndistortImages( mTimestepImages,
                         TriangulationPipeline::Cancelled,
                         mCamIndexesToExcludePair,
                         mConfig.writeUndistImages );
        //common::undistortImages( mTimestepImagesBuffered, camMatrices.K, camMatrices.distortion, writeUndistImages );

        for( int64_t j = 0; j < mConfig.vizUndistortedImages; ++j ){
            for( uint32_t cam = 0; cam < mTimestepImages[j].size(); ++cam){
                if (!mIsCancelled)
                {
                    imshow( "Undistorted Image", mTimestepImages[j][cam] );
                    waitKey(0);
                }
            }
        }
    }
    return;
}

void TriangulationPipeline::Threshold()
{
    //ImageSet mTimestepImagesBuffered = mTimestepImages;




    //get Affine transforms for images in the set
    //fprintf( stderr, "Getting Affine Transforms...\n" );
    std::vector< std::map< std::string, cv::Mat > > affineTransforms;
    //getAffineTransforms( mTimestepImages, affineTransforms );
    //fprintf( stderr, " ... got all transforms.\n" );

    //ImageSet  mBinaryImages;
    //int dist1 = 0;
    //int dist2 = hRightbound - hLeftbound;
    //for( int i = 360 - hRightbound; i < hLeftbound; ++i, ++dist1);

    //Leftbound = left bound, upper bound = right bound with a circular hue where the values decrease from left to right.
    fprintf( stderr, "LeftBound: %d , RightBound: %d\n", mConfig.hLeftbound, mConfig.hRightbound);

    ThresholdImages( mTimestepImages,
                     mBinaryImages,
                     TriangulationPipeline::Cancelled,
                     Scalar(mConfig.hLeftbound, mConfig.sLeftbound, mConfig.vLeftbound),
                     Scalar(mConfig.hRightbound, mConfig.sRightbound, mConfig.vRightbound),
                     mConfig.noiseFilterSize,
                     mConfig.noiseIterations,
                     mConfig.noiseThreshold);


    if( mConfig.vizThresholds ){
        //DEBUG--------------------
        fprintf( stderr, "There are %lu total timesteps. \n", mTimestepImages.size() );

        for( int64_t j=0; j < mConfig.vizThresholds; ++j )
        {
            fprintf( stderr, "Timestep %lu has %ld total images. \n\n", j, mTimestepImages[j].size() );
            for( uint32_t i=0; i < mTimestepImages[j].size(); ++i )
            {
                if (mIsCancelled)
                {
                    return;
                }
                fprintf( stderr, "---Timestep: %ld, Image: %d \n", j, i );
                Mat lOld = mTimestepImages[j][i].clone();
                Mat lNew = mBinaryImages[j][i].clone();

                std::vector< std::vector< cv::Point > > contours;
                std::vector< cv::Vec4i > hierarchy;

                cv::findContours( lNew, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );

                lNew = cv::Mat::zeros( cv::Size( lNew.cols, lNew.rows ), lNew.type() );
                // sort contours biggest first
                std::sort(contours.begin(), contours.end(), CompareContourAreas);

                std::vector< std::vector<cv::Point> > hulls(mConfig.maxNumPoints);
                for( uint32_t i = 0; i < contours.size() && i < mConfig.maxNumPoints; ++i ){
                    // Initialize the contour with the convex hull points
                    cv::convexHull(cv::Mat(contours[i]), hulls[i]);

                    // And draw any found contours, filled
                    cv::drawContours(lNew, hulls, i/*-1allcontours*/, 255, CV_FILLED);

                }
                try{
                    ViewOldAndNewImage( lOld, lNew );
                } catch( const std::exception& e){
                    cerr << "Problem with viewOldAndNewImage: " << e.what() << endl;
                }
            }
        }
        //END DEBUG--------------------
    }
    return;
}

void TriangulationPipeline::ClusterPixels()
{
    PixelClusterSet clustered_pixels;
    //sortPixelsIntoGroups( interestingPixels, clustered_pixels);
    GetPixelClusters( mBinaryImages,
                      TriangulationPipeline::Cancelled,
                      mCamIndexesToExcludePair,
                      mConfig.maxNumPoints,
                      clustered_pixels );

    uint64_t num_clustered_pixels = 0;
    for( uint32_t t = 0; t< clustered_pixels.size(); ++t)
    {
        for( uint32_t c =0; c < clustered_pixels[t].size(); ++c ) num_clustered_pixels += clustered_pixels[t][c].size();
    }
    fprintf( stderr, "\n\n +++ image timesteps:%ld, clustered_pixels:%ld\n", mTimestepImages.size(), num_clustered_pixels );


    //Get rid of images that don't have enough shared pixels across timestep, and get rid of timesteps without enough images (min 2)
    if( !mConfig.allowHiddenPointMarking )
    {
        ShrinkClustersToMaxSubset( clustered_pixels, mConfig.minNumCamsTriangulation );
    }

   //Use a gaussian mean for each cluster to determine center
//    PixelSet mPixelsToTrack;
    ConvertClustersToPixels( clustered_pixels, mPixelsToTrack );

    uint64 num_mPixelsToTrack = 0;
    for( uint32_t t = 0; t< mPixelsToTrack.size(); ++t){
        for( uint32_t c =0; c < mPixelsToTrack[t].size(); ++c ) num_mPixelsToTrack += mPixelsToTrack[t][c].size();
    }
    fprintf(stderr, "\n\n +++ image timesteps:%ld, mPixelsToTrack:%ld\n", mTimestepImages.size(), num_mPixelsToTrack);

    if( mConfig.vizCameraPose ){
        CamMats* cam_matrices = CamMats::Instance();
        cam_matrices->Init( mConfig.calibFileLocation );
        ConvertPointsToLinesForViewing( mPixelsToTrack, cam_matrices, mConfig.vizCameraPose );
    }
    return ;
}

bool TriangulationPipeline::RunUserMatchingHelper()
{
    return !mSkipUserInput;
}

bool TriangulationPipeline::FirstMatchingTimestep()
{
    ///@todo :TODO: Show images from first timestep and have the use select the coordinates for the points that match between images.
      ///This is needed for maxNumPoints > 5 otherwise we have to calculate (maxNumPoints!)^(numCAm -1) combinations
      ///Instead of a hardcoding at < 5, there could just be a quick calc of the number of combinations given the params of the system ( i.e. 3 points with 6 cams = ~8000 combos -->calculate combinations,
      /// where as ( 5 points with 6 cams = ~24 billion combos --> don't calculate )
      ///
    //Checking to make sure the first images from all the cameras show all the points to be tracked.
    //uint32_t pointCount = 0;
    //for( uint32_t i =0; i< first_timestep_with_all_points[0].size(); ++i ){
    //    if( first_timestep_with_all_points[0][i].size() == maxNumPoints ) ++pointCount;
    //}
    //int comboNum = pow(Factorial( maxNumPoints ), numCameras -1); //total number of possible point combinations across multiple cameras with no repeats.

    std::vector< std::vector< std::vector< int32_t > > > best_fit;
    std::vector< double > best_fit_errors;
    PixelSet first_timestep_with_all_points;

    uint64_t first_match_timestep = 0;
    for (; first_match_timestep < mPixelsToTrack.size(); ++first_match_timestep)
    {
        bool hasAllPoints = true;
        if (!mConfig.allowHiddenPointMarking)
        {
            for (uint32_t i =0; i < mPixelsToTrack[first_match_timestep].size(); ++i)
            {
                if (mConfig.camIndexesToExclude.count( i ) == 1)
                {
                    continue;
                }
                if (mPixelsToTrack[first_match_timestep][i].size() != mConfig.maxNumPoints)
                {
                    hasAllPoints = false;
                }
            }
        }
        if (hasAllPoints && mPixelsToTrack[first_match_timestep].size() > 0)
        {
            break;
        }
    }
    mFirstMatchTimestep = first_match_timestep;

    //force user to select points manually
    std::vector< unsigned long > timestepsToCheck;
    timestepsToCheck.push_back( first_match_timestep );

    if ( true /*pointCount != numCameras  || comboNum > 1000000*/)
    {


        mSkipUserInput = mConfig.tryToUseSavedMarkedPoints;
        if (mConfig.tryToUseSavedMarkedPoints)
        {
            ReadMarkedPointsFromFile( mConfig.triangulationOutput, mMarkedPoints, best_fit );
            if (mMarkedPoints.size() == mConfig.numCameras && best_fit.size() > 0)
            {
                for (uint32_t i =0; i < mMarkedPoints.size(); ++i)
                {
                    if (mConfig.camIndexesToExclude.count( i ) == 1)
                    {
                        continue;
                    }
                    if (mMarkedPoints[i].size() != mConfig.maxNumPoints)
                    {
                        mSkipUserInput = false;
                        break;
                    }
                }
            }
            else
            {
                mSkipUserInput = false;
            }
        }
    }
    else
    {
        time_t start = std::time(nullptr);
        FindCorrespondingPoints(   mPixelsToTrack,
                                      best_fit,
                                      best_fit_errors );

        time_t stop = std::time(nullptr);

        std::fprintf( stderr, "%lu seconds to calculate all optimal corrections.\n", stop -start);
    }
}

bool TriangulationPipeline::PointCorrespondence()
{

    void* lib_hndl = dlopen("libMatchAlgorithm.so", RTLD_NOW);
    if (NULL == lib_hndl)
    {
        std::cerr << "Could not load the requested matching algorithm library." << std::endl;
        std::cerr << dlerror() << std::endl;
        return false;
    }
    void* match_maker = dlsym( lib_hndl, "MatchMaker" );
    if (NULL == match_maker)
    {
        std::cerr << "The MatchMaker function does not exist in the supplied library." << std::endl;
        return false;
    }

    //:TRICKY: The following three lines can look very confusing. The reason is that we aren't
    // allowed to cast a void* to a function*. So in order to do that, we instead have to do this
    // referencing and dereferencing workaround.
    MatchingAlgorithmBase* (*match_maker_ptr)();
    *(void**)(&match_maker_ptr) = match_maker;
    MatchingAlgorithmBase* match_alg_ptr = match_maker_ptr();

    bool run_successfully = false;
    match_alg_ptr->MatchPoints( mFirstMatchTimestep,
                                mPixelsToTrack,
                                mConfig,
                                //mTimestepImages,
                                mCamIndexesToExcludePair,
                                TriangulationPipeline::Cancelled,
                                mMarkedPoints,
                                run_successfully
                                //bestFit,
                                //bestFitErrors,
                                //first_timestep_with_all_points
                                );
    if (!run_successfully)
    {
        std::cerr << "Error with algorithm, no ordered points returned." << std::endl;
        return false;
    }
    dlclose( lib_hndl );

    //DEBUG ---part2-------------
    if( mConfig.vizPointCorrespondences ){
        for( int j = 0; j < mConfig.vizPointCorrespondences; ++j){
            if( mPixelsToTrack[j].size() == 0 ){
                continue;
            }

            //Draw lines between subsequent matching points in subsequent images.
            fprintf(stderr, "Draw lines between subsequent matching points in subsequent images.\n");

            cv::Mat tmp_line_mat = cv::Mat::zeros( mTimestepImages[j][0].cols, mTimestepImages[j][0].rows, mTimestepImages[j][0].type() );
            std::vector<SmtPixel> set1;
            std::vector<SmtPixel> set2;

            if( mPixelsToTrack[j].size() == 0 ){
                fprintf( stderr, "****ERROR? No orderedPixels in this timestep...\n" );
                continue;
            }

            //Make sure we line up the pixel sets with the camera images they go with
            set1 = mPixelsToTrack[j][0];
            for( uint32_t idx = 1; idx < mPixelsToTrack[j].size(); ++idx )
            {
                if (mIsCancelled)
                {
                    return true;
                }
                set2 = mPixelsToTrack[j][idx];
                //double angle = 0;

                cv::Mat imageRot;
                cv::Mat imageRot2;
                for( uint32_t i = 0; i < mTimestepImages[j].size(); ++i ){
                    if(  set1[0].Cam() == mTimestepImages[j][i].Cam() ){
                        //angle = camAngles[ set1[0].cam ];
                        //imageRot = rotate(mTimestepImagesBuffered[j][i], angle);
                        imageRot = mTimestepImages[j][i](Rect((mImageBufferLength-mImageWidth)/2, (mImageBufferLength-mImageHeight)/2, mImageWidth, mImageHeight)).clone();
                    }
                    else if(  set2[0].Cam() == mTimestepImages[j][i].Cam() ){
                        //angle = camAngles[ set2[0].cam ];
                        //imageRot2 = rotate(mTimestepImages[j][i], angle);
                        imageRot2 = mTimestepImages[j][i](Rect((mImageBufferLength-mImageWidth)/2, (mImageBufferLength-mImageHeight)/2, mImageWidth, mImageHeight)).clone();
                    }
                    //fprintf(stderr, "set1[0].cam:%d, set2[0].cam:%d, mTimestepImagesBuffered[%d][%d].cam:%d\n", set1[0].cam, set2[0].cam,j,i, mTimestepImagesBuffered[j][i].cam );
                }
                //fprintf(stderr, "Right before addWeighted. imageRot.cols:%d, imageRot2.cols:%d \n", imageRot.cols, imageRot2.cols);
                addWeighted( imageRot, 1.0, imageRot2, 1.0, 0.0, tmp_line_mat );
            }
            if( set1.size() == set2.size() ){
                for( uint32_t p = 0; p < set1.size(); ++p){
                    //fprintf(stderr, "P%d(%lf,%lf) matched with P%d(%lf,%lf)  -- dist:%lf\n",p, set1[p].x,
                    //                                                                        set1[p].y, p,set2[p].x, set2[p].y, opensource::getShortestDistance( set1[p], set2[p] ));
                    line( tmp_line_mat, set1[p], set2[p], cv::Scalar(0, 255, 0), 1);
                    circle( tmp_line_mat,
                            set1[p],
                            1, //radius
                            cv::Scalar(0, 255, 255),
                            1 //thickness
                            );
                    circle( tmp_line_mat,
                            set2[p],
                            1, //radius
                            Scalar(0, 255, 255),
                            1 //thickness
                            );
                }
                //imshow("connecting lines", tmp_line_mat);
                //waitKey(0);
            }

        }
    }

    return true;
}

void TriangulationPipeline::Triangulate()
{
    //Now try using triangulatePoints to get the world coordinates for my clusters. If that doesn't work, write my own iterative method? Polynomial-Absolute?
    std::vector< std::vector< cv::Point3d > > calculatedPoints;

    int pixelsTriangulated = TriangulatePixels( mPixelsToTrack, calculatedPoints );
    //int pixelsTriangulated = common::triangulatePixelsWithKalman( mPixelsToTrack, camMatrices, camIndexesToExcludePair, point_tracker, maxNumPoints, calculatedPoints );

    //Try to smooth out any very noisy 3d point calculations.
    MultiPointSmoother3d point_tracker( mConfig.maxNumPoints );
    for( uint32_t j = mFirstMatchTimestep; j < calculatedPoints.size(); ++j ){
        if (mIsCancelled)
        {
            return;
        }
        std::vector< cv::Point3d > kalmanCorrected3dPoints;
        //Setup Kalman Filters
        if( !point_tracker.IsSetup() ){
            //We know that the timestep being used to initialize has all of the points because when the user helps with correspondences,
            //we get rid of any timesteps that don't have all the points for all the views.
            point_tracker.Initialize(mFirstMatchTimestep, calculatedPoints[j] );
        }
        fprintf(stderr, "Update Kalman for timestep %d\n", j);
        point_tracker.Next( calculatedPoints[j], j, kalmanCorrected3dPoints );
        
        calculatedPoints[j] = kalmanCorrected3dPoints;
    }

    cv::Point3d mean(0,0,0);
    for( uint32_t j = mFirstMatchTimestep; j < calculatedPoints.size(); ++j )
    {
        if (mIsCancelled)
        {
            return;
        }
        pixelsTriangulated += calculatedPoints[j].size();
        for( uint32_t i = 0; i < calculatedPoints[j].size(); ++i ){
            mean.x += calculatedPoints[j][i].x;
            mean.y += calculatedPoints[j][i].y;
            mean.z += calculatedPoints[j][i].z;
        }
    }
    mean.x = mean.x / pixelsTriangulated;
    mean.y = mean.y / pixelsTriangulated;
    mean.z = mean.z / pixelsTriangulated;

    cv::FileStorage fs( mConfig.triangulationOutput, cv::FileStorage::WRITE );
    WriteTriangulationInfoToFile( fs, "SimplePointTracking", calculatedPoints, mConfig.maxNumPoints);
    fs.release();

    //int pixelsTriangulated = common::triangulatePixelsWithHistory( orderedPixelsToTrack, camMatrices.extrinsic, calculatedPoints );

    fprintf( stderr, "We triangulated %d points\n", pixelsTriangulated );

    return;
}

int TriangulationPipeline::Run( const std::string& iConfigFile )
{
    TrackerConfigFile config_struct;
    ProcessConfigFile( iConfigFile, config_struct );

    return Run( config_struct );
}

int TriangulationPipeline::Run( const TrackerConfigFile& iConfig )
{
    /* This function looks a little funny because I didn't want to next a bunch of ifs. The elses are needed because
     * it could be the case that the state of cancelled changes back and forth quickly. If this happens I would want
     * the function to fully exit and not start again halfway through. */
    mIsCancelled = false;
    const int cancelled = -1;
    Initialize( iConfig );

    if (!Cancelled())
    {
        Undistort();
    }
    else
    {
        return cancelled;
    }

    if (!Cancelled())
    {
        if( mConfig.maskFileLocation != "" )
        {
//            CamMats* cam_matrices = CamMats::Instance();
//            cam_matrices->Init( mConfig.calibFileLocation );
//            ImageSet background; //when the images get read in we already remove background so, we just pass in this empty set.
//            opensource::EPVHInterface::WriteVisualHullPolygons( mConfig.triangulationOutput,
//                                                            mTimestepImages,
//                                                            background,
//                                                            cam_matrices );
        }
    }

    if (!Cancelled())
    {
        Threshold();
    }
    else
    {
        return cancelled;
    }

    if (!Cancelled())
    {
        ClusterPixels();
    }
    else
    {
        return cancelled;
    }

    if (!Cancelled())
    {
        FirstMatchingTimestep();
    }
    else
    {
        return -1;
    }


    std::vector< std::vector< std::vector< int32_t > > > best_fit;
    ReadMarkedPointsFromFile( mConfig.triangulationOutput, mMarkedPoints, best_fit );

    if (mMarkedPoints.size() == 0)
    {
        std::cerr << "There was no marked points file available. Run the GUI application in order to fill in the marked points, or create the file by hand.\n" << std::endl;
        return -3;
    }


    if (!Cancelled())
    {
        bool lib_load_succ = PointCorrespondence();
        if (!lib_load_succ)
        {
            std::cerr << "There was an error with the Matching algorithm library." << std::endl;
            return -2;
        }
    }
    else
    {
        return cancelled;
    }
    time_t startFull = std::time(nullptr);

    if (!Cancelled())
    {
        Triangulate();
    }
    else
    {
        return cancelled;
    }

    time_t endFull = std::time(nullptr);
    std::fprintf( stderr, "%lu seconds to calculate all optimal corrections.\n", endFull - startFull);

    return 0;
}

//void TriangulationPipeline::CreateVisualHull(const std::string& iConfigFile)
//{
//    if (!mIsInit)
//    {
//        TrackerConfigFile config_struct;
//        ProcessConfigFile( iConfigFile, config_struct );
//        Initialize(config_struct);
//    }
//    if( mConfig.maskFileLocation != "" )
//    {
//        CamMats* cam_matrices = CamMats::Instance();
//        ImageSet background; //when the images get read in we already remove background so, we just pass in this empty set.
//        opensource::EPVHInterface::WriteVisualHullPolygons( mConfig.triangulationOutput,
//                                                        mTimestepImages,
//                                                        background,
//                                                        cam_matrices );
//    }
//}

bool TriangulationPipeline::Cancelled()
{
    if ( NULL != TriangulationPipeline::mpsInstance)
    {
        return TriangulationPipeline::mpsInstance->mIsCancelled;
    }
    return false;
}

std::vector< SmtImage > TriangulationPipeline::Images()
{
    if (mTimestepImages.size() > 0 && mFirstMatchTimestep < mTimestepImages.size())
    {
        return mTimestepImages[mFirstMatchTimestep];
    }
    std::vector<SmtImage> tmp_vector;
    return tmp_vector;
}

int TriangulationPipeline::TimestepUsed()
{
    return mFirstMatchTimestep;
}

std::vector< std::vector< SmtPixel > > TriangulationPipeline::MarkersFound()
{
    if (mPixelsToTrack.size() > 0 && mFirstMatchTimestep < mPixelsToTrack.size())
    {
        return mPixelsToTrack[mFirstMatchTimestep];
    }
    std::vector< std::vector< SmtPixel > > tmp_vector;
    return tmp_vector;
}

std::pair<unsigned long, std::set<unsigned long> > TriangulationPipeline::CamsToExclude()
{
    return mCamIndexesToExcludePair;
}

int TriangulationPipeline::NumMarkersToFind()
{
    return mConfig.maxNumPoints;
}

void TriangulationPipeline::GetNewMarkedPointsInfo()
{
    std::vector< std::vector< std::vector< int32_t > > > best_fit;

    ReadMarkedPointsFromFile( mConfig.triangulationOutput, mMarkedPoints, best_fit );
}
