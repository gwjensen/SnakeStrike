#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <iostream>

#include "io/File.h"
#include "SmtPixel.h"
#include "CamMats.h"
#include "PixelCluster.h"

#include "SmtImage.h"

//static function
bool SmtImage::CompareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 )
{
    //Order largest contour area first
    double i = fabs( contourArea(cv::Mat(contour1)) );
    double j = fabs( contourArea(cv::Mat(contour2)) );
    return ( i > j );
}

SmtImage::SmtImage(const std::string    iLocation,
                   const int            iCam,
                   const int            iTimestep,
                   const cv::Mat&       iImage,
                   const bool           iIsRGB)
    :cv::Mat( iImage.clone() ),
      mLocation(iLocation),
      mCam(iCam),
      mTimestep(iTimestep),
      mIsRGB(iIsRGB)
{
    mFileSuffix  = GetFileExt( iLocation );
}

SmtImage::SmtImage(const std::string    iLocation,
                   const int            iCam,
                   const int            iTimestep)
    : mLocation(iLocation),
      mCam(iCam),
      mTimestep(iTimestep)
{
    mFileSuffix  = GetFileExt( iLocation );
    Load( iLocation );
}


SmtImage::SmtImage( const SmtImage&  iFrom,
                    const cv::Mat&   iNew,
                    const bool iIsRGB )
    :cv::Mat( iNew.clone() )
{
    mLocation  = iFrom.Location();
    mCam       = iFrom.Cam();
    mTimestep  = iFrom.Timestep();
    mFileSuffix = iFrom.FileSuffix();
    mIsRGB = iIsRGB;
}

SmtImage::SmtImage( const SmtImage& iFrom )
    :cv::Mat( iFrom.clone() )
{
    mLocation   = iFrom.Location();
    mCam        = iFrom.Cam();
    mTimestep   = iFrom.Timestep();
    mFileSuffix = iFrom.FileSuffix();
    mIsRGB      = iFrom.RGB();
}

SmtImage::~SmtImage()
{

}

SmtImage& SmtImage::operator=(const SmtImage& iImage) // copy assignment
{
    if (this != &iImage)
    { // self-assignment check expected
        mLocation = iImage.mLocation;
        mFileSuffix = iImage.mFileSuffix;
        mCam =  iImage.mCam;
        mTimestep = iImage.mTimestep;
        mIsRGB = iImage.RGB();
        Mat::operator=( iImage.clone() );
    }
    return *this;
}

SmtImage& SmtImage::operator=(const cv::Mat& iMat) // copy assignment
{
    if (this != &iMat)
    { // self-assignment check expected
        Mat::operator=( iMat.clone() );
    }
    return *this;
}

bool SmtImage::Save( const std::string& iPath )
{
    std::vector<int> params;
    params.push_back(CV_IMWRITE_PNG_COMPRESSION);

    //:NOTE:Set to zero because compression slows the system down and we
    //become CPU limited not File IO limited
    params.push_back( 0 );//0-9 compression level

    mLocation = iPath;
    mFileSuffix = GetFileExt( iPath );

    //:NOTE: opencv assumes the image is in BGR, so it converts it to RGB before saving. Since
    // we assume the image is in RGB, we have to do this extra conversion.
    cv::cvtColor(*this, *this, CV_RGB2BGR);

    return cv::imwrite( iPath, *this, params );
}

bool SmtImage::WriteToPipe( FILE* iPipe )
{
    //cv::Mat img = cv::Mat(cv::Size(videoWidth, videoHeight), CV_8UC3, videoBuffer); // videoBuffer directly from libav transcode to memory
    //cv::namedWindow("Display window", cv::WINDOW_NORMAL); // Create a window for display.
    //cv::imshow("Display window", img); // Show our image inside it.

    // encode image to jpeg
    //std::vector<uchar> buff; //buffer for coding
    //std::vector<int> param(2);
    //param[0] = CV_IMWRITE_PNG_COMPRESSION;
    //param[1] = 0; // 0-9 compression level 0, is none
    //cv::imencode(".raw", *this, buff);
    fprintf(stdout, "Image data size: %lu bytes (%d kB)\n", this->total()*3, (int) (this->total()*3 / 1024));

    // write encoded image to pipe/fifo
    unsigned int written_data = fwrite(this->data, this->total()*3, 1, iPipe);
    if ( 1 != written_data )
    {
        fprintf(stderr, "Error write image data to fifo!\n");
        return false;
    }
    return true;
}

bool SmtImage::Load( const std::string& iPath )
{
    if (!boost::filesystem::exists( iPath ))
    {
        std::cerr << "Can't find image file at location '" << iPath << "'." << std::endl;
        return false;
    }
    cv::Mat tmp = cv::imread( iPath, CV_LOAD_IMAGE_COLOR );

    //:NOTE:opencv always loads the image BGR format, i.e. it assumes the image was saved in RGB
    //format and converts it to BGR
    cv::cvtColor(tmp, tmp, CV_BGR2RGB);
    //tmp.copyTo(*this);
    *this = tmp.clone();
    mIsRGB = true;
    return true;
}

void SmtImage::CopyAttributes( const SmtImage& iImage )
{
    mLocation = iImage.mLocation;
    mFileSuffix = iImage.mFileSuffix;
    mCam =  iImage.mCam;
    mTimestep = iImage.mTimestep;
}

void SmtImage::GammaCorrection( float fGamma )
{
    unsigned char lut[256];
    for (int i = 0; i < 256; i++)
    {
        lut[i] = cv::saturate_cast<uchar>( pow( (float)(i / 255.0), fGamma ) * 255.0f );
    }

    const int channels = this->channels();

    switch (channels)
    {
        case 1:
        {
            cv::MatIterator_< uchar > it, end;
            for (it = this->begin< uchar >(), end = this->end< uchar >(); it != end; ++it){
                *it = lut[(*it)];
            }
            break;
        }
        case 3:
        {
            cv::MatIterator_< cv::Vec3b > it, end;
            for (it = this->begin< cv::Vec3b >(), end = this->end< cv::Vec3b >(); it != end; ++it){
                (*it)[0] = lut[ (*it)[0] ];
                (*it)[1] = lut[ (*it)[1] ];
                (*it)[2] = lut[ (*it)[2] ];
            }
            break;
        }
    }
 }



void SmtImage::BlobDetection()
{
    // Read image
    //Mat im = imread( iFilename, IMREAD_GRAYSCALE );

    // Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;
    cv::Mat im;
    cv::cvtColor(*this, im, CV_BGR2GRAY);

    // Change thresholds
    params.minThreshold = 10;
    params.maxThreshold = 255;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 10;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.8;

    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.87;

    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.8;


    // Storage for blobs
    std::vector<cv::KeyPoint> keypoints;


    // Set up detector with params
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    // Detect blobs
    detector->detect( im, keypoints);


    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures
    // the size of the circle corresponds to the size of blob

    cv::Mat im_with_keypoints;
    drawKeypoints( im,
                   keypoints,
                   im_with_keypoints,
                   cv::Scalar(0,0,255),
                   cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    // Show blobs
    cv::imshow("blob keypoints", im_with_keypoints );
    cv::waitKey(0);
}


void SmtImage::Threshold( const cv::Scalar& iLowerb,
                        const cv::Scalar& iUpperb,
                        const SmtImageThresholdInfo& iInfo)
{
    cv::Mat tmp_mat;
    Threshold( iInfo.noise_filter_size,
               iInfo.noise_iterations,
               iInfo.max_num_points,
               iInfo.noise_threshold,
               iLowerb,
               iUpperb,
               tmp_mat);
    *this = tmp_mat.clone();
    ///////////////////////////////////////////////////////////////////////////////////
    /*
    //cv::Mat gammaCorrectedImage;
    // Convert input image to HSV
    //cv::Mat tmp_image;
    //cv::medianBlur( iImage, tmp_image, 5 ); //smoothes out the image with 5x5 filter
    //GammaCorrection(tmp_image, gammaCorrectedImage, .5);
   // cv::Mat image_copy = this->clone();

    cv::Mat hsv_image;
    if (mIsRGB)
    {
        cvtColor( *this, hsv_image, cv::COLOR_RGB2HSV );
    }
    else
    {
        cvtColor( *this, hsv_image, cv::COLOR_BGR2HSV );
    }
    cv::Mat tmp_mask;
    //cv::inRange( hsv_image, iLowerb, iUpperb, tmpMask ); //ex iLowerb = cv::Scalar(0,125,0)
    //viewOldAndNewImage( iImage, tmpMask);

    if (iLowerb[0] < iUpperb[0])
    { //logic to handle red values, i.e. values that cross over zero on the color wheel
        cv::Mat tmp_bound;
        cv::inRange( hsv_image,
                    cv::Scalar(0, iLowerb[1], iLowerb[2]),
                    cv::Scalar(iLowerb[0]/2, iUpperb[1], iUpperb[2]),
                    tmp_bound ); //ex iLeftb = cv::Scalar(0,125,0)
        cv::inRange( hsv_image,
                    cv::Scalar(iUpperb[0]/2, iLowerb[1], iLowerb[2]),
                    cv::Scalar(360/2, iUpperb[1], iUpperb[2]),
                    tmp_mask ); //ex iLeftb = cv::Scalar(0,125,0)
        cv::addWeighted( tmp_bound, 1.0, tmp_mask, 1.0, 0.0, tmp_mask );
    }
    else
    {
        //Have to swap left and right hue values as inRange counts upwards
        cv::inRange( hsv_image,
                    cv::Scalar( iUpperb[0]/2, iLowerb[1], iLowerb[2] ),
                    cv::Scalar( iLowerb[0]/2, iUpperb[1], iUpperb[2] ),
                    tmp_mask ); //ex iLeftb = cv::Scalar(0,125,0)
    }

    //Filter out parts of the image with high intensity light reflections (currently disabled)
    cv::Scalar mask_others_lowerb( 0, 0, 180);
    cv::Scalar mask_others_upperb( 0, 0, 180);
    cv::Mat tmp_mask_other;

    cv::inRange( hsv_image, mask_others_lowerb, mask_others_upperb, tmp_mask_other );
    //cv::inRange( iImage, cv::Scalar( 230,230,230 ), cv::Scalar(255,255,255), tmpMaskOther );
    //viewOldAndNewImage( iImage, tmpMaskOther);
    //viewOldAndNewImage( iImage, tmpMask );

    cv::bitwise_not( tmp_mask_other, tmp_mask_other );

    cv::Mat tmp;

    tmp_mask.copyTo( tmp, tmp_mask_other );
    //tmpMask.copyTo( tmp );
    //viewOldAndNewImage( iImage, tmp );

    //Noise reduction code ---
    ///@todo should be user specified as it is very variable on the data collection how much color noise there is....
    //These thresholding operations are fine as we are currently dealing with
    // a 1-d mask for the color we were searching for
    int noise_filter_size = iNoiseThresholdInfo[0];
    int noise_iterations = iNoiseThresholdInfo[1];
    int noise_threshold = iNoiseThresholdInfo[2];
    for (int i= 0; i < noise_iterations; ++i)
    {
        //cv::imshow("before", tmp);

        cv::blur( tmp, *this, cv::Size(noise_filter_size, noise_filter_size), cv::Point(-1, -1) );
        cv::threshold( *this, tmp, noise_threshold, 255, cv::THRESH_BINARY );

        //cv::imshow("after", tmp);
        //cv::waitKey(0);
    }
    //cv::blur( tmp, oImage, cv::Size(3, 3), cv::Point(-1, -1) );

    //second pass to get rid of small clusters missed by first pass...
   //cv::threshold( oImage, tmp, 100, 255, cv::THRESH_BINARY );
    mIsBinary = true;
    *this = tmp.clone();
    //imshow("thresholded image", oImage);
    //blobDetection(iImage);

    //invert the mask
    //transform( tmpMask.begin<uint8_t>(), tmpMask.end<uint8_t>(), oImage.begin<uint8_t>(),std::logical_not<uint8_t>());

    //DEBUG
    //Use the Hough transform to detect circles in the thresholded image
    //std::vector<Vec3f> circles;

    //void HoughCircles(cv::InputArray image, OutputArray circles, int method, double dp, double minDist, double param1=100, double param2=100, int minRadius=0, int maxRadius=0 )
    //HoughCircles(oImage, circles, CV_HOUGH_GRADIENT, 1, 5, 100, 5, 3, 30);

    */
    //////////////////////////////////////////////////////////////////////////////////////////////////

}

void SmtImage::FindContours( const unsigned int iMaxNumPoints, cv::Mat& ioBinaryImage )
{
    //::NOTE:: This function at FindContours below are almost identical except for one section. I know this is
    // crappy code, but make sure these two functions stay in lock-step when making changes....or just write
    //these functions better :)
//        +--------+----+----+----+----+------+------+------+------+
//        |        | C1 | C2 | C3 | C4 | C(5) | C(6) | C(7) | C(8) |
//        +--------+----+----+----+----+------+------+------+------+
//        | CV_8U  |  0 |  8 | 16 | 24 |   32 |   40 |   48 |   56 |
//        | CV_8S  |  1 |  9 | 17 | 25 |   33 |   41 |   49 |   57 |
//        | CV_16U |  2 | 10 | 18 | 26 |   34 |   42 |   50 |   58 |
//        | CV_16S |  3 | 11 | 19 | 27 |   35 |   43 |   51 |   59 |
//        | CV_32S |  4 | 12 | 20 | 28 |   36 |   44 |   52 |   60 |
//        | CV_32F |  5 | 13 | 21 | 29 |   37 |   45 |   53 |   61 |
//        | CV_64F |  6 | 14 | 22 | 30 |   38 |   46 |   54 |   62 |
//        +--------+----+----+----+----+------+------+------+------+
        int count_black = cv::countNonZero(ioBinaryImage == 0);
        if( ioBinaryImage.type() < 7 && count_black > (ioBinaryImage.cols * ioBinaryImage.rows)/2)
        {
            //Show the contours for the pixel groups
            std::vector< std::vector< cv::Point > > contours;
            std::vector< cv::Vec4i > hierarchy;

            cv::findContours( ioBinaryImage, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );

            ioBinaryImage = cv::Mat::zeros( cv::Size( ioBinaryImage.cols, ioBinaryImage.rows ), ioBinaryImage.type() );
            // sort contours biggest first
            std::sort(contours.begin(), contours.end(), CompareContourAreas);

            std::vector< std::vector<cv::Point> > hulls(iMaxNumPoints);
            for( unsigned int i = 0; i < contours.size() && i < iMaxNumPoints; ++i )
            {
                // Initialize the contour with the convex hull points
                cv::convexHull(cv::Mat(contours[i]), hulls[i]);

                // And draw any found contours, filled
                cv::drawContours(ioBinaryImage, hulls, i/*-1allcontours*/, 255, CV_FILLED);

            }
        }
}


void SmtImage::FindContours(  const unsigned int iMaxNumPoints, cv::Mat& oBinaryImage, std::vector<PixelCluster>& oGroupedPoints ) const
{
    //::NOTE:: This function at FindContours above are almost identical except for one section. I know this is
    // crappy code, but make sure these two functions stay in lock-step when making changes....or just write
    //these functions better :)
//        +--------+----+----+----+----+------+------+------+------+
//        |        | C1 | C2 | C3 | C4 | C(5) | C(6) | C(7) | C(8) |
//        +--------+----+----+----+----+------+------+------+------+
//        | CV_8U  |  0 |  8 | 16 | 24 |   32 |   40 |   48 |   56 |
//        | CV_8S  |  1 |  9 | 17 | 25 |   33 |   41 |   49 |   57 |
//        | CV_16U |  2 | 10 | 18 | 26 |   34 |   42 |   50 |   58 |
//        | CV_16S |  3 | 11 | 19 | 27 |   35 |   43 |   51 |   59 |
//        | CV_32S |  4 | 12 | 20 | 28 |   36 |   44 |   52 |   60 |
//        | CV_32F |  5 | 13 | 21 | 29 |   37 |   45 |   53 |   61 |
//        | CV_64F |  6 | 14 | 22 | 30 |   38 |   46 |   54 |   62 |
//        +--------+----+----+----+----+------+------+------+------+

        int count_black = cv::countNonZero(*this == 0);

        //Show the contours for the pixel groups
        std::vector< std::vector< cv::Point > > contours;
        std::vector< cv::Vec4i > hierarchy;

        //We assume here that we have a thresholded image, its just has 3 channels and the first
        //channel has the info we want.
        if (this->type() < 7 && count_black > (this->cols * this->rows)/2)
        {
            cv::findContours( *this, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
        }
        else if (this->type() > 7 && count_black > (this->cols * this->rows)/2)
        {
            std::vector<Mat> channels(3);
            cv::split(*this, channels);
            oBinaryImage = channels[0];
            cv::findContours( oBinaryImage, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
        }
        else
        {
            std::cerr << "Image not in expected format." << std::endl;
            return;
        }
        oBinaryImage = cv::Mat::zeros( cv::Size( this->cols, this->rows ), this->type() );

        // sort contours biggest first
        std::sort(contours.begin(), contours.end(), CompareContourAreas);

        std::vector< std::vector<cv::Point> > hulls(iMaxNumPoints);
        for( unsigned int i = 0; i < contours.size() && i < iMaxNumPoints; ++i )
        {
            // Initialize the contour with the convex hull points
            cv::convexHull(cv::Mat(contours[i]), hulls[i]);

            // And draw any found contours, filled
            cv::drawContours(oBinaryImage, hulls, i/*-1allcontours*/, 255, CV_FILLED);

            //////////////////////////////////////////////////////////////
            cv::Mat tmp = cv::Mat::zeros( oBinaryImage.size(), CV_8UC1 );
            cv::drawContours(tmp, hulls, i/*-1allcontours*/, 255, CV_FILLED);

//            cv::namedWindow( "GetPixelClusters", CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL | CV_WINDOW_KEEPRATIO);
//            cv::imshow("GetPixelClusters", tmp );

            PixelCluster cluster;
//            for ( uint32_t point = 0; point < hulls.size(); ++point)
//            {
//                SmtPixel new_pixel(cv::Point2d(hulls[i][point].x, hulls[i][point].y), mCam);
//                cluster.Add( new_pixel );
//            }
            for (int r = 0; r < tmp.rows; ++r)
            {
                for (int c = 0; c < tmp.cols; ++c)
                {
                    if (tmp.at<uchar>(r,c) == 255)
                    {
                        SmtPixel new_pixel( cv::Point2d(c,r), mCam );
                        cluster.Add( new_pixel );
                    }
                }
            }
            if (cluster.Size() > 0)
            {
                oGroupedPoints.push_back( cluster );
            }

        }

}

void SmtImage::ThresholdToBinary( const uint32_t iFilterSize,
                                  const uint32_t iNumIterations,
                                  const uint32_t iLowerNoiseThreshold,
                                  const cv::Scalar& iLeftb,
                                  const cv::Scalar& iRightb,
                                  cv::Mat& oMat)
{


    ///::TODO:: Need to check for RGB vs BGR


    cv::Mat hsv_image;
    assert( !this->empty());

    if ( mIsRGB ){
        cv::cvtColor( *this, hsv_image, cv::COLOR_RGB2HSV );
    }
    else{
        cv::cvtColor( *this, hsv_image, cv::COLOR_BGR2HSV );
    }


    cv::Mat tmpMask;
    if( iLeftb[0] < iRightb[0] )
    { //logic to handle red values, i.e. values that cross over zero on the color wheel
        cv::Mat tmpBound;
        cv::inRange( hsv_image,
                     cv::Scalar(0, iLeftb[1], iLeftb[2]),
                     cv::Scalar(iLeftb[0]/2, iRightb[1], iRightb[2]),
                     tmpBound ); //ex iLeftb = cv::Scalar(0,125,0)
        cv::inRange( hsv_image,
                     cv::Scalar(iRightb[0]/2, iLeftb[1], iLeftb[2]),
                     cv::Scalar(360/2, iRightb[1], iRightb[2]),
                     tmpMask ); //ex iLeftb = cv::Scalar(0,125,0)
        cv::addWeighted( tmpBound, 1.0, tmpMask, 1.0, 0.0, tmpMask );
    }
    else
    {
        //Have to swap left and right hue values as inRange counts upwards
        cv::inRange( hsv_image,
                     cv::Scalar( iRightb[0]/2, iLeftb[1], iLeftb[2] ),
                     cv::Scalar( iLeftb[0]/2, iRightb[1], iRightb[2] ),
                    tmpMask ); //ex iLeftb = cv::Scalar(0,125,0)
    }


    //Filter out parts of the image with high intensity light reflections
    cv::Scalar maskOthersLowerb( 0, 0, 180);
    //cv::Scalar maskOthersUpperb( 360, 30, 255);
    cv::Scalar maskOthersUpperb( 0, 0, 180);
    //cv::Scalar maskOthersLowerb( 0, 0, 0);
    //cv::Scalar maskOthersUpperb( 0, 0, 0);
    cv::Mat tmpMaskOther;
    cv::inRange( hsv_image, maskOthersLowerb, maskOthersUpperb, tmpMaskOther );
    //cv::inRange( iImage, cv::Scalar( 230,230,230 ), cv::Scalar(255,255,255), tmpMaskOther );


    cv::bitwise_not( tmpMaskOther, tmpMaskOther );
    //cv::imshow("tmpMaskOther", tmpMaskOther);
    //cv::waitKey( 0 );
    cv::Mat tmp;
    cv::Mat tmp_image;

    tmpMask.copyTo( tmp, tmpMaskOther );
    //cv::imshow("tmp", tmp);
    //cv::waitKey(0);
    //Noise reduction code ---
    ///@todo should be user specified as it is very variable on the data collection how much color noise there is....
    //These thresholding operations are fine as we are currently dealing with a 1-d mask for the color we were searching for
    if ( iNumIterations > 0 )
    {
        for( uint32_t i = 0; i < iNumIterations; ++i )
        {
            cv::blur( tmp, tmp_image, cv::Size(iFilterSize, iFilterSize), cv::Point(-1, -1) );
            cv::threshold( tmp_image, tmp, iLowerNoiseThreshold, 255, cv::THRESH_BINARY );
        }

        oMat = tmp.clone();
    }
    else
    {
        cv::threshold( tmp, tmp_image, iLowerNoiseThreshold, 255, cv::THRESH_BINARY );
        oMat = tmp_image.clone();
    }

}

void SmtImage::Threshold( const uint32_t iFilterSize,
                          const uint32_t iNumIterations,
                          const uint32_t iMaxNumPoints,
                          const uint32_t iLowerNoiseThreshold,
                          const cv::Scalar& iLeftb,
                          const cv::Scalar& iRightb,
                          cv::Mat& oMat)
{

        Mat tmp;
        ThresholdToBinary(iFilterSize,
                          iNumIterations,
                          iLowerNoiseThreshold,
                          iLeftb,
                          iRightb,
                          tmp);

        FindContours( iMaxNumPoints, tmp );

        //cvtColor ( tmp, tmp_image, CV_GRAY2BGR );
        //cv::Mat img2 = cv::Mat::zeros( cv::Size( tmp.cols, tmp.rows ), CV_8UC3 );
        cv::Mat g = cv::Mat::zeros( cv::Size( tmp.cols, tmp.rows ), CV_8UC1 );
        std::vector<cv::Mat> mergedImage = {tmp, tmp, g}; //B, G, R not R, G, B
        cv::Mat final_image;
        cv::merge( mergedImage, final_image );
        oMat = final_image.clone(); 
}

////rotate image by arbitrary degree ( very expensive)
//SmtImage SmtImage::Rotate( double iRotationAngle )
//{
//    cv::Point2f point_cp( this->cols*0.5, this->rows*0.5 );
//    cv::Mat M = cv::getRotationMatrix2D( point_cp, iRotationAngle, 1.0 );
//    cv::Mat tmp;
//    cv::warpAffine( *this, tmp, M, this->size(), cv::INTER_CUBIC ); //Nearest is too rough,
//    SmtImage new_image(mLocation,
//                     mCam,
//                     mTimestep,
//                     (cv::Mat)(*this),
//                     mIsRGB);

//    return SmtImage(new_image, tmp, mIsRGB );
//}

void SmtImage::BackgroundSubtraction( const SmtImage& iBackImage )
{
    cv::Mat diff;
    cv::absdiff( *this, iBackImage, diff );

    cv::Mat mask_bw;
    cv::cvtColor( diff, mask_bw, CV_BGR2GRAY );

    //attempting to prevent small lighting changes to be caught by mask
    cv::threshold( mask_bw , mask_bw , 20, 255, cv::THRESH_TOZERO );

    //imshow( "mask_bw", mask_bw);
    //cv::waitKey(0);
    //cv::Mat maskedImage;
    this->copyTo( diff, mask_bw );
    *this = diff.clone();
}

void SmtImage::BackgroundSubtraction( const cv::Mat& iBackImage )
{
    cv::Mat diff;
    cv::absdiff( *this, iBackImage, diff );

    cv::Mat mask_bw;
    cv::cvtColor( diff, mask_bw, CV_BGR2GRAY );

    //attempting to prevent small lighting changes to be caught by mask
    cv::threshold( mask_bw , mask_bw , 20, 255, cv::THRESH_TOZERO );

    //imshow( "mask_bw", mask_bw);
    //cv::waitKey(0);
    //cv::Mat maskedImage;
    this->copyTo( diff, mask_bw );
    *this = diff.clone();
}

void SmtImage::GetInterestingPixels( std::vector<SmtPixel>& oPixels ) const
{
    cv::Vec3b intensity;
    for (int x = 0; x < this->rows; ++x)
    {
        for (int y = 0; y < this->cols; ++y)
        {
            intensity = this->at< cv::Vec3b >( cv::Point(x,y) );
            int intensitySum = intensity.val[0] + //blue
                               intensity.val[1] + //green
                               intensity.val[2]; //red

            if (intensitySum > 0)
            {
                SmtPixel new_pixel( cv::Point(x,y), mCam );
                oPixels.push_back( new_pixel );
            }
        }
    }
}

void SmtImage::Undistort()
{
    CamMats* cam_matrices = CamMats::Instance();
    if (!cam_matrices->isInit())
    {
        assert("Must initialize CamMats object before use.");
    }

    cv::Mat image_out;
    undistort( *this, image_out, cam_matrices->K()[mCam], cam_matrices->Distortion()[mCam] );

    *this = image_out.clone();
    mIsUndistorted = true;
}


void SmtImage::GetPixelClusters( const unsigned int iMaxNumClusters, std::vector<PixelCluster>& oGroupedPoints ) const
{
    //probably don't need these inits, just use current object, but being safe as this is a quick change.
    cv::Mat tmp_mat;

    FindContours(  iMaxNumClusters, tmp_mat, oGroupedPoints );


///////////////////////////////////////////////////////////////////////////////////
/*
    ///@TODO should do a check here to make sure the image is binary before continuing, otherwise there will probably be a opencv assert error

    std::vector< std::vector< cv::Point > > contours;
    std::vector< cv::Vec4i > hierarchy;

    ///@TODO
    //findContours just gives us the outline, to get all the points we would need to use drawContours to create a mask and then apply the mask and process.
    cv::findContours( *this, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );

    // sort contours biggest first
    std::sort(contours.begin(), contours.end(), CompareContourAreas);
    std::vector< cv::Scalar > colors = { cv::Scalar(128, 128, 128),
                                         cv::Scalar(0, 255, 0),
                                         cv::Scalar(255, 0, 0),
                                         cv::Scalar(255, 0, 255),
                                         cv::Scalar(0,255,255),
                                         cv::Scalar(0,0,255),
                                         cv::Scalar(255,255,128),
                                         cv::Scalar(128,255,255),
                                         cv::Scalar(255,128,255),
                                         cv::Scalar(255, 255, 0) } ; // BGR color ordering

    for (unsigned int i = 0; i < contours.size() && i < iMaxNumClusters; ++i)
    {
        cv::Mat temporary_mask_image( this->rows, this->cols, CV_64F, double(0) ); // Work image

        // Create with 1 "contour" for our convex hull
        std::vector< std::vector<cv::Point> > hulls(1);

        // Initialize the contour with the convex hull points
        cv::convexHull( cv::Mat(contours[i]), hulls[0] );

        // And draw that single contour, filled
        cv::drawContours( temporary_mask_image, hulls, 0, 255, CV_FILLED );
        //cv::imshow("drawn Contours", temporary_mask_image );
        //cv::waitKey(0);

        PixelCluster cluster;
        for (int r = 0; r < temporary_mask_image.rows; ++r)
        {
            for (int c = 0; c < temporary_mask_image.cols; ++c)
            {
                if (temporary_mask_image.at<double>(r,c) == 255)
                {
                    SmtPixel new_pixel( cv::Point2d(c,r), mCam );
                    cluster.Add( new_pixel );
                }
            }
        }
        if (cluster.Size() > 0)
        {
            oGroupedPoints.push_back( cluster );
            //cv::circle( temporary_mask_image, aCluster.mean, 5, colors[i], 3 );
            //cv::imshow("Clusters", temporary_mask_image );
            //cv::waitKey(0);
        }
    }
    */
////////////////////////////////////////////////////////////////////////
//		for (int group = 0; group < contours.size(); ++group)
//      {
//			Cluster cluster;
//			for (int pixel = 0; pixel < contours[group].size(); ++pixel)
//          {
//				SmtPixel new_pixel( contours[group][pixel], cam );
//				cluster.addPoint( new_pixel );
//			}
//			if (cluster.size > 0)
//          {
//				oGroupedPoints.push_back( cluster );
//			}
//		}
}

bool SmtImage::RGB() const
{
    return mIsRGB;
}
