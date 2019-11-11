#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/video.hpp>

#include "common_types.h" 

#include "opencv_viz.h"

void ViewCorrespondence( const std::vector< SmtPixel >& iFixedPixels,
                         const std::vector< SmtPixel >& iPixelForSpecificCam,
                         SmtImage iFixedImage,
                         SmtImage iImageForCam,
                         std::string const& iTitle )
{
    //printMat(iImageOld, 1, 4);
    //printMat(iImageNew, 1, 4);

    int width = iFixedImage.cols;
    int height = iFixedImage.rows;
    cv::Mat double_image = cv::Mat::zeros( cv::Size( width * 2, height ), CV_8UC3 );
    //std::fprintf(stderr, "doubleImage.size(): (%d,%d,%d,%d)\n", doubleImage.rows, doubleImage.cols, doubleImage.channels(), doubleImage.type());
    cv::Rect rect1( 0, 0, width, height );
    //std::fprintf(stderr, "rect1.size(x,y,w,h): (%d,%d,%d,%d)\n", rect1.x, rect1.y, rect1.width, rect1.height);
    cv::Mat topLeft( double_image, rect1 );

    cv::Rect rect2( width, 0, width, height );
    //std::fprintf(stderr, "rect2.size(x,y,w,h): (%d,%d,%d,%d)\n", rect2.x, rect2.y, rect2.width, rect2.height);

    cv::Mat topRight( double_image, rect2 );


    std::vector< cv::Scalar > colors = { cv::Scalar(128, 128, 128),
                                         cv::Scalar(0, 255, 0),
                                         cv::Scalar(255, 0, 0),
                                         cv::Scalar(255, 255, 0) } ; // BGR color ordering

    //std::fprintf( stderr, "Threre are %ld fixed pixels\n", iFixedPixels.size() );
    for( uint32_t h = 0; h < iFixedPixels.size(); ++h ) {
        //std::fprintf( stderr, "--- fixedPixel (%lf, %lf)\n", iFixedPixels[h].x, iFixedPixels[h].y );
        cv::circle( iFixedImage, iFixedPixels[h], 5, colors[h], 3 );
    }
    //std::fprintf( stderr, "Threre are %ld pixels ForSpecificCam\n", iPixelForSpecificCam.size() );
    for( uint32_t h = 0; h < iPixelForSpecificCam.size(); ++h ) {
        //std::fprintf( stderr, "--- iPixelForSpecificCam[ (%lf, %lf)\n", iPixelForSpecificCam[h].x, iPixelForSpecificCam[h].y );
        cv::circle( iImageForCam, iPixelForSpecificCam[h], 5, colors[h], 3 );
    }


    iFixedImage.copyTo( topLeft );
    iImageForCam.copyTo( topRight );

    cv::imshow( iTitle, double_image);
    cv::waitKey(0); //wait until a key is pressed.

}


void ViewCorrespondence( const std::vector< std::vector< SmtPixel > >& iPixelsForEachCam,
                         std::vector< SmtImage > iCamImages,
                         std::string const & iTitle  )
{
        //printMat(iImageOld, 1, 4);
        //printMat(iImageNew, 1, 4);

        int width = iCamImages[0].cols;
        int height = iCamImages[0].rows;
        cv::Mat double_image = cv::Mat::zeros( cv::Size( width * 2, height * 2 ), CV_8UC3 );
        //std::fprintf(stderr, "doubleImage.size(): (%d,%d,%d,%d)\n",
        //             doubleImage.rows, doubleImage.cols, doubleImage.channels(), doubleImage.type());
        cv::Rect rect1( 0, 0, width, height );
        //std::fprintf(stderr, "rect1.size(x,y,w,h): (%d,%d,%d,%d)\n", rect1.x, rect1.y, rect1.width, rect1.height);
        cv::Mat top_left( double_image, rect1 );

        cv::Rect rect2( width, 0, width, height );
        //std::fprintf(stderr, "rect2.size(x,y,w,h): (%d,%d,%d,%d)\n", rect2.x, rect2.y, rect2.width, rect2.height);

        cv::Mat top_right( double_image, rect2 );

        cv::Rect rect3( 0, height, width, height );
        //std::fprintf(stderr, "rect1.size(x,y,w,h): (%d,%d,%d,%d)\n", rect1.x, rect1.y, rect1.width, rect1.height);
        cv::Mat bottom_left( double_image, rect3 );

        cv::Rect rect4( width, height, width, height );
        //std::fprintf(stderr, "rect2.size(x,y,w,h): (%d,%d,%d,%d)\n", rect2.x, rect2.y, rect2.width, rect2.height);

        cv::Mat bottom_right( double_image, rect4 );

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
        for (uint32_t i = 0; i < iPixelsForEachCam.size(); ++i)
        {
            if (0 == iPixelsForEachCam[i].size())
            {
                continue;
            }
            int cam_idx = iPixelsForEachCam[i][0].Cam();
            for (uint32_t h = 0; h < iPixelsForEachCam[i].size(); ++h)
            {
                if (iCamImages[ cam_idx ].Cam() != cam_idx)
                {
                    std::fprintf( stderr, "***********\nERROR*****\n*************\n    Camera images are not in 0-3 order....\n" );
                }
                else
                {
                    cv::circle( iCamImages[ cam_idx ], iPixelsForEachCam[i][h], 5, colors[h], 3 );
                    //std::fprintf(stderr, "--- iPixelForSpecificCam[ (%lf, %lf)\n",
                    //             iPixelsForEachCam[i][h].x,iPixelsForEachCam[i][h].y);
                }
            }
        }
        iCamImages[0].copyTo( top_left );
        iCamImages[1].copyTo( top_right );
        iCamImages[2].copyTo( bottom_left );
        iCamImages[3].copyTo( bottom_right );

        cv::imshow( iTitle, double_image);
        cv::waitKey(0); //wait until a key is pressed.
}

void ViewOldAndNewImage( const cv::Mat& iImageOld, const cv::Mat& iImageNew )
{
        //printMat(iImageOld, 1, 4);
        //printMat(iImageNew, 1, 4);
        cv::Mat double_image = cv::Mat::zeros( cv::Size( iImageOld.cols + iImageNew.cols, iImageNew.rows), CV_8UC3 );
        //std::fprintf(stderr, "doubleImage.size(): (%d,%d,%d,%d)\n", doubleImage.rows, doubleImage.cols, doubleImage.channels(), doubleImage.type());
        cv::Rect rect1( 0, 0, iImageNew.cols, iImageNew.rows );
        //std::fprintf(stderr, "rect1.size(x,y,w,h): (%d,%d,%d,%d)\n", rect1.x, rect1.y, rect1.width, rect1.height);
        cv::Mat left( double_image, rect1 );

        cv::Rect rect2( iImageOld.cols, 0, iImageOld.cols, iImageOld.rows );
        //std::fprintf(stderr, "rect2.size(x,y,w,h): (%d,%d,%d,%d)\n", rect2.x, rect2.y, rect2.width, rect2.height);

        cv::Mat right( double_image, rect2 );

        cv::Mat img2 = cv::Mat::zeros( cv::Size( iImageNew.cols, iImageNew.rows ), CV_8UC3 );
        cv::Mat g = cv::Mat::zeros( cv::Size( iImageNew.cols, iImageNew.rows ), CV_8UC1 );
        //std::fprintf(stderr, "g.size(row,col,chan,type): (%d,%d,%d,%d)\n", g.rows, g.cols, g.channels(), g.type());
        //std::fprintf(stderr, "iImageOld.size(row,col,chan,type): (%d,%d,%d,%d), iImageNew.size(): (%d,%d,%d,%d), img2: (%d,%d,%d,%d)\n", iImageOld.rows, iImageOld.cols,iImageOld.channels(),iImageOld.type(), iImageNew.rows, iImageNew.cols, iImageNew.channels(), iImageNew.type(), img2.rows, img2.cols, img2.channels(), img2.type());

        std::vector<cv::Mat> merged_image = {g, g, iImageNew}; //B, G, R not R, G, B
        cv::merge( merged_image, img2 );

        //std::fprintf( stderr, " \n" );
        img2.copyTo( left );
        iImageOld.copyTo( right );
        cv::imshow("Thresholded Image", double_image);
        cv::waitKey(0); //wait until a key is pressed.
}

void PrintMat( const cv::Mat& iMat, int iNumRows/*, int iPrec*/ )
{
    /*if (iMat.size().height == 0){
        std::cerr << "----Matrix is of size zero!!!" << std::endl;
    }
    else{
        for( int i = 0; i < iNumRows; ++i){
            std::cout << "[";
            for( int j = 0; j < iMat.size().width; ++j){
                std::cout << setprecision(iPrec) << iMat.at<double>(i,j);
                if( j != iMat.size().width -1)
                    std::cout << ", ";
                else
                    std::cout << "]" << std::endl;
            }
        }
    }*/
    cv::Mat small_mat( iMat, cv::Rect( 0,0, iMat.cols, iNumRows ) );
    std::cerr << small_mat << std::endl;
}
void PrintMat( const cv::Mat& iMat/*, int iPrec = 9*/ )
{
    std::cerr << iMat << std::endl;
    //printMat(iMat, iMat.size().height, iPrec);
}

void ShowPerspective( cv::Mat& iMat1, cv::Mat& iMat2 )
{
    // Convert images to gray scale;
    cv::Mat im1_gray, im2_gray;
    cvtColor( iMat1, im1_gray, CV_BGR2GRAY );
    cvtColor( iMat2, im2_gray, CV_BGR2GRAY );

    // Define the motion model
    const int warp_mode = cv::MOTION_EUCLIDEAN;

    // Set a 2x3 or 3x3 warp matrix depending on the motion model.
    cv::Mat warp_matrix;

    // Initialize the matrix to identity
    if ( warp_mode == cv::MOTION_HOMOGRAPHY )
    {
        warp_matrix = cv::Mat::eye( 3, 3, CV_32F );
    }
    else
    {
        warp_matrix = cv::Mat::eye( 2, 3, CV_32F );
    }

    // Specify the number of iterations.
    int number_of_iterations = 50;

    // Specify the threshold of the increment
    // in the correlation coefficient between two iterations
    double termination_eps = 1e-10;

    // Define termination criteria
    cv::TermCriteria criteria ( cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                                number_of_iterations,
                                termination_eps );

    // Run the ECC algorithm. The results are stored in warp_matrix.
    cv::findTransformECC( im1_gray,
                          im2_gray,
                          warp_matrix,
                          warp_mode,
                          criteria );

    // Storage for warped image.
    cv::Mat im2_aligned;

    if (warp_mode != cv::MOTION_HOMOGRAPHY)
    {
        // Use warpAffine for Translation, Euclidean and Affine
        cv::warpAffine( iMat2, im2_aligned, warp_matrix, iMat1.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP );
    }
    else
    {
        // Use warpPerspective for Homography
        cv::warpPerspective( iMat2, im2_aligned, warp_matrix, iMat1.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP );
    }

    // Show final result
    cv::Mat tmp_img_not_align;
    cv::addWeighted( iMat1, 1.0, iMat2, 1.0, 0.0, tmp_img_not_align );
    cv::imshow("Image 2 NOT-Aligned", tmp_img_not_align);
    cv::addWeighted( iMat1, 1.0, im2_aligned, 1.0, 0.0, im2_aligned );
    cv::imshow("Image 2 Aligned", im2_aligned);
    cv::waitKey(0);
}


