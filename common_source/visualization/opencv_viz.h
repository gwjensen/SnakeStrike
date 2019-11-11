#ifndef COMMON_VIZ_OPENCV_VIZ_H
#define COMMON_VIZ_OPENCV_VIZ_H

#include <opencv2/core.hpp>

#include "image/SmtPixel.h"
#include "image/SmtImage.h"
#include "common_types.h" 


void ViewCorrespondence(const std::vector< SmtPixel >& iFixedPixels, const std::vector< SmtPixel >& iPixelForSpecificCam, SmtImage iFixedImage, SmtImage iImageForCam, std::string const& iTitle="Corresponding Pixels in the images" ) ;


void ViewCorrespondence(const std::vector< std::vector< SmtPixel > >& iPixelsForEachCam, std::vector< SmtImage > iCamImages, std::string const & iTitle ="Corresponding Pixels in the images" ) ;

void ViewOldAndNewImage( const cv::Mat& iImageOld, const cv::Mat& iImageNew) ;

void PrintMat( const cv::Mat& iMat, int iNumRows/*, int iPrec*/ );

void PrintMat( const cv::Mat& iMat/*, int iPrec = 9*/);

void ShowPerspective( cv::Mat& iMat1, cv::Mat& iMat2 );

#endif //COMMON_VIZ_OPENCV_VIZ_H
