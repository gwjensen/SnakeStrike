#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <QDir>

#include "image/SmtImage.h"

#include "GuiImage.h"

    //Pylon::CPylonImageWindow imageWindow;
GuiImage::GuiImage(const std::string  iLocation,
                   const int          iCam,
                   const int          iTimestep,
                   const cv::Mat&     iImage,
                   const bool         iRGB,
                   const uint64_t     iTimestamp)
    : SmtImage( iLocation, iCam, iTimestep, iImage, iRGB ),
     mTimestamp( iTimestamp )
{

}


GuiImage::GuiImage( const std::string  iLocation,
                    const int          iCam,
                    const int          iTimestep,
                    const uint64_t     iTimestamp)
    : SmtImage( iLocation, iCam, iTimestep ),
      mTimestamp( iTimestamp )
{

}

GuiImage::GuiImage( const SmtImage&    iFrom,
                    const cv::Mat&     iNew,
                    const bool         iRGB,
                    const uint64_t     iTimestamp)
    : SmtImage( iFrom, iNew, iRGB ),
      mTimestamp( iTimestamp )
{

}

GuiImage::GuiImage( const SmtImage&   iFrom,
                    const uint64_t    iTimestamp)
    : SmtImage( iFrom ),
      mTimestamp( iTimestamp )
{

}

bool CompareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 )
{
    //Order largest contour area first
    double i = fabs( contourArea(cv::Mat(contour1)) );
    double j = fabs( contourArea(cv::Mat(contour2)) );
    return ( i > j );
}

void GuiImage::ThresholdImage( const uint32_t iFilterSize,
                               const uint32_t iNumIterations,
                               const uint32_t iMaxNumPoints,
                               const uint32_t iLowerNoiseThreshold,
                               const cv::Scalar& iLeftb,
                               const cv::Scalar& iRightb,
                               QPixmap& oPixMap )
{
    cv::Mat tmp_mat;
    SmtImage::Threshold( iFilterSize,
                         iNumIterations,
                         iMaxNumPoints,
                         iLowerNoiseThreshold,
                         iLeftb,
                         iRightb,
                         tmp_mat );
    oPixMap = QPixmap::fromImage( QImage(tmp_mat.data,
                                         tmp_mat.size().width,
                                         tmp_mat.size().height,
                                         QImage::Format_RGB888) );

}

QImage GuiImage::ToQImage()
{
    QImage image((uint8_t*)data, Width(), Height(), QImage::Format_RGB888);
    return image;
}

GuiImage::~GuiImage()
{
    //delete data;
}

QPixmap GuiImage::ToQPixmap()
{
    QPixmap pixmap = QPixmap::fromImage( QImage((uint8_t*)data,
                                                Width(),
                                                Height(),
                                                QImage::Format_RGB888));
    return pixmap;
}

void GuiImage::SavePng( const std::string& iPath )
{
    Save( iPath );
}

void BackgroundSubtractionSimple( const cv::Mat& iImage,
                                  const cv::Mat& iBackImage,
                                  cv::Mat& oImage )
{
    cv::Mat diff;
    cv::absdiff(iImage, iBackImage, diff);

    cv::Mat mask_bw;
    cv::cvtColor(diff, mask_bw, CV_BGR2GRAY);

    //attempting to prevent small lighting changes to be caught by mask
    cv::threshold(mask_bw , mask_bw , 20, 255, cv::THRESH_TOZERO);

    iImage.copyTo( oImage, mask_bw );
}

uint64_t GuiImage::Timestamp() const
{
    return mTimestamp;
}
uint64_t& GuiImage::Timestamp()
{
    return mTimestamp;
}


