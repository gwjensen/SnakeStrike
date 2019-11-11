#ifndef GuiImage_h
#define GuiImage_h
#include <QImage>
#include <QPixmap>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "image/SmtImage.h"


class GuiImage: public SmtImage
{
    public:
        //GuiImage(GrabResultPtr_t pointer, const uint32_t iCamIdx);
        GuiImage(const std::string  iLocation,
                 const int          iCam,
                 const int          iTimestep,
                 const cv::Mat&     iImage,
                 const bool         iRGB,
                 const uint64_t     iTimestamp);

        //Note: Matrix is not initialized....did this to save a matrix copy. Should only be used in combination
        //      with the Load() function or with a function that checks if the returned image is empty or not
        GuiImage(const std::string  iLocation,
                 const int          iCam,
                 const int          iTimestep,
                 const uint64_t     iTimestamp);

        GuiImage( const SmtImage&    iFrom,
                  const cv::Mat&     iNew,
                  const bool         iRGB,
                  const uint64_t     iTimestamp);

        GuiImage( const SmtImage&   iFrom,
                  const uint64_t    iTimestamp);

        //GuiImage( const std::string& iFilename,
        //          const uint32_t iCamIdx );

        //GuiImage( const std::string& iFileName,
        //          const std::string& iMaskName,
        //          const uint32_t iCamIdx );
        ~GuiImage();
        QImage ToQImage();
        static QImage ToQImage( cv::Mat iMat );
        QPixmap ToQPixmap();
        static QPixmap ToQPixmap( cv::Mat iMat );
        //void SaveBmp( QString iPath );
        void SavePng( const std::string& iPath );

        void ThresholdImage( const uint32_t iFilterSize,
                             const uint32_t iNumIterations,
                             const uint32_t iMaxNumPoints,
                             const uint32_t iLowerNoiseThreshold,
                             const cv::Scalar& iLowerb,
                             const cv::Scalar& iUpperb,
                             QPixmap& oPixMap
                             );
        /*void thresholdImage( const uint32_t iFilterSize,
                             const uint32_t iNumIterations,
                             const uint32_t iLowerNoiseThreshold,
                             const cv::Scalar& iLowerb,
                             const cv::Scalar& iUpperb
                             );*/

        uint64_t Timestamp() const;
        uint64_t& Timestamp();

    private:
        uint64_t mTimestamp;

};

#endif // GuiImage_h
