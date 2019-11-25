#ifndef SmtImage_h
#define SmtImage_h

#include <string>

#include <opencv2/core.hpp>

#include "SmtPixel.h"
#include "PixelCluster.h"


//This image is expected to be either RGB or BGR

struct SmtImageThresholdInfo
{
   uint32_t max_num_points;
   uint32_t noise_filter_size;
   uint32_t noise_iterations;
   uint32_t noise_threshold;

   SmtImageThresholdInfo() : max_num_points(0),
                             noise_filter_size(0),
                             noise_iterations(0),
                             noise_threshold(0) {}
   SmtImageThresholdInfo( uint32_t iMaxPoints,
                          uint32_t iFiltSize,
                          uint32_t iIter,
                          uint32_t iThresh )
                          : max_num_points(iMaxPoints),
                            noise_filter_size(iFiltSize),
                            noise_iterations(iIter),
                            noise_threshold(iThresh) {}
};

class SmtImage : public cv::Mat
{
    public:
        explicit SmtImage(const std::string  iLocation,
                 const int          iCam,
                 const int          iTimestep,
                 const cv::Mat&     iImage,
                 const bool         iRGB);

        //Note: Matrix is not initialized....did this to save a matrix copy. Should only be used in combination
        //      with the Load() function.
        explicit SmtImage(const std::string  iLocation,
                 const int          iCam,
                 const int          iTimestep );

        explicit SmtImage( const SmtImage&    iFrom,
                  const cv::Mat&     iNew,
                  const bool         iRGB);

        SmtImage( const SmtImage&   iFrom );
        virtual ~SmtImage();

        SmtImage& operator=(const SmtImage& other); // copy assignment

        static bool CompareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 );

        bool Save( const std::string& iPath );
        bool WriteToPipe( FILE* iPipe );
        bool Load( const std::string& iPath );

        void CopyAttributes( const SmtImage& iImage );

        void GammaCorrection( float fGamma );


        void Threshold( const cv::Scalar& iLowerb,
                        const cv::Scalar& iUpperb,
                        const SmtImageThresholdInfo& iExtraThresholdInfo );

        void Threshold( const uint32_t iFilterSize,
                                  const uint32_t iNumIterations,
                                  const uint32_t iMaxNumPoints,
                                  const uint32_t iLowerNoiseThreshold,
                                  const cv::Scalar& iLeftb,
                                  const cv::Scalar& iRightb,
                                  cv::Mat& oMat);

        //rotate image by arbitrary degree ( very expensive)
//        SmtImage Rotate( double iRotationAngle );
        //SmtImage Rotate( double iRotationAngle, SmtImage& oImage );

        //SmtImage Undistort( );
        void Undistort();

        void BackgroundSubtraction( const SmtImage& iBackImage );
        void BackgroundSubtraction( const cv::Mat& iBackImage );

        void BlobDetection();
        void GetInterestingPixels( std::vector<SmtPixel>& oPixels ) const;

        //Note: only works when the image is in a binary format.
        void GetPixelClusters( const unsigned int iMaxNumClusters, std::vector<PixelCluster>& oGroupedPoints ) const;

        std::string Location()  const { return mLocation; }
        std::string FileSuffix() const { return mFileSuffix; }
        int         Cam()       const { return mCam; }
        int         Timestep()  const { return mTimestep; }

        bool        IsBinary()  const { return mIsBinary; }
        bool        IsEmpty()   const { return this->empty(); }

        int Width() const { return size().width; }
        int Height() const { return size().height; }
        bool RGB() const;


    private:
        //I have to disallow this equals because people will not keep track of BGR vs RBG.
        SmtImage& operator=(const cv::Mat& other); // copy assignment

        void FindContours(  const unsigned int iMaxNumPoints, cv::Mat& ioBinaryImage, std::vector<PixelCluster>& oGroupedPoints ) const;

        void FindContours( const unsigned int iMaxNumPoints, cv::Mat& ioBinaryImage );

        void ThresholdToBinary( const uint32_t iFilterSize,
                                  const uint32_t iNumIterations,
                                  const uint32_t iLowerNoiseThreshold,
                                  const cv::Scalar& iLeftb,
                                  const cv::Scalar& iRightb,
                                  cv::Mat& oMat);

        std::string mLocation; //Will be empty when the image is not loaded from saved file
        std::string mFileSuffix; //Will be empty when the image is not loaded from saved file
        int         mCam;
        int         mTimestep;
        bool        mIsBinary;
        bool        mIsUndistorted;
        bool        mIsRGB;

};

#endif // SmtImage_h

