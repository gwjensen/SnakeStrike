#ifndef ImageSet_h
#define ImageSet_h

#include <set>
#include <opencv2/core.hpp>

#include "SmtImage.h"
#include "PixelSet.h"
#include "PixelClusterSet.h"

//ImageSets are assumed to be [timestep[camera_image]]
typedef std::vector< std::vector< SmtImage > > ImageSet;


void ThresholdImages( const ImageSet& iImagesSet,
                      ImageSet& oImagesSet,
                      bool (*CancelFunc)(),
                      const cv::Scalar& iLowerb,
                      const cv::Scalar& iUpperb,
                      const SmtImageThresholdInfo& iExtraThresholdInfo );

//void thresholdImages( ImageSet& ioImagesSet, const cv::Scalar& iLowerb, const cv::Scalar& iUpperb );

void MergeThresholdHalves( const ImageSet& iImageSet1,
                           const ImageSet& iImageSet2,
                           ImageSet& oImageSet
                           );

void UndistortImages( ImageSet& ioImagesSet,
                      bool (*CancelFunc)(),
                      const std::pair<unsigned long, std::set<unsigned long> >& iCamIndexesToExclude,
                      const std::string& iWriteLocation
                      );

void BackgroundSubtraction( const uint32_t iCamIdx );


void BackgroundSubtraction( const uint32_t iCamIdx, cv::Mat& oMask );

void GetInterestingPixels( const ImageSet& iImageSet, PixelSet& oPixelSet );

void GetPixelClustersGroupTimestep( int iStartTimestep,
                                    int iEndTimestep,
                                    const ImageSet& iImageSet,
                                    bool (*CancelFunc)(),
                                    const int iMaxNumClusters,
                                    const std::pair<unsigned long, std::set<unsigned long> >& iCamIndexesToExclude,
                                    PixelClusterSet& oClusterSet );

void GetPixelClusters( const ImageSet& iImagesSet,
                       bool (*CancelFunc)(),
                       const std::pair<unsigned long, std::set<unsigned long> >& iCamIndexesToExclude,
                       const int iMaxNumClusters,
                       PixelClusterSet& oClusterSet );

#endif // ImageSet_h
