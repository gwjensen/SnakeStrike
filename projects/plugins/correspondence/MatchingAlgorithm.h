#ifndef MatchingAlgorithm_h
#define MatchingAlgorithm_h

#include "MatchingAlgorithmBase.h"

class MatchingAlgorithm : public MatchingAlgorithmBase
{
    public:
        MatchingAlgorithm();
        void MatchPoints( uint64_t& oFirstMatchTimestep,
                          PixelSet&   ioPixelsToTrack,
                          const TrackerConfigFile&       iConfig,
                         // const ImageSet& iTimestepImages,
                          const std::pair<unsigned long, std::set<unsigned long> >& iCamIndexesToExclude,
                          bool (*CancelFunc)(),
                          std::vector< std::vector< SmtPixel > > iMarkedPoints,
                          bool& oRunSuccessfully
                          //std::vector< std::vector< std::vector< int32_t > > >& oBestFit,
                          //std::vector< double >& oBestFitErrors,
                          //PixelSet& oFirstTimestepWithAllPoints
                          );
};

#endif // MatchingAlgorithm_h
