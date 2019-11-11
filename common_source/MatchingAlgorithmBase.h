#ifndef MatchingAlgorithmBase_h
#define MatchingAlgorithmBase_h

#include "PixelSet.h"
#include "File.h"
#include "Correspondence.h"

class MatchingAlgorithmBase
{
    public:
        MatchingAlgorithmBase(){};
        ~MatchingAlgorithmBase(){};
        virtual void MatchPoints( uint64_t& oFirstMatchTimestep,
                          PixelSet&   ioPixelsToTrack,
                          const TrackerConfigFile&       iConfig,
                          //const ImageSet& iTimestepImages,
                          const std::pair<unsigned long, std::set<unsigned long> >& iCamIndexesToExclude,
                          bool (*CancelFunc)(),
                          std::vector< std::vector< SmtPixel > > iMarkedPoints,
                          bool& oRunSuccessfully
                          //std::vector< std::vector< std::vector< int32_t > > >& oBestFit,
                          //std::vector< double >& oBestFitErrors,
                          //PixelSet& oFirstTimestepWithAllPoints
                                  ) = 0;
};

#endif // MatchingAlgorithmBase_h
