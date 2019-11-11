#ifndef MultiPointTracker_h
#define MultiPointTracker_h

#include <vector>

#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "munkres/Munkres.h"
#include "common_types.h"
#include "image/SmtPixel.h"

class MultiPointTracker
{
    public:
        MultiPointTracker( uint8_t iNumPoints);
        virtual ~MultiPointTracker();
        void Initialize( std::vector< SmtPixel > iStartingPoints  );

        bool Next( const std::vector< SmtPixel >& iUnorderedPoints,
                   uint32_t iTimestep,
                   std::vector< SmtPixel >& oCorresPoints );
        bool FillInMissingPointsFromPredictions( const std::vector< SmtPixel>& iIncompleteList,
                                                 const uint32_t iTimestep,
                                                 std::vector< SmtPixel >&  oFilledInList );


    protected:
        bool GetPredictedStates( const uint32_t iTimestep, std::vector< cv::Point2d> & oPredictedLocations );

        std::vector< cv::KalmanFilter* > mKFilterPtrs;
        std::vector< uint32_t > mKFilterTimestep;
        uint8_t mNumPoints;
        uint8_t mNumCams;

    private:
        cv::Mat_<int> GetCostMat(const std::vector< SmtPixel >& iUnorderedPoints, uint32_t iTimestep );
        std::vector< std::pair<int, int> > OrderCorrespondingPoints(const cv::Mat_<int>& iCostMat);
};



#endif // MultiPointTracker_h
