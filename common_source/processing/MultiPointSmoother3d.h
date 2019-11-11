#ifndef MultiPointSmoother3d_h
#define MultiPointSmoother3d_h

#include "MultiPointTracker.h"

class MultiPointSmoother3d: public MultiPointTracker
{
    public:
        MultiPointSmoother3d( uint8_t iNumPoints);
        ~MultiPointSmoother3d();

        void Initialize(uint32_t iFirstMatchTimestep, std::vector< cv::Point3d > iStartingPoints  );
        bool Next( const std::vector< cv::Point3d >& iMeasuredPoints,
                   uint32_t iTimestep,
                   std::vector< cv::Point3d >& oSmoothedPoints );

        bool IsSetup(){ return mIsInitialized; }

     protected:

        //cv::Mat_<int> getCostMat(const std::vector< SmtPixel >& iUnorderedPoints, uint32_t iTimestep );

     private:
        bool mIsInitialized;

};

#endif // MultiPointSmoother3d_h
