#ifndef PixelCluster_h
#define PixelCluster_h

#include <opencv2/core.hpp>

#include "SmtPixel.h"

class PixelCluster
{
    public:
        PixelCluster();

        virtual ~PixelCluster();

        void Add( const SmtPixel& iPoint );

        void Remove( const SmtPixel& iPoint );

        void Merge( const PixelCluster& iCluster );

        void Reset( );


        inline float CalcDistance( const SmtPixel& iPoint ) const
        {
            return std::sqrt( std::pow( iPoint.x - mMean.x, 2 ) + std::pow( iPoint.y - mMean.y, 2 ) );
        }

        inline float CalcDistance( const PixelCluster& iCluster ) const
        {
            return std::sqrt( std::pow( mMean.x - iCluster.mMean.x, 2 ) + std::pow( mMean.y - iCluster.mMean.y, 2 ) );
        }

        cv::Point2f const Mean() const { return mMean; }
        float StdDev() const { return mStdDev; }
        int Size() const { return (const int) mSize; }
        std::vector< SmtPixel > Points() const { return mPoints; }

    private:
        cv::Point2f                 mMean;
        float                       mStdDev;
        std::vector< SmtPixel >   mPoints;
        int                         mSize;
        cv::Point2f                 mCenterSum;

        void CalculateStdDev();

};

#endif // PixelCluster_h
