#include "PixelCluster.h"

PixelCluster::PixelCluster()
{
    Reset();
}

PixelCluster::~PixelCluster()
{}

void PixelCluster::Add( const SmtPixel& iPoint )
{
    mPoints.push_back( iPoint );
    ++mSize;
    mCenterSum.x = mCenterSum.x + iPoint.x;
    mCenterSum.y = mCenterSum.y + iPoint.y;
    mMean.x = mCenterSum.x / mSize;
    mMean.y = mCenterSum.y / mSize;
    CalculateStdDev();
}

void PixelCluster::Remove( const SmtPixel& iPoint )
{
    std::vector< SmtPixel >::iterator point_location = find( mPoints.begin(),
                                                               mPoints.end(),
                                                               iPoint);
    if (mPoints.end() != point_location)
    {
        mPoints.erase( point_location );
        --mSize;
        mCenterSum.x = mCenterSum.x - iPoint.x;
        mCenterSum.y = mCenterSum.y - iPoint.y;
        mMean.x = mCenterSum.x / mSize;
        mMean.y = mCenterSum.y / mSize;
        CalculateStdDev();
    }
}
void PixelCluster::Merge( const PixelCluster& iCluster )
{
    for (int i =0; i < iCluster.mSize; ++i){
        Add( iCluster.mPoints[i] );
    }
}

void PixelCluster::Reset( )
{
    mPoints.clear();
    mMean.x = 0;
    mMean.y = 0;
    mStdDev = 0.0;
    mSize = mPoints.size();
    mCenterSum.x = 0;
    mCenterSum.y = 0;
}

void PixelCluster::CalculateStdDev()
{
    float sum_of_distances = 0;
    for (int i = 0; i < mSize; ++i)
    {
        sum_of_distances = sum_of_distances + pow( mPoints[i].x - mMean.x, 2 ) + pow( mPoints[i].y - mMean.y, 2 );
    }
    if (mSize < 2)
    {
        mStdDev = 0;
    }
    else
    {
        mStdDev = sqrt( (1.0 / (mSize - 1)) * sum_of_distances );
    }
}



