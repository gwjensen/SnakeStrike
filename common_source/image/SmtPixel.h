#ifndef SmtPixel_h
#define SmtPixel_h

#include <opencv2/core.hpp>

class SmtPixel : public cv::Point2d
{
    public:
        SmtPixel( int iX, int iY, int iCam );
        SmtPixel( cv::Point2d   iLocation,
                  int           iCam );

        virtual ~SmtPixel(){}

        inline float CalcDistance( const SmtPixel& iPoint) const
        {
            return std::sqrt( std::pow( iPoint.x - this->x, 2 ) + std::pow( iPoint.y - this->y, 2 ) );
        }

        bool    isValid()  const    { return mValid; }
        bool&    rIsValid()          { return mValid; }
        int     Cam()      const    { return mCam; }
        int&    rCam()              { return mCam; }


    private:
        int mCam;
        bool mValid;

        SmtPixel(){}
};

#endif // SmtPixel_h

