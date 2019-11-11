#include "SmtPixel.h"

SmtPixel::SmtPixel( cv::Point2d iLocation,
                    int         iCam )
    :cv::Point2d( iLocation.x,
                  iLocation.y ),
     mCam( iCam ),
     mValid( true )
{
     //std::fprintf(stderr, "SmtPixel():New pixel created for location (%f, %f) in cam/image %d\n", iLocation.x, iLocation.y, iCam);
}

SmtPixel::SmtPixel( int iX, int iY, int iCam )
    :cv::Point2d( iX, iY),
      mCam(iCam),
      mValid(true)
{

}

