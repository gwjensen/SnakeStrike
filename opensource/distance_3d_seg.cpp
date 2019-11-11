#include "distance_3d_seg.h"

namespace opensource {
cv::Point3d extendRay(const cv::Point3d& line_start, const cv::Point3d& line_end, float iExtendLength )
{
    cv::Point3d tmp21(line_end.x - line_start.x,
                  line_end.y - line_start.y,
                  line_end.z - line_start.z);

    //Extending the starting point by a unit vector times iExtendLength
    double sum = sqrt(pow(tmp21.x,2) + pow(tmp21.y,2) + pow(tmp21.z,2));
    //Getting a scaled unit vector
    cv::Point3d line_end_new( tmp21.x / sum * iExtendLength, tmp21.y / sum * iExtendLength, tmp21.z / sum * iExtendLength );
    //Adding the scaled unit vector to the inital starting point to get a much longer segment.
    //line_end_new = line_end_new + line_start;
    //fprintf( stderr, "extendRay::  old(%lf,%lf,%lf) new(%lf,%lf,%lf)\n", line_end.x, line_end.y, line_end.z, line_end_new.x, line_end_new.y, line_end_new.z );
    return line_end_new;
}

//Only function of Softsurfer, others are written by GWJ
//Extends line in direction of second point by the value specified by iExtendLength
double getShortestDistance(cv::Point3d line1_start,
                                         cv::Point3d line1_end,
                                         cv::Point3d line2_start,
                                         cv::Point3d line2_end,
                                         float iExtendLength )
{

    double EPS = 0.00000001;

    //If we don't extend the ray so that the rays have the possibility to cross each other, we will only get the closest segment distance.
    cv::Point3d line1_end_new;
    cv::Point3d line2_end_new;
    if( iExtendLength > 1 ){
        line1_end_new = extendRay( line1_start, line1_end, iExtendLength );
        line2_end_new = extendRay( line2_start, line2_end, iExtendLength );
    }
    else{
        line1_end_new = line1_end;
        line2_end_new = line2_end;
    }

    cv::Point3d delta21(line1_end_new.x - line1_start.x,
                    line1_end_new.y - line1_start.y,
                    line1_end_new.z - line1_start.z);

    cv::Point3d delta41(line2_end_new.x - line2_start.x,
                    line2_end_new.y - line2_start.y,
                    line2_end_new.z - line2_start.z);


    cv::Point3d delta13(line1_start.x - line2_start.x,
                    line1_start.y - line2_start.y,
                    line1_start.z - line2_start.z);

    double a = cv::Mat(delta21).dot( cv::Mat(delta21) );
    double b = cv::Mat(delta21).dot( cv::Mat(delta41) );
    double c = cv::Mat(delta41).dot( cv::Mat(delta41) );
    double d = cv::Mat(delta21).dot( cv::Mat(delta13) );
    double e = cv::Mat(delta41).dot( cv::Mat(delta13) );
    double D = a * c - b * b;

    double sc, sN, sD = D;
    double tc, tN, tD = D;

    if (D < EPS)
    {
        sN = 0.0;
        sD = 1.0;
        tN = e;
        tD = c;
    }
    else
    {
        sN = (b * e - c * d);
        tN = (a * e - b * d);
        if (sN < 0.0)
        {
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD)
        {
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0)
    {
        tN = 0.0;

        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else
        {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD)
    {
        tN = tD;
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else
        {
            sN = (-d + b);
            sD = a;
        }
    }

    if (abs(sN) < EPS) sc = 0.0;
    else sc = sN / sD;
    if (abs(tN) < EPS) tc = 0.0;
    else tc = tN / tD;

    cv::Point3d dP(delta13.x + (sc * delta21.x) - (tc * delta41.x),
                delta13.y + (sc * delta21.y) - (tc * delta41.y),
                delta13.z + (sc * delta21.z) - (tc * delta41.z));

    //return Math.Sqrt(dot(dP, dP));
    return cv::norm(dP);
}

double getShortestDistance( cv::Point2d iPoint1, cv::Point2d iPoint2 ){
    return sqrt( pow( iPoint1.x - iPoint2.x, 2) + pow( iPoint1.y - iPoint2.y, 2) );
}

} //namespace opensource
