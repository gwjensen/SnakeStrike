/*
* Converted to C# by John Burns from C++ sourced from:
* http://softsurfer.com/Archive/algorithm_0106/algorithm_0106.htm
*
* Original C++ Copyright Notice
* ===================
* Copyright 2001, softSurfer (www.softsurfer.com)
* This code may be freely used and modified for any purpose
* providing that this copyright notice is included with it.
* SoftSurfer makes no warranty for this code, and cannot be held
* liable for any real or imagined damage resulting from its use.
* Users of this code must verify correctness for their application.
*//* Code modified by Grady Jensen in 2017 to work with OpenCV and with the ability to use it for Rays instead of segments. */

#ifndef OPENSOURCE_DISTANCE_3D_SEG_H
#define OPENSOURCE_DISTANCE_3D_SEG_H

#include <opencv2/core.hpp>

 
namespace opensource{
    cv::Point3d extendRay(const cv::Point3d& line_start,
                          const cv::Point3d& line_end,
                          float iExtendLength );
	 
	//Only function of Softsurfer, others are written by GWJ 
	//Extends line in direction of second point by the value specified by iExtendLength
    double getShortestDistance(cv::Point3d line1_start,
                               cv::Point3d line1_end,
                               cv::Point3d line2_start,
                               cv::Point3d line2_end,
                               float iExtendLength = 1000);

    double getShortestDistance( cv::Point2d iPoint1, cv::Point2d iPoint2 );
	
} //namespace opensource	
#endif //OPENSOURCE_DISTANCE_3D_SEG_H
