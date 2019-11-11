---
tags:
  - function
overloads:
  double getShortestDistance(cv::Point2d, cv::Point2d):
    description: __MISSING__
    arguments:
      - description: __OPTIONAL__
        name: iPoint1
        type: cv::Point2d
      - name: iPoint2
        description: __OPTIONAL__
        type: cv::Point2d
    return: __OPTIONAL__
    signature_with_names: double getShortestDistance(cv::Point2d iPoint1, cv::Point2d iPoint2)
  double getShortestDistance(cv::Point3d, cv::Point3d, cv::Point3d, cv::Point3d, float):
    signature_with_names: double getShortestDistance(cv::Point3d line1_start, cv::Point3d line1_end, cv::Point3d line2_start, cv::Point3d line2_end, float iExtendLength)
    description: __MISSING__
    arguments:
      - description: __OPTIONAL__
        name: line1_start
        type: cv::Point3d
      - description: __OPTIONAL__
        name: line1_end
        type: cv::Point3d
      - description: __OPTIONAL__
        name: line2_start
        type: cv::Point3d
      - description: __OPTIONAL__
        name: line2_end
        type: cv::Point3d
      - description: __OPTIONAL__
        name: iExtendLength
        type: float
    return: __OPTIONAL__
brief: Get the shortest distance between two points or line segments in 3d space.
defined-in-file: "distance_3d_seg.h"
layout: function
owner: gwjensen
title: getShortestDistance
---
