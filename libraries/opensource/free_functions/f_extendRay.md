---
title: extendRay
defined-in-file: "distance_3d_seg.h"
tags:
  - function
owner: gwjensen
overloads:
  cv::Point3d extendRay(const cv::Point3d &, const cv::Point3d &, float):
    signature_with_names: cv::Point3d extendRay(const cv::Point3d & line_start, const cv::Point3d & line_end, float iExtendLength)
    return: __OPTIONAL__
    description:
    arguments:
      - description: __OPTIONAL__
        type: const cv::Point3d &
        name: line_start
      - description: __OPTIONAL__
        name: line_end
        type: const cv::Point3d &
      - type: float
        description: __OPTIONAL__
        name: iExtendLength
layout: function
brief: This function takes a ray defined by two points in 3d space and extends the ray by the given amount.
---
