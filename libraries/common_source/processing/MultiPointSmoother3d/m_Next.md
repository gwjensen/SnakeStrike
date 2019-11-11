---
overloads:
  bool Next(const std::vector<cv::Point3d> &, uint32_t, std::vector<cv::Point3d> &):
    return: __OPTIONAL__
    signature_with_names: bool Next(const std::vector<cv::Point3d> & iMeasuredPoints, uint32_t iTimestep, std::vector<cv::Point3d> & oSmoothedPoints)
    description:
    arguments:
      - type: const std::vector<cv::Point3d> &
        description: __OPTIONAL__
        name: iMeasuredPoints
      - description: __OPTIONAL__
        name: iTimestep
        type: uint32_t
      - type: std::vector<cv::Point3d> &
        name: oSmoothedPoints
        description: __OPTIONAL__
defined-in-file: "processing/MultiPointSmoother3d.h"
title: Next
owner: gwjensen
layout: method
tags:
  - method
brief: Get a prediction for the next timestep.
---
