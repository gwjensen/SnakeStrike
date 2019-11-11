---
layout: method
tags:
  - method
title: GetPredictedStates
brief: Get the next predicted locations for the points that are being tracked.
owner: gwjensen
defined-in-file: "processing/MultiPointTracker.h"
overloads:
  bool GetPredictedStates(const uint32_t, std::vector<cv::Point2d> &):
    annotation:
      - protected
    signature_with_names: bool GetPredictedStates(const uint32_t iTimestep, std::vector<cv::Point2d> & oPredictedLocations)
    return: __OPTIONAL__
    arguments:
      - type: const uint32_t
        name: iTimestep
        description: __OPTIONAL__
      - name: oPredictedLocations
        description: __OPTIONAL__
        type: std::vector<cv::Point2d> &
    description:
---
