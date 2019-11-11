---
brief: A kalman filter implementation for tracking in 2D.
layout: class
declaration: "\nclass MultiPointTracker;"
tags:
  - class
fields:
  mKFilterPtrs:
    type: std::vector<cv::KalmanFilter *>
    annotation:
      - protected
    description: __MISSING__
  mKFilterTimestep:
    annotation:
      - protected
    description: __MISSING__
    type: std::vector<uint32_t>
  mNumPoints:
    type: uint8_t
    description: __MISSING__
    annotation:
      - protected
  mNumCams:
    description: __MISSING__
    annotation:
      - protected
    type: uint8_t
owner: gwjensen
defined-in-file: "processing/MultiPointTracker.h"
title: MultiPointTracker
---
