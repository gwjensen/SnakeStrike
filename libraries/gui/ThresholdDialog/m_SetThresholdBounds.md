---
brief: Set the left and right bounds for the thresholding.
defined-in-file: "projects/gui/ThresholdDialog.h"
owner: gwjensen
tags:
  - method
layout: method
title: SetThresholdBounds
overloads:
  void SetThresholdBounds(cv::Scalar, cv::Scalar):
    signature_with_names: void SetThresholdBounds(cv::Scalar iLeftBound, cv::Scalar iRightBound)
    arguments:
      - type: cv::Scalar
        description: __OPTIONAL__
        name: iLeftBound
      - description: __OPTIONAL__
        type: cv::Scalar
        name: iRightBound
    annotation:
      - private
    description: __MISSING__
    return: __OPTIONAL__
  void SetThresholdBounds(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t):
    signature_with_names: void SetThresholdBounds(uint32_t iLH, uint32_t iLS, uint32_t iLV, uint32_t iRH, uint32_t iRS, uint32_t iRV)
    return: __OPTIONAL__
    description: __MISSING__
    annotation:
      - private
    arguments:
      - description: __OPTIONAL__
        type: uint32_t
        name: iLH
      - description: __OPTIONAL__
        name: iLS
        type: uint32_t
      - description: __OPTIONAL__
        type: uint32_t
        name: iLV
      - description: __OPTIONAL__
        type: uint32_t
        name: iRH
      - description: __OPTIONAL__
        name: iRS
        type: uint32_t
      - description: __OPTIONAL__
        name: iRV
        type: uint32_t
---
