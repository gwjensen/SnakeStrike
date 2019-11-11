---
overloads:
  void Threshold(const cv::Scalar &, const cv::Scalar &, const cv::Scalar &):
    return: __OPTIONAL__
    description:
    arguments:
      - name: iLowerb
        type: const cv::Scalar &
        description: __OPTIONAL__
      - type: const cv::Scalar &
        name: iUpperb
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: const cv::Scalar &
        name: iNoiseThresholdInfo
    signature_with_names: void Threshold(const cv::Scalar & iLowerb, const cv::Scalar & iUpperb, const cv::Scalar & iNoiseThresholdInfo)
  void Threshold(const uint32_t, const uint32_t, const uint32_t, const uint32_t, const cv::Scalar &, const cv::Scalar &, cv::Mat &):
    arguments:
      - type: const uint32_t
        description: __OPTIONAL__
        name: iFilterSize
      - type: const uint32_t
        name: iNumIterations
        description: __OPTIONAL__
      - name: iMaxNumPoints
        type: const uint32_t
        description: __OPTIONAL__
      - name: iLowerNoiseThreshold
        type: const uint32_t
        description: __OPTIONAL__
      - type: const cv::Scalar &
        name: iLeftb
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: iRightb
        type: const cv::Scalar &
      - description: __OPTIONAL__
        type: cv::Mat &
        name: oMat
    description:
    return: __OPTIONAL__
    signature_with_names: void Threshold(const uint32_t iFilterSize, const uint32_t iNumIterations, const uint32_t iMaxNumPoints, const uint32_t iLowerNoiseThreshold, const cv::Scalar & iLeftb, const cv::Scalar & iRightb, cv::Mat & oMat)
defined-in-file: "image/SmtImage.h"
layout: method
brief: Threshold the colors in an image based on the passed in boundary conditions. The values for the lower bound (iLowerb) and the upper bound (iUpperb) are based on HSV. iFilterSize, iNumIterations, and iLowerNoiseThreshold are used for smoothing the thresholded image to try to decrease the effect that noise can have on creating clusters of pixels.
owner: gwjensen
tags:
  - method
title: Threshold
---
