---
defined-in-file: "projects/gui/GuiImage.h"
layout: method
overloads:
  void ThresholdImage(const uint32_t, const uint32_t, const uint32_t, const uint32_t, const cv::Scalar &, const cv::Scalar &, QPixmap &):
    arguments:
      - description: __OPTIONAL__
        type: const uint32_t
        name: iFilterSize
      - name: iNumIterations
        description: __OPTIONAL__
        type: const uint32_t
      - type: const uint32_t
        name: iMaxNumPoints
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: const uint32_t
        name: iLowerNoiseThreshold
      - type: const cv::Scalar &
        description: __OPTIONAL__
        name: iLowerb
      - type: const cv::Scalar &
        description: __OPTIONAL__
        name: iUpperb
      - description: __OPTIONAL__
        type: QPixmap &
        name: oPixMap
    return: __OPTIONAL__
    signature_with_names: void ThresholdImage(const uint32_t iFilterSize, const uint32_t iNumIterations, const uint32_t iMaxNumPoints, const uint32_t iLowerNoiseThreshold, const cv::Scalar & iLowerb, const cv::Scalar & iUpperb, QPixmap & oPixMap)
    description: __MISSING__
brief: Threshold the image.
tags:
  - method
owner: gwjensen
title: ThresholdImage
---
