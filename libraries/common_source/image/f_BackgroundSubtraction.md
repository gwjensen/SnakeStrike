---
brief: Background subtraction using mixture of gaussians. Don't use this.
tags:
  - function
layout: function
title: BackgroundSubtraction
owner: gwjensen
overloads:
  void BackgroundSubtraction(const uint32_t):
    description: __MISSING__
    arguments:
      - type: const uint32_t
        description: __OPTIONAL__
        name: iCamIdx
    signature_with_names: void BackgroundSubtraction(const uint32_t iCamIdx)
    return: __OPTIONAL__
  void BackgroundSubtraction(const uint32_t, cv::Mat &):
    arguments:
      - description: __OPTIONAL__
        type: const uint32_t
        name: iCamIdx
      - type: cv::Mat &
        description: __OPTIONAL__
        name: oMask
    signature_with_names: void BackgroundSubtraction(const uint32_t iCamIdx, cv::Mat & oMask)
    return: __OPTIONAL__
    description: __MISSING__
defined-in-file: "image/ImageSet.h"
---
