---
title: ThresholdImages
defined-in-file: "image/ImageSet.h"
layout: function
overloads:
  void ThresholdImages(const ImageSet &, ImageSet &, bool (*)(), const cv::Scalar &, const cv::Scalar &, const int, const int, const int):
    return: __OPTIONAL__
    arguments:
      - type: const ImageSet &
        name: iImagesSet
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: ImageSet &
        name: oImagesSet
      - description: __OPTIONAL__
        name: CancelFunc
        type: bool (*)()
      - name: iLowerb
        type: const cv::Scalar &
        description: __OPTIONAL__
      - name: iUpperb
        type: const cv::Scalar &
        description: __OPTIONAL__
      - name: noiseFilterSize
        type: const int
        description: __OPTIONAL__
      - type: const int
        description: __OPTIONAL__
        name: noiseIterations
      - name: noiseThreshold
        type: const int
        description: __OPTIONAL__
    signature_with_names: void ThresholdImages(const ImageSet & iImagesSet, ImageSet & oImagesSet, bool (*)() CancelFunc, const cv::Scalar & iLowerb, const cv::Scalar & iUpperb, const int noiseFilterSize, const int noiseIterations, const int noiseThreshold)
    description: __MISSING__
tags:
  - function
brief: Threshold the images using the passed in constraints. These values are typically passed in from a gui or some other easier way for the user to specify the values.
owner: gwjensen
---
