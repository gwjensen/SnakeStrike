---
tags:
  - method
defined-in-file: "projects/gui/ThresholdDialog.h"
brief: Callback function that gets called by a mouse event and then gives the value of the pixel. If the image that is being shown is a thresholde image, then it will only show the values of the parts of the image that pass the threshold ( i.e. the non-black pixels) and furthermore, it will show the value of that pixel in the original image. This is useful for figuring out why certain pixels are making it past the thresholding and then fine tuning the constraints.
title: UpdateGrabbedPixel
owner: gwjensen
overloads:
  void UpdateGrabbedPixel(uint32_t, uint32_t, uint32_t):
    description: __MISSING__
    arguments:
      - type: uint32_t
        name: iCamIdx
        description: __OPTIONAL__
      - type: uint32_t
        description: __OPTIONAL__
        name: iX
      - type: uint32_t
        name: iY
        description: __OPTIONAL__
    signature_with_names: void UpdateGrabbedPixel(uint32_t iCamIdx, uint32_t iX, uint32_t iY)
    return: __OPTIONAL__
layout: method
---
