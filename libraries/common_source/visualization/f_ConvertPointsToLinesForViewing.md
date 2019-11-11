---
layout: function
brief: Visualization of the camera(s) pose in world coordinates.
owner: gwjensen
defined-in-file: "visualization/pcl_viz.h"
tags:
  - function
title: ConvertPointsToLinesForViewing
overloads:
  void ConvertPointsToLinesForViewing(const PixelSet &, const CamMats *const, const uint64_t):
    arguments:
      - description: __OPTIONAL__
        name: iPixelSet
        type: const PixelSet &
      - description: __OPTIONAL__
        name: iCamMatrix
        type: const CamMats *const
      - description: __OPTIONAL__
        name: iNumTimesteps
        type: const uint64_t
    signature_with_names: void ConvertPointsToLinesForViewing(const PixelSet & iPixelSet, const CamMats *const iCamMatrix, const uint64_t iNumTimesteps)
    description:
    return: __OPTIONAL__
---
