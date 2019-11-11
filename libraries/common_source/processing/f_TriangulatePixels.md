---
brief: Triangulates the matched pixel set it is given and returns the triangulated 3d points sorted into timesteps.
tags:
  - function
owner: gwjensen
defined-in-file: "processing/Triangulation.h"
overloads:
  int TriangulatePixels(const PixelSet &, std::vector<std::vector<cv::Point3d>> &):
    signature_with_names: int TriangulatePixels(const PixelSet & iPixelSet, std::vector<std::vector<cv::Point3d>> & oCalcdPoints)
    arguments:
      - description: __OPTIONAL__
        name: iPixelSet
        type: const PixelSet &
      - type: std::vector<std::vector<cv::Point3d>> &
        name: oCalcdPoints
        description: __OPTIONAL__
    return: __OPTIONAL__
    description:
title: TriangulatePixels
layout: function
---
