---
overloads:
  void TriangulatePixelsWithKalman(const PixelSet &, const CamMats &, const std::pair<int, std::set<int>> &, MultiPointSmoother3d &, const int, std::vector<std::vector<cv::Point3d>> &):
    return: __OPTIONAL__
    signature_with_names: void TriangulatePixelsWithKalman(const PixelSet & iPixelSet, const CamMats & iCamMats, const std::pair<int, std::set<int>> & iCamIndexesToExclude, MultiPointSmoother3d & ioTracker, const int iNumPoints, std::vector<std::vector<cv::Point3d>> & oCalcdPoints)
    arguments:
      - description: __OPTIONAL__
        type: const PixelSet &
        name: iPixelSet
      - description: __OPTIONAL__
        name: iCamMats
        type: const CamMats &
      - type: const std::pair<int, std::set<int>> &
        description: __OPTIONAL__
        name: iCamIndexesToExclude
      - type: MultiPointSmoother3d &
        description: __OPTIONAL__
        name: ioTracker
      - description: __OPTIONAL__
        type: const int
        name: iNumPoints
      - description: __OPTIONAL__
        name: oCalcdPoints
        type: std::vector<std::vector<cv::Point3d>> &
    description: __MISSING__
owner: gwjensen
title: TriangulatePixelsWithKalman
tags:
  - function
defined-in-file: "processing/Triangulation.h"
layout: function
brief: Triangulate the matched pixels for each timestep, but use a kalman filter to inform the triangulation and make it more smooth in the presence of errors.
---
