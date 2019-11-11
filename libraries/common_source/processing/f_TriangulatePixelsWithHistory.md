---
tags:
  - function
owner: gwjensen
layout: function
brief: Triangulate the matched pixels for each timestep, but use the information about previous steps to inform the triangulation and make it more smooth in the presence of errors. (Not currently working)
defined-in-file: "processing/Triangulation.h"
title: TriangulatePixelsWithHistory
overloads:
  int TriangulatePixelsWithHistory(const PixelSet &, const CamMats &, std::vector<std::vector<cv::Point3d>> &):
    arguments:
      - description: __OPTIONAL__
        name: iPixelSet
        type: const PixelSet &
      - description: __OPTIONAL__
        type: const CamMats &
        name: iCamMats
      - description: __OPTIONAL__
        name: oCalcdPoints
        type: std::vector<std::vector<cv::Point3d>> &
    return: __OPTIONAL__
    signature_with_names: int TriangulatePixelsWithHistory(const PixelSet & iPixelSet, const CamMats & iCamMats, std::vector<std::vector<cv::Point3d>> & oCalcdPoints)
    description:
---
