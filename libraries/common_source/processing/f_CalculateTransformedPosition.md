---
tags:
  - function
owner: gwjensen
layout: function
brief: Creates a ray with arbitrary specified length from the camera through the image pixel specified.
title: CalculateTransformedPosition
defined-in-file: "processing/Triangulation.h"
overloads:
  void CalculateTransformedPosition(const int &, const std::vector<SmtPixel> &, const CamMats &, cv::Point3d &, std::vector<cv::Point3d> &, float):
    signature_with_names: void CalculateTransformedPosition(const int & iCamIdx, const std::vector<SmtPixel> & iCandidateEndPoints, const CamMats & iCamMatrix, cv::Point3d & oStart, std::vector<cv::Point3d> & oEnds, float iExtendLength)
    arguments:
      - type: const int &
        description: __OPTIONAL__
        name: iCamIdx
      - description: __OPTIONAL__
        type: const std::vector<SmtPixel> &
        name: iCandidateEndPoints
      - type: const CamMats &
        description: __OPTIONAL__
        name: iCamMatrix
      - description: __OPTIONAL__
        name: oStart
        type: cv::Point3d &
      - description: __OPTIONAL__
        name: oEnds
        type: std::vector<cv::Point3d> &
      - description: __OPTIONAL__
        name: iExtendLength
        type: float
    description:
    return: __OPTIONAL__
---
