---
tags:
  - method
layout: method
owner: gwjensen
overloads:
  static bool CompareContourAreas(std::vector<cv::Point>, std::vector<cv::Point>):
    return: __OPTIONAL__
    arguments:
      - type: std::vector<cv::Point>
        name: contour1
        description: __OPTIONAL__
      - name: contour2
        type: std::vector<cv::Point>
        description: __OPTIONAL__
    annotation:
      - private
    description:
    signature_with_names: static bool CompareContourAreas(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2)
title: CompareContourAreas
defined-in-file: "TriangulationPipeline.h"
brief: Function for comparing the hulls created by two groups of points to find out whether one is bigger than the other.
---
