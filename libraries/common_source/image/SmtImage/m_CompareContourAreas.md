---
layout: method
overloads:
  static bool CompareContourAreas(std::vector<cv::Point>, std::vector<cv::Point>):
    return: __OPTIONAL__
    signature_with_names: static bool CompareContourAreas(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2)
    description:
    arguments:
      - name: contour1
        description: __OPTIONAL__
        type: std::vector<cv::Point>
      - name: contour2
        type: std::vector<cv::Point>
        description: __OPTIONAL__
tags:
  - method
owner: gwjensen
defined-in-file: "image/SmtImage.h"
title: CompareContourAreas
brief: Compare the sizes of two contour areas to determine which one is bigger. This is used for ordering lists contour areas.
---
