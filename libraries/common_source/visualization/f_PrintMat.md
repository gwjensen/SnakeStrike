---
overloads:
  void PrintMat(const cv::Mat &):
    description: __MISSING__
    arguments:
      - description: __OPTIONAL__
        name: iMat
        type: const cv::Mat &
    return: __OPTIONAL__
    signature_with_names: void PrintMat(const cv::Mat & iMat)
  void PrintMat(const cv::Mat &, int):
    description: __MISSING__
    return: __OPTIONAL__
    signature_with_names: void PrintMat(const cv::Mat & iMat, int iNumRows)
    arguments:
      - description: __OPTIONAL__
        name: iMat
        type: const cv::Mat &
      - type: int
        description: __OPTIONAL__
        name: iNumRows
owner: gwjensen
title: PrintMat
defined-in-file: "visualization/opencv_viz.h"
tags:
  - function
brief: A pretty print for the contents of a matrix.
layout: function
---
