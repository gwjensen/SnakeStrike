---
owner: gwjensen
tags:
  - function
title: CorrectTimestepPoints
overloads:
  double CorrectTimestepPoints(const std::vector<cv::Mat> &, const std::vector<cv::Mat> &, const std::pair<int, std::set<int>> &, std::vector<cv::Mat> &):
    arguments:
      - name: iCamProj
        type: const std::vector<cv::Mat> &
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: iPoints
        type: const std::vector<cv::Mat> &
      - type: const std::pair<int, std::set<int>> &
        name: iCamIndexesToExclude
        description: __OPTIONAL__
      - name: oPoints
        description: __OPTIONAL__
        type: std::vector<cv::Mat> &
    description: __MISSING__
    signature_with_names: double CorrectTimestepPoints(const std::vector<cv::Mat> & iCamProj, const std::vector<cv::Mat> & iPoints, const std::pair<int, std::set<int>> & iCamIndexesToExclude, std::vector<cv::Mat> & oPoints)
    return: __OPTIONAL__
  double CorrectTimestepPoints(const std::vector<cv::Mat> &, const std::vector<cv::Mat> &, std::vector<cv::Mat> &):
    arguments:
      - name: iCamProj
        type: const std::vector<cv::Mat> &
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: iPoints
        type: const std::vector<cv::Mat> &
      - name: oPoints
        type: std::vector<cv::Mat> &
        description: __OPTIONAL__
    signature_with_names: double CorrectTimestepPoints(const std::vector<cv::Mat> & iCamProj, const std::vector<cv::Mat> & iPoints, std::vector<cv::Mat> & oPoints)
    description: __MISSING__
    return: __OPTIONAL__
brief: This function does optimal correction of the points in the images according to the matrices of the cameras. If the world was perfect and there was no noise, there would be no need for this step as the epipolar lines from the matching points of each camera's image would intersect in 3d space. However, since there is noise they don't always intersect. This function adjusts the camera pixels in such a way that the minimum amount of pixel movement in order to get the epipolar lines to intersect is achieved.
defined-in-file: "processing/Triangulation.h"
layout: function
---
