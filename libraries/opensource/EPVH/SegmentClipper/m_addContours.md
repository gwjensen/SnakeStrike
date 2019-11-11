---
owner: gwjensen
layout: method
title: addContours
defined-in-file: ""
overloads:
  void addContours(std::vector<std::vector<cv::Point2f>> &):
    return: __OPTIONAL__
    arguments:
      - type: std::vector<std::vector<cv::Point2f>> &
        name: contours
        description: __OPTIONAL__
    description: __MISSING__
    signature_with_names: void addContours(std::vector<std::vector<cv::Point2f>> & contours)
  void addContours(std::vector<std::vector<Eigen::Vector2d>> &):
    arguments:
      - name: contours
        type: std::vector<std::vector<Eigen::Vector2d>> &
        description: __OPTIONAL__
    return: __OPTIONAL__
    signature_with_names: void addContours(std::vector<std::vector<Eigen::Vector2d>> & contours)
    description: __MISSING__
  void addContours(std::vector<std::vector<cv::Point>> &):
    description: __MISSING__
    arguments:
      - description: __OPTIONAL__
        type: std::vector<std::vector<cv::Point>> &
        name: contours
    signature_with_names: void addContours(std::vector<std::vector<cv::Point>> & contours)
    return: __OPTIONAL__
tags:
  - method
brief: __MISSING__
---
