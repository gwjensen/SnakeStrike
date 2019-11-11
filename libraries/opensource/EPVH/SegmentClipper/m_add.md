---
brief: __MISSING__
defined-in-file: ""
tags:
  - method
title: add
overloads:
  void add(int, int, Eigen::Vector2d, Eigen::Vector2d):
    signature_with_names: void add(int contourId, int segmentId, Eigen::Vector2d end1, Eigen::Vector2d end2)
    return: __OPTIONAL__
    arguments:
      - description: __OPTIONAL__
        name: contourId
        type: int
      - description: __OPTIONAL__
        name: segmentId
        type: int
      - type: Eigen::Vector2d
        name: end1
        description: __OPTIONAL__
      - name: end2
        type: Eigen::Vector2d
        description: __OPTIONAL__
    description: __MISSING__
  void add(int, int, cv::Point2f, cv::Point2f):
    signature_with_names: void add(int contourId, int segmentId, cv::Point2f end1, cv::Point2f end2)
    description: __MISSING__
    return: __OPTIONAL__
    arguments:
      - name: contourId
        type: int
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: segmentId
        type: int
      - type: cv::Point2f
        description: __OPTIONAL__
        name: end1
      - type: cv::Point2f
        description: __OPTIONAL__
        name: end2
layout: method
owner: gwjensen
---
