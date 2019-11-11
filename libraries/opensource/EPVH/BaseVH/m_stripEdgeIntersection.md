---
overloads:
  void stripEdgeIntersection(int, int, int, cv::Point2f, cv::Point2f, cv::Point2f &, std::pair<double, double> &):
    return: __OPTIONAL__
    description: __MISSING__
    signature_with_names: void stripEdgeIntersection(int camId, int contourId, int stripId, cv::Point2f end1, cv::Point2f end2, cv::Point2f & intersectionPoint, std::pair<double, double> & coefficientPairs)
    arguments:
      - description: __OPTIONAL__
        name: camId
        type: int
      - type: int
        description: __OPTIONAL__
        name: contourId
      - description: __OPTIONAL__
        name: stripId
        type: int
      - name: end1
        type: cv::Point2f
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: end2
        type: cv::Point2f
      - description: __OPTIONAL__
        type: cv::Point2f &
        name: intersectionPoint
      - name: coefficientPairs
        type: std::pair<double, double> &
        description: __OPTIONAL__
    annotation:
      - protected
  void stripEdgeIntersection(int, int, int, Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d &, std::pair<double, double> &):
    description: __MISSING__
    signature_with_names: void stripEdgeIntersection(int camId, int contourId, int stripId, Eigen::Vector2d end1, Eigen::Vector2d end2, Eigen::Vector2d & intersectionPoint, std::pair<double, double> & coefficientPairs)
    annotation:
      - protected
    arguments:
      - type: int
        description: __OPTIONAL__
        name: camId
      - description: __OPTIONAL__
        name: contourId
        type: int
      - type: int
        description: __OPTIONAL__
        name: stripId
      - type: Eigen::Vector2d
        description: __OPTIONAL__
        name: end1
      - type: Eigen::Vector2d
        name: end2
        description: __OPTIONAL__
      - type: Eigen::Vector2d &
        name: intersectionPoint
        description: __OPTIONAL__
      - name: coefficientPairs
        type: std::pair<double, double> &
        description: __OPTIONAL__
    return: __OPTIONAL__
defined-in-file: ""
tags:
  - method
brief: __MISSING__
title: stripEdgeIntersection
layout: method
owner: gwjensen
---
