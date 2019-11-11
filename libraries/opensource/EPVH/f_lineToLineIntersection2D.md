---
defined-in-file: ""
tags:
  - function
overloads:
  float lineToLineIntersection2D(tr::Point2 &, tr::Point2 &, tr::Point2 &, tr::Point2 &, tr::Point2 &):
    arguments:
      - description: __OPTIONAL__
        name: point1
        type: tr::Point2 &
      - description: __OPTIONAL__
        name: point2
        type: tr::Point2 &
      - name: point3
        description: __OPTIONAL__
        type: tr::Point2 &
      - description: __OPTIONAL__
        name: point4
        type: tr::Point2 &
      - description: __OPTIONAL__
        name: intersection
        type: tr::Point2 &
    return: __OPTIONAL__
    description: __MISSING__
    signature_with_names: float lineToLineIntersection2D(tr::Point2 & point1, tr::Point2 & point2, tr::Point2 & point3, tr::Point2 & point4, tr::Point2 & intersection)
  double lineToLineIntersection2D(Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d, std::pair<double, double> &):
    arguments:
      - type: Eigen::Vector2d
        description: __OPTIONAL__
        name: point1
      - type: Eigen::Vector2d
        description: __OPTIONAL__
        name: point2
      - type: Eigen::Vector2d
        name: point3
        description: __OPTIONAL__
      - type: Eigen::Vector2d
        name: point4
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: coefficients
        type: std::pair<double, double> &
    return: __OPTIONAL__
    description: __MISSING__
    signature_with_names: double lineToLineIntersection2D(Eigen::Vector2d point1, Eigen::Vector2d point2, Eigen::Vector2d point3, Eigen::Vector2d point4, std::pair<double, double> & coefficients)
  void lineToLineIntersection2D(cv::Point2f, cv::Point2f, cv::Point2f, cv::Point2f, cv::Point2f &, std::pair<double, double> &):
    return: __OPTIONAL__
    arguments:
      - name: point1
        type: cv::Point2f
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: point2
        type: cv::Point2f
      - type: cv::Point2f
        name: point3
        description: __OPTIONAL__
      - type: cv::Point2f
        description: __OPTIONAL__
        name: point4
      - name: intersection
        description: __OPTIONAL__
        type: cv::Point2f &
      - type: std::pair<double, double> &
        description: __OPTIONAL__
        name: coefficients
    signature_with_names: void lineToLineIntersection2D(cv::Point2f point1, cv::Point2f point2, cv::Point2f point3, cv::Point2f point4, cv::Point2f & intersection, std::pair<double, double> & coefficients)
    description: __MISSING__
  float lineToLineIntersection2D(tr::Point2f, tr::Point2f, tr::Point2f, tr::Point2f, std::pair<float, float> &):
    description: __MISSING__
    arguments:
      - name: point1
        description: __OPTIONAL__
        type: tr::Point2f
      - type: tr::Point2f
        description: __OPTIONAL__
        name: point2
      - description: __OPTIONAL__
        type: tr::Point2f
        name: point3
      - name: point4
        description: __OPTIONAL__
        type: tr::Point2f
      - description: __OPTIONAL__
        name: coefficients
        type: std::pair<float, float> &
    signature_with_names: float lineToLineIntersection2D(tr::Point2f point1, tr::Point2f point2, tr::Point2f point3, tr::Point2f point4, std::pair<float, float> & coefficients)
    return: __OPTIONAL__
  void lineToLineIntersection2D(tr::Point2 &, tr::Point2 &, tr::Point2 &, tr::Point2 &, tr::Point2 &, std::pair<float, float> &):
    description: __MISSING__
    signature_with_names: void lineToLineIntersection2D(tr::Point2 & point1, tr::Point2 & point2, tr::Point2 & point3, tr::Point2 & point4, tr::Point2 & intersection, std::pair<float, float> & coefficients)
    return: __OPTIONAL__
    arguments:
      - type: tr::Point2 &
        name: point1
        description: __OPTIONAL__
      - type: tr::Point2 &
        description: __OPTIONAL__
        name: point2
      - name: point3
        description: __OPTIONAL__
        type: tr::Point2 &
      - description: __OPTIONAL__
        name: point4
        type: tr::Point2 &
      - name: intersection
        type: tr::Point2 &
        description: __OPTIONAL__
      - name: coefficients
        type: std::pair<float, float> &
        description: __OPTIONAL__
  float lineToLineIntersection2D(cv::Point2f, cv::Point2f, cv::Point2f, cv::Point2f, cv::Point2f &):
    arguments:
      - name: point1
        type: cv::Point2f
        description: __OPTIONAL__
      - type: cv::Point2f
        name: point2
        description: __OPTIONAL__
      - type: cv::Point2f
        description: __OPTIONAL__
        name: point3
      - type: cv::Point2f
        name: point4
        description: __OPTIONAL__
      - name: intersection
        description: __OPTIONAL__
        type: cv::Point2f &
    description: __MISSING__
    return: __OPTIONAL__
    signature_with_names: float lineToLineIntersection2D(cv::Point2f point1, cv::Point2f point2, cv::Point2f point3, cv::Point2f point4, cv::Point2f & intersection)
  void lineToLineIntersection2D(Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d &, std::pair<double, double> &):
    arguments:
      - name: point1
        description: __OPTIONAL__
        type: Eigen::Vector2d
      - description: __OPTIONAL__
        type: Eigen::Vector2d
        name: point2
      - name: point3
        description: __OPTIONAL__
        type: Eigen::Vector2d
      - type: Eigen::Vector2d
        name: point4
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: Eigen::Vector2d &
        name: intersection
      - type: std::pair<double, double> &
        description: __OPTIONAL__
        name: coefficients
    description: __MISSING__
    return: __OPTIONAL__
    signature_with_names: void lineToLineIntersection2D(Eigen::Vector2d point1, Eigen::Vector2d point2, Eigen::Vector2d point3, Eigen::Vector2d point4, Eigen::Vector2d & intersection, std::pair<double, double> & coefficients)
  void lineToLineIntersection2D(cv::Point2f, cv::Point2f, cv::Point2f, cv::Point2f, cv::Point2f &, std::pair<float, float> &):
    description: __MISSING__
    arguments:
      - name: point1
        type: cv::Point2f
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: point2
        type: cv::Point2f
      - name: point3
        description: __OPTIONAL__
        type: cv::Point2f
      - description: __OPTIONAL__
        name: point4
        type: cv::Point2f
      - name: intersection
        description: __OPTIONAL__
        type: cv::Point2f &
      - name: coefficients
        type: std::pair<float, float> &
        description: __OPTIONAL__
    return: __OPTIONAL__
    signature_with_names: void lineToLineIntersection2D(cv::Point2f point1, cv::Point2f point2, cv::Point2f point3, cv::Point2f point4, cv::Point2f & intersection, std::pair<float, float> & coefficients)
title: lineToLineIntersection2D
layout: function
brief: __MISSING__
owner: gwjensen
---
