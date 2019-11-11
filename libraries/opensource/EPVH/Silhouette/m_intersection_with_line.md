---
owner: gwjensen
layout: method
title: intersection_with_line
tags:
  - method
overloads:
  void intersection_with_line(cv::Vec3f, std::vector<cv::Point2f> &):
    arguments:
      - name: line
        type: cv::Vec3f
        description: __OPTIONAL__
      - name: intersection_points
        type: std::vector<cv::Point2f> &
        description: __OPTIONAL__
    description: __MISSING__
    signature_with_names: void intersection_with_line(cv::Vec3f line, std::vector<cv::Point2f> & intersection_points)
    return: __OPTIONAL__
  void intersection_with_line(cv::Point2f, cv::Point2f, std::vector<cv::Point2f> &):
    arguments:
      - description: __OPTIONAL__
        name: point1
        type: cv::Point2f
      - type: cv::Point2f
        description: __OPTIONAL__
        name: point2
      - type: std::vector<cv::Point2f> &
        name: intersection_points
        description: __OPTIONAL__
    description: __MISSING__
    return: __OPTIONAL__
    signature_with_names: void intersection_with_line(cv::Point2f point1, cv::Point2f point2, std::vector<cv::Point2f> & intersection_points)
brief: __MISSING__
defined-in-file: ""
---
