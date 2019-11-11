---
owner: gwjensen
title: triangleArea
tags:
  - function
brief: __MISSING__
layout: function
overloads:
  double triangleArea(Eigen::Vector2f &, Eigen::Vector2f &, Eigen::Vector2f &):
    description: __MISSING__
    return: __OPTIONAL__
    arguments:
      - description: __OPTIONAL__
        type: Eigen::Vector2f &
        name: pt1
      - description: __OPTIONAL__
        name: pt2
        type: Eigen::Vector2f &
      - name: pt3
        description: __OPTIONAL__
        type: Eigen::Vector2f &
    signature_with_names: double triangleArea(Eigen::Vector2f & pt1, Eigen::Vector2f & pt2, Eigen::Vector2f & pt3)
  double triangleArea(tr::Point2 &, tr::Point2 &, tr::Point2 &):
    return: __OPTIONAL__
    signature_with_names: double triangleArea(tr::Point2 & pt1, tr::Point2 & pt2, tr::Point2 & pt3)
    description: __MISSING__
    arguments:
      - description: __OPTIONAL__
        name: pt1
        type: tr::Point2 &
      - name: pt2
        type: tr::Point2 &
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: tr::Point2 &
        name: pt3
  double triangleArea(cv::Point2f, cv::Point2f, cv::Point2f):
    description: __MISSING__
    return: __OPTIONAL__
    arguments:
      - type: cv::Point2f
        description: __OPTIONAL__
        name: pt1
      - description: __OPTIONAL__
        name: pt2
        type: cv::Point2f
      - name: pt3
        type: cv::Point2f
        description: __OPTIONAL__
    signature_with_names: double triangleArea(cv::Point2f pt1, cv::Point2f pt2, cv::Point2f pt3)
defined-in-file: ""
---
