---
owner: gwjensen
defined-in-file: ""
brief: __MISSING__
tags:
  - function
layout: function
title: distance
overloads:
  float distance(tr::Point2 &, tr::Point2 &):
    signature_with_names: float distance(tr::Point2 & point1, tr::Point2 & point2)
    return: __OPTIONAL__
    description: __MISSING__
    arguments:
      - name: point1
        description: __OPTIONAL__
        type: tr::Point2 &
      - name: point2
        type: tr::Point2 &
        description: __OPTIONAL__
  float distance(cv::Point2f, cv::Point2f):
    description: __MISSING__
    return: __OPTIONAL__
    signature_with_names: float distance(cv::Point2f point1, cv::Point2f point2)
    arguments:
      - name: point1
        description: __OPTIONAL__
        type: cv::Point2f
      - description: __OPTIONAL__
        type: cv::Point2f
        name: point2
  float distance(cv::Point2f &, tr::Point2 &):
    return: __OPTIONAL__
    description: __MISSING__
    arguments:
      - type: cv::Point2f &
        name: point1
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: point2
        type: tr::Point2 &
    signature_with_names: float distance(cv::Point2f & point1, tr::Point2 & point2)
---
