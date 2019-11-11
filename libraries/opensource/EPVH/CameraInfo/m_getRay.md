---
defined-in-file: ""
owner: gwjensen
title: getRay
brief: __MISSING__
overloads:
  void getRay(int, const Eigen::Vector2d &, Eigen::Vector3d &):
    description: __MISSING__
    arguments:
      - type: int
        name: camId
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: const Eigen::Vector2d &
        name: imagePos
      - name: rayDir
        type: Eigen::Vector3d &
        description: __OPTIONAL__
    signature_with_names: void getRay(int camId, const Eigen::Vector2d & imagePos, Eigen::Vector3d & rayDir)
    return: __OPTIONAL__
  void getRay(int, const cv::Point2f &, Eigen::Vector3d &):
    description: __MISSING__
    return: __OPTIONAL__
    arguments:
      - description: __OPTIONAL__
        name: camId
        type: int
      - type: const cv::Point2f &
        description: __OPTIONAL__
        name: imagePos
      - description: __OPTIONAL__
        type: Eigen::Vector3d &
        name: rayDir
    signature_with_names: void getRay(int camId, const cv::Point2f & imagePos, Eigen::Vector3d & rayDir)
tags:
  - method
layout: method
---
