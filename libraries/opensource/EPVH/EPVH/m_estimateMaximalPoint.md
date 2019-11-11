---
owner: gwjensen
layout: method
tags:
  - method
brief: __MISSING__
overloads:
  Eigen::Vector3d estimateMaximalPoint(Eigen::Vector3d &, tr::Vector3d &, tr::Generator *, tr::Generator *, uchar &, int):
    signature_with_names: Eigen::Vector3d estimateMaximalPoint(Eigen::Vector3d & startPoint, tr::Vector3d & direction, tr::Generator * gen1, tr::Generator * gen2, uchar & intersectionIndex, int ignoreIndex)
    description: __MISSING__
    annotation:
      - protected
    return: __OPTIONAL__
    arguments:
      - type: Eigen::Vector3d &
        name: startPoint
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: tr::Vector3d &
        name: direction
      - name: gen1
        description: __OPTIONAL__
        type: tr::Generator *
      - description: __OPTIONAL__
        name: gen2
        type: tr::Generator *
      - description: __OPTIONAL__
        type: uchar &
        name: intersectionIndex
      - type: int
        description: __OPTIONAL__
        name: ignoreIndex
title: estimateMaximalPoint
defined-in-file: ""
---
