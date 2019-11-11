---
overloads:
  bool correct_triplet_tensor(const Eigen::Matrix3d *, const Eigen::Vector3d &, const Eigen::Vector3d &, const Eigen::Vector3d &, Eigen::Vector3d &, Eigen::Vector3d &, Eigen::Vector3d &, double *, double, double):
    arguments:
      - type: const Eigen::Matrix3d []
        description: __OPTIONAL__
        name: tfT
      - type: const Eigen::Vector3d &
        name: x0i
        description: __OPTIONAL__
      - name: x1i
        description: __OPTIONAL__
        type: const Eigen::Vector3d &
      - description: __OPTIONAL__
        type: const Eigen::Vector3d &
        name: x2i
      - name: x0c
        description: __OPTIONAL__
        type: Eigen::Vector3d &
      - description: __OPTIONAL__
        name: x1c
        type: Eigen::Vector3d &
      - name: x2c
        type: Eigen::Vector3d &
        description: __OPTIONAL__
      - name: reproj
        type: double *
        description: __OPTIONAL__
      - name: Max_Iter
        description: __OPTIONAL__
        type: double
      - name: Conv_EPS
        description: __OPTIONAL__
        type: double
    return: __OPTIONAL__
    signature_with_names: bool correct_triplet_tensor(const Eigen::Matrix3d * tfT, const Eigen::Vector3d & x0i, const Eigen::Vector3d & x1i, const Eigen::Vector3d & x2i, Eigen::Vector3d & x0c, Eigen::Vector3d & x1c, Eigen::Vector3d & x2c, double * reproj, double Max_Iter, double Conv_EPS)
    description: __MISSING__
layout: function
title: correct_triplet_tensor
tags:
  - function
defined-in-file: ""
brief: __MISSING__
owner: gwjensen
---
