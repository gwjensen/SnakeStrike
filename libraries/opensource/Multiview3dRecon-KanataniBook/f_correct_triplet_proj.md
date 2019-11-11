---
tags:
  - function
layout: function
defined-in-file: ""
title: correct_triplet_proj
overloads:
  bool correct_triplet_proj(const Matrix34d *, const Eigen::Vector3d &, const Eigen::Vector3d &, const Eigen::Vector3d &, Eigen::Vector3d &, Eigen::Vector3d &, Eigen::Vector3d &, double *, int, double):
    signature_with_names: bool correct_triplet_proj(const Matrix34d * Proj, const Eigen::Vector3d & x0i, const Eigen::Vector3d & x1i, const Eigen::Vector3d & x2i, Eigen::Vector3d & x0c, Eigen::Vector3d & x1c, Eigen::Vector3d & x2c, double * reproj, int Max_Iter, double Conv_EPS)
    description: __MISSING__
    return: __OPTIONAL__
    arguments:
      - description: __OPTIONAL__
        name: Proj
        type: const Matrix34d [3]
      - description: __OPTIONAL__
        type: const Eigen::Vector3d &
        name: x0i
      - type: const Eigen::Vector3d &
        description: __OPTIONAL__
        name: x1i
      - description: __OPTIONAL__
        type: const Eigen::Vector3d &
        name: x2i
      - type: Eigen::Vector3d &
        name: x0c
        description: __OPTIONAL__
      - type: Eigen::Vector3d &
        description: __OPTIONAL__
        name: x1c
      - description: __OPTIONAL__
        type: Eigen::Vector3d &
        name: x2c
      - type: double *
        name: reproj
        description: __OPTIONAL__
      - name: Max_Iter
        description: __OPTIONAL__
        type: int
      - description: __OPTIONAL__
        type: double
        name: Conv_EPS
owner: gwjensen
brief: __MISSING__
---
