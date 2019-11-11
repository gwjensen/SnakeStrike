---
title: optimal_correction_Grady
owner: gwjensen
brief: __MISSING__
defined-in-file: ""
tags:
  - function
layout: function
overloads:
  bool optimal_correction_Grady(const Eigen::Matrix3d (*)[3], int, Eigen::Vector3d *, Eigen::Vector3d *, Eigen::Vector3d *, double *, int, double):
    return: __OPTIONAL__
    arguments:
      - name: tfT
        type: const Eigen::Matrix3d [][3]
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: int
        name: CNum
      - name: xk
        description: __OPTIONAL__
        type: Eigen::Vector3d []
      - name: xkal
        description: __OPTIONAL__
        type: Eigen::Vector3d []
      - name: xc
        description: __OPTIONAL__
        type: Eigen::Vector3d []
      - name: reperr
        type: double *
        description: __OPTIONAL__
      - type: int
        description: __OPTIONAL__
        name: Max_Iter
      - name: Conv_EPS
        description: __OPTIONAL__
        type: double
    description: __MISSING__
    signature_with_names: bool optimal_correction_Grady(const Eigen::Matrix3d (*)[3] tfT, int CNum, Eigen::Vector3d * xk, Eigen::Vector3d * xkal, Eigen::Vector3d * xc, double * reperr, int Max_Iter, double Conv_EPS)
---
