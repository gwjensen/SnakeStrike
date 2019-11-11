---
owner: gwjensen
brief: __MISSING__
overloads:
  bool triangulation_all(const Matrix34d *, int, Eigen::MatrixXd *, Eigen::Vector3d *, int, const Eigen::MatrixXi &, double):
    return: __OPTIONAL__
    signature_with_names: bool triangulation_all(const Matrix34d * Prj, int CamNum, Eigen::MatrixXd * x, Eigen::Vector3d * Xr, int PtNum, const Eigen::MatrixXi & idx, double f0)
    description: __MISSING__
    arguments:
      - description: __OPTIONAL__
        type: const Matrix34d []
        name: Prj
      - name: CamNum
        type: int
        description: __OPTIONAL__
      - type: Eigen::MatrixXd []
        description: __OPTIONAL__
        name: x
      - name: Xr
        type: Eigen::Vector3d []
        description: __OPTIONAL__
      - name: PtNum
        type: int
        description: __OPTIONAL__
      - name: idx
        type: const Eigen::MatrixXi &
        description: __OPTIONAL__
      - name: f0
        type: double
        description: __OPTIONAL__
tags:
  - function
layout: function
defined-in-file: ""
title: triangulation_all
---
