---
title: optimal_correction_all
defined-in-file: ""
overloads:
  bool optimal_correction_all(Matrix34d *, int, Eigen::MatrixXd *, Eigen::MatrixXd *, Eigen::MatrixXd *, Eigen::MatrixXi &, double *, int, int, double, double):
    return: __OPTIONAL__
    arguments:
      - name: Proj
        type: Matrix34d []
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: CamNumAll
        type: int
      - description: __OPTIONAL__
        name: Xk0
        type: Eigen::MatrixXd []
      - name: XkLastPoint0
        description: __OPTIONAL__
        type: Eigen::MatrixXd []
      - type: Eigen::MatrixXd []
        description: __OPTIONAL__
        name: Xkc0
      - name: idx
        description: __OPTIONAL__
        type: Eigen::MatrixXi &
      - type: double []
        name: reperr
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: PtNum
        type: int
      - description: __OPTIONAL__
        name: Max_Iter
        type: int
      - description: __OPTIONAL__
        name: Conv_EPS
        type: double
      - name: f0
        type: double
        description: __OPTIONAL__
    description: __MISSING__
    signature_with_names: bool optimal_correction_all(Matrix34d * Proj, int CamNumAll, Eigen::MatrixXd * Xk0, Eigen::MatrixXd * XkLastPoint0, Eigen::MatrixXd * Xkc0, Eigen::MatrixXi & idx, double * reperr, int PtNum, int Max_Iter, double Conv_EPS, double f0)
  bool optimal_correction_all(Matrix34d *, int, Eigen::MatrixXd *, Eigen::MatrixXd *, Eigen::MatrixXi &, double *, int, int, double, double):
    arguments:
      - type: Matrix34d []
        name: Proj
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: int
        name: CamNumAll
      - description: __OPTIONAL__
        type: Eigen::MatrixXd []
        name: Xk0
      - name: Xkc0
        type: Eigen::MatrixXd []
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: idx
        type: Eigen::MatrixXi &
      - description: __OPTIONAL__
        name: reperr
        type: double []
      - type: int
        description: __OPTIONAL__
        name: PtNum
      - type: int
        description: __OPTIONAL__
        name: Max_Iter
      - description: __OPTIONAL__
        name: Conv_EPS
        type: double
      - description: __OPTIONAL__
        name: f0
        type: double
    return: __OPTIONAL__
    description: __MISSING__
    signature_with_names: bool optimal_correction_all(Matrix34d * Proj, int CamNumAll, Eigen::MatrixXd * Xk0, Eigen::MatrixXd * Xkc0, Eigen::MatrixXi & idx, double * reperr, int PtNum, int Max_Iter, double Conv_EPS, double f0)
layout: function
tags:
  - function
owner: gwjensen
brief: __MISSING__
---
