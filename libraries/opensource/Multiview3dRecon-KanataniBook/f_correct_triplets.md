---
defined-in-file: ""
layout: function
owner: gwjensen
tags:
  - function
title: correct_triplets
brief: __MISSING__
overloads:
  bool correct_triplets(const Matrix34d *, Eigen::Vector2d *, Eigen::Vector2d *, Eigen::Vector2d *, Eigen::Vector2d *, Eigen::Vector2d *, Eigen::Vector2d *, int, double *, int, double, double):
    arguments:
      - type: const Matrix34d *
        name: Proj
        description: __OPTIONAL__
      - name: x0i
        description: __OPTIONAL__
        type: Eigen::Vector2d []
      - description: __OPTIONAL__
        type: Eigen::Vector2d []
        name: x1i
      - type: Eigen::Vector2d []
        description: __OPTIONAL__
        name: x2i
      - description: __OPTIONAL__
        name: x0c
        type: Eigen::Vector2d []
      - type: Eigen::Vector2d []
        description: __OPTIONAL__
        name: x1c
      - type: Eigen::Vector2d []
        description: __OPTIONAL__
        name: x2c
      - name: Num
        description: __OPTIONAL__
        type: int
      - description: __OPTIONAL__
        name: reproj
        type: double []
      - type: int
        name: Mac_Iter
        description: __OPTIONAL__
      - type: double
        description: __OPTIONAL__
        name: Conv_EPS
      - description: __OPTIONAL__
        type: double
        name: f0
    description: __MISSING__
    signature_with_names: bool correct_triplets(const Matrix34d * Proj, Eigen::Vector2d * x0i, Eigen::Vector2d * x1i, Eigen::Vector2d * x2i, Eigen::Vector2d * x0c, Eigen::Vector2d * x1c, Eigen::Vector2d * x2c, int Num, double * reproj, int Mac_Iter, double Conv_EPS, double f0)
    return: __OPTIONAL__
---
