---
layout: function
defined-in-file: ""
brief: __MISSING__
owner: gwjensen
tags:
  - function
overloads:
  double optimal_correction(const Eigen::Vector2d &, const Eigen::Vector2d &, const Vector9d &, Eigen::Vector2d &, Eigen::Vector2d &, int, double, double):
    signature_with_names: double optimal_correction(const Eigen::Vector2d & p0, const Eigen::Vector2d & p1, const Vector9d & theta, Eigen::Vector2d & np0, Eigen::Vector2d & np1, int Max_Iter, double Conv_EPS, double f0)
    description: __MISSING__
    return: __OPTIONAL__
    arguments:
      - type: const Eigen::Vector2d &
        description: __OPTIONAL__
        name: p0
      - type: const Eigen::Vector2d &
        description: __OPTIONAL__
        name: p1
      - name: theta
        type: const Vector9d &
        description: __OPTIONAL__
      - type: Eigen::Vector2d &
        description: __OPTIONAL__
        name: np0
      - name: np1
        type: Eigen::Vector2d &
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: Max_Iter
        type: int
      - name: Conv_EPS
        type: double
        description: __OPTIONAL__
      - type: double
        description: __OPTIONAL__
        name: f0
  bool optimal_correction(Eigen::Vector2d *, Eigen::Vector2d *, int, const Vector9d &, Eigen::Vector2d *, Eigen::Vector2d *, int, double, double):
    description: __MISSING__
    return: __OPTIONAL__
    signature_with_names: bool optimal_correction(Eigen::Vector2d * p0, Eigen::Vector2d * p1, int Num, const Vector9d & theta, Eigen::Vector2d * np0, Eigen::Vector2d * np1, int Max_Iter, double Conv_EPS, double f0)
    arguments:
      - description: __OPTIONAL__
        name: p0
        type: Eigen::Vector2d []
      - description: __OPTIONAL__
        name: p1
        type: Eigen::Vector2d []
      - name: Num
        type: int
        description: __OPTIONAL__
      - type: const Vector9d &
        name: theta
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: np0
        type: Eigen::Vector2d []
      - type: Eigen::Vector2d []
        description: __OPTIONAL__
        name: np1
      - description: __OPTIONAL__
        name: Max_Iter
        type: int
      - description: __OPTIONAL__
        name: Conv_EPS
        type: double
      - description: __OPTIONAL__
        name: f0
        type: double
title: optimal_correction
---
