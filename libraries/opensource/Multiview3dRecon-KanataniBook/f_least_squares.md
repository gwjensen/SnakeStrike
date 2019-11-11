---
owner: gwjensen
tags:
  - function
overloads:
  bool least_squares(Eigen::Vector2d *, Eigen::Vector2d *, int, const Matrix34d &, const Matrix34d &, Eigen::Vector3d *, double):
    arguments:
      - name: pos0
        type: Eigen::Vector2d []
        description: __OPTIONAL__
      - type: Eigen::Vector2d []
        description: __OPTIONAL__
        name: pos1
      - description: __OPTIONAL__
        type: int
        name: Num
      - type: const Matrix34d &
        description: __OPTIONAL__
        name: P0
      - name: P1
        type: const Matrix34d &
        description: __OPTIONAL__
      - type: Eigen::Vector3d []
        description: __OPTIONAL__
        name: X
      - name: f0
        type: double
        description: __OPTIONAL__
    return: __OPTIONAL__
    signature_with_names: bool least_squares(Eigen::Vector2d * pos0, Eigen::Vector2d * pos1, int Num, const Matrix34d & P0, const Matrix34d & P1, Eigen::Vector3d * X, double f0)
    description: __MISSING__
  Eigen::Vector3d least_squares(const Eigen::Vector2d &, const Eigen::Vector2d &, const Matrix34d &, const Matrix34d &, double):
    arguments:
      - type: const Eigen::Vector2d &
        name: pos0
        description: __OPTIONAL__
      - name: pos1
        type: const Eigen::Vector2d &
        description: __OPTIONAL__
      - type: const Matrix34d &
        name: P0
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: P1
        type: const Matrix34d &
      - name: f0
        description: __OPTIONAL__
        type: double
    return: __OPTIONAL__
    signature_with_names: Eigen::Vector3d least_squares(const Eigen::Vector2d & pos0, const Eigen::Vector2d & pos1, const Matrix34d & P0, const Matrix34d & P1, double f0)
    description: __MISSING__
defined-in-file: ""
layout: function
brief: __MISSING__
title: least_squares
---
