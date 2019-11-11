---
overloads:
  Eigen::Vector3d triangulation(const Matrix34d &, const Matrix34d &, const Matrix34d &, const Eigen::Vector2d &, const Eigen::Vector2d &, const Eigen::Vector2d &, double):
    arguments:
      - description: __OPTIONAL__
        type: const Matrix34d &
        name: P0
      - description: __OPTIONAL__
        type: const Matrix34d &
        name: P1
      - name: P2
        type: const Matrix34d &
        description: __OPTIONAL__
      - type: const Eigen::Vector2d &
        name: pos0
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: const Eigen::Vector2d &
        name: pos1
      - type: const Eigen::Vector2d &
        name: pos2
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: f0
        type: double
    return: __OPTIONAL__
    signature_with_names: Eigen::Vector3d triangulation(const Matrix34d & P0, const Matrix34d & P1, const Matrix34d & P2, const Eigen::Vector2d & pos0, const Eigen::Vector2d & pos1, const Eigen::Vector2d & pos2, double f0)
    description: __MISSING__
  bool triangulation(const Matrix34d &, const Matrix34d &, const Matrix34d &, Eigen::Vector2d *, Eigen::Vector2d *, Eigen::Vector2d *, Eigen::Vector3d *, int, double):
    arguments:
      - name: P0
        description: __OPTIONAL__
        type: const Matrix34d &
      - description: __OPTIONAL__
        name: P1
        type: const Matrix34d &
      - description: __OPTIONAL__
        name: P2
        type: const Matrix34d &
      - name: pos0
        type: Eigen::Vector2d []
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: Eigen::Vector2d []
        name: pos1
      - type: Eigen::Vector2d []
        name: pos2
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: r
        type: Eigen::Vector3d []
      - type: int
        description: __OPTIONAL__
        name: Num
      - description: __OPTIONAL__
        name: f0
        type: double
    description: __MISSING__
    signature_with_names: bool triangulation(const Matrix34d & P0, const Matrix34d & P1, const Matrix34d & P2, Eigen::Vector2d * pos0, Eigen::Vector2d * pos1, Eigen::Vector2d * pos2, Eigen::Vector3d * r, int Num, double f0)
    return: __OPTIONAL__
  Eigen::Vector3d triangulation(const Matrix34d *, const Eigen::Vector2d *, int, double):
    description: __MISSING__
    return: __OPTIONAL__
    signature_with_names: Eigen::Vector3d triangulation(const Matrix34d * Prj, const Eigen::Vector2d * xy, int CamNum, double f0)
    arguments:
      - description: __OPTIONAL__
        name: Prj
        type: const Matrix34d []
      - name: xy
        type: const Eigen::Vector2d []
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: int
        name: CamNum
      - name: f0
        description: __OPTIONAL__
        type: double
brief: __MISSING__
tags:
  - function
layout: function
owner: gwjensen
defined-in-file: ""
title: triangulation
---
