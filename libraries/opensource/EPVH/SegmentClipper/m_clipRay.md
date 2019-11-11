---
tags:
  - method
owner: gwjensen
defined-in-file: ""
overloads:
  bool clipRay(Eigen::Vector2d, Eigen::Vector2d, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &, std::vector<std::pair<int, int>> &):
    description: __MISSING__
    arguments:
      - name: end1
        description: __OPTIONAL__
        type: Eigen::Vector2d
      - description: __OPTIONAL__
        name: dir
        type: Eigen::Vector2d
      - type: std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &
        description: __OPTIONAL__
        name: clippedSegments
      - description: __OPTIONAL__
        type: std::vector<std::pair<int, int>> &
        name: stripIds
    return: __OPTIONAL__
    signature_with_names: bool clipRay(Eigen::Vector2d end1, Eigen::Vector2d dir, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> & clippedSegments, std::vector<std::pair<int, int>> & stripIds)
brief: __MISSING__
layout: method
title: clipRay
---
