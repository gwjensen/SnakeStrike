---
owner: gwjensen
overloads:
  bool clipSegment2(Eigen::Vector2d, Eigen::Vector2d, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &, std::vector<std::pair<int, int>> &):
    arguments:
      - name: end1
        type: Eigen::Vector2d
        description: __OPTIONAL__
      - name: end2
        type: Eigen::Vector2d
        description: __OPTIONAL__
      - type: std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &
        description: __OPTIONAL__
        name: clippedSegments
      - type: std::vector<std::pair<int, int>> &
        description: __OPTIONAL__
        name: stripIds
    return: __OPTIONAL__
    description: __MISSING__
    signature_with_names: bool clipSegment2(Eigen::Vector2d end1, Eigen::Vector2d end2, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> & clippedSegments, std::vector<std::pair<int, int>> & stripIds)
tags:
  - method
title: clipSegment2
defined-in-file: ""
brief: __MISSING__
layout: method
---
