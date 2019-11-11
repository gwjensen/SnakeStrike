---
overloads:
  bool clipEdge(int, int, Eigen::Vector3d &, Eigen::Vector3d &, int &, int &, int &):
    signature_with_names: bool clipEdge(int camId1, int camId2, Eigen::Vector3d & point1, Eigen::Vector3d & point2, int & clipCamId, int & clipContourId, int & clipStripId)
    return: __OPTIONAL__
    description: __MISSING__
    arguments:
      - type: int
        name: camId1
        description: __OPTIONAL__
      - name: camId2
        description: __OPTIONAL__
        type: int
      - name: point1
        description: __OPTIONAL__
        type: Eigen::Vector3d &
      - name: point2
        type: Eigen::Vector3d &
        description: __OPTIONAL__
      - name: clipCamId
        description: __OPTIONAL__
        type: int &
      - type: int &
        description: __OPTIONAL__
        name: clipContourId
      - type: int &
        description: __OPTIONAL__
        name: clipStripId
    annotation:
      - protected
  void clipEdge(tr::Edge, int, std::vector<tr::Edge> &):
    description: __MISSING__
    arguments:
      - description: __OPTIONAL__
        name: edge
        type: tr::Edge
      - type: int
        description: __OPTIONAL__
        name: camId
      - type: std::vector<tr::Edge> &
        name: clippedEdges
        description: __OPTIONAL__
    signature_with_names: void clipEdge(tr::Edge edge, int camId, std::vector<tr::Edge> & clippedEdges)
    return: __OPTIONAL__
    annotation:
      - protected
title: clipEdge
layout: method
brief: __MISSING__
tags:
  - method
defined-in-file: ""
owner: gwjensen
---
