---
title: EPVH
layout: class
declaration: "\nclass tr::EPVH;"
owner: gwjensen
brief: __MISSING__
tags:
  - class
defined-in-file: ""
fields:
  mCameraViewingEdges:
    type: std::vector<std::vector<std::vector<Edge>> >
    description: __MISSING__
    annotation:
      - protected
  mDisplayDebug:
    description: __MISSING__
    annotation:
      - protected
    type: bool
  mClippers:
    description: __MISSING__
    annotation:
      - protected
    type: std::vector<SegmentClipper>
  mVertexChainCounter:
    annotation:
      - protected
    description: __MISSING__
    type: int
  mEdges:
    type: std::vector<std::vector<Edge>>
    description: __MISSING__
    annotation:
      - protected
  mVertices:
    description: __MISSING__
    annotation:
      - protected
    type: std::vector<Vertex *>
namespace:
  - tr
---
