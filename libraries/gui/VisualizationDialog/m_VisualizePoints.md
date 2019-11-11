---
tags:
  - method
title: VisualizePoints
brief: Show the given points in the visualization window. This is done in a series so that the window looks like an animation.
layout: method
owner: gwjensen
overloads:
  void VisualizePoints(const std::vector<std::vector<pcl::PointXYZRGB>> &, const int, const int, const bool, const bool, const bool, int, int):
    annotation:
      - private
    arguments:
      - type: const std::vector<std::vector<pcl::PointXYZRGB>> &
        description: __OPTIONAL__
        name: iTriangulatedPoints
      - type: const int
        description: __OPTIONAL__
        name: iNumTracePoints
      - type: const int
        description: __OPTIONAL__
        name: iLengthTraceHistory
      - name: iCenterX
        description: __OPTIONAL__
        type: const bool
      - description: __OPTIONAL__
        type: const bool
        name: iCenterY
      - type: const bool
        description: __OPTIONAL__
        name: iCenterZ
      - type: int
        name: iStartTimeStep
        description: __OPTIONAL__
      - name: iEndTimeStep
        description: __OPTIONAL__
        type: int
    signature_with_names: void VisualizePoints(const std::vector<std::vector<pcl::PointXYZRGB>> & iTriangulatedPoints, const int iNumTracePoints, const int iLengthTraceHistory, const bool iCenterX, const bool iCenterY, const bool iCenterZ, int iStartTimeStep, int iEndTimeStep)
    description: __MISSING__
    return: __OPTIONAL__
defined-in-file: "projects/gui/VisualizationDialog.h"
---
