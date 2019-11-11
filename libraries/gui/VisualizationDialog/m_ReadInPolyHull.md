---
defined-in-file: "projects/gui/VisualizationDialog.h"
tags:
  - method
overloads:
  void ReadInPolyHull(QString, uint32_t, std::vector<vtkSmartPointer<vtkPolyData>> &):
    signature_with_names: void ReadInPolyHull(QString iDirectory, uint32_t iTotalNumTimesteps, std::vector<vtkSmartPointer<vtkPolyData>> & oPolyData)
    description: __MISSING__
    return: __OPTIONAL__
    annotation:
      - protected
    arguments:
      - description: __OPTIONAL__
        name: iDirectory
        type: QString
      - name: iTotalNumTimesteps
        description: __OPTIONAL__
        type: uint32_t
      - description: __OPTIONAL__
        type: std::vector<vtkSmartPointer<vtkPolyData>> &
        name: oPolyData
title: ReadInPolyHull
layout: method
owner: gwjensen
brief: If a convex hull of the object that was in the camera setup was captured (this requires mask images to be made) then read that hull in.
---
