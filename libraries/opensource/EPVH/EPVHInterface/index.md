---
tags:
  - class
namespace:
  - opensource
layout: class
declaration: "\nclass opensource::EPVHInterface;"
owner: gwjensen
fields:
  hullMeshActor:
    type: vtkActor *
    description: __MISSING__
    annotation:
      - private
  hullinteractor:
    type: vtkRenderWindowInteractor *
    description: __MISSING__
    annotation:
      - private
  hullwindow:
    annotation:
      - private
    description: __MISSING__
    type: vtkRenderWindow *
  hullMeshMapper:
    annotation:
      - private
    type: vtkPolyDataMapper *
    description: __MISSING__
  hullrenderer:
    description: __MISSING__
    type: vtkRenderer *
    annotation:
      - private
  hullstyle:
    description: __MISSING__
    type: vtkInteractorStyleTrackballCamera *
    annotation:
      - private
  mutex:
    annotation:
      - private
    type: std::mutex
    description: __MISSING__
defined-in-file: ""
title: EPVHInterface
brief: __MISSING__
---
