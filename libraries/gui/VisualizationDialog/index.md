---
title: VisualizationDialog
declaration: "\nclass VisualizationDialog;"
brief: Dialog for viewing the triangulated points.
defined-in-file: "projects/gui/VisualizationDialog.h"
fields:
  mCurTimestep:
    annotation:
      - private
    type: int
    description: __MISSING__
  mNumMissingTimesteps:
    description: __MISSING__
    type: int
    annotation:
      - private
  mTriangulatedPointsPCL:
    type: std::vector<std::vector<pcl::PointXYZRGB>>
    description: __MISSING__
    annotation:
      - private
  mViewer:
    annotation:
      - private
    description: __MISSING__
    type: boost::shared_ptr<pcl::visualization::PCLVisualizer>
  mPointNoise:
    annotation:
      - private
    type: double
    description: __MISSING__
  mVisualHullInterface:
    type: boost::shared_ptr<opensource::EPVHInterface>
    description: __MISSING__
    annotation:
      - private
  mTriangulatedPointsCV:
    description: __MISSING__
    type: std::vector<std::vector<cv::Point3d>>
    annotation:
      - private
  mCloud:
    description: __MISSING__
    type: pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    annotation:
      - private
  mViewRunning:
    annotation:
      - private
    type: bool
    description: __MISSING__
  mMutex:
    type: QMutex
    annotation:
      - private
    description: __MISSING__
  mMaxTimesteps:
    type: int
    annotation:
      - private
    description: __MISSING__
  mpUi:
    type: Ui::VisualizationDialog *
    annotation:
      - private
    description: __MISSING__
  mHullmesh:
    annotation:
      - private
    description: __MISSING__
    type: pcl::PolygonMesh::Ptr
  mOffsetX:
    description: __MISSING__
    annotation:
      - private
    type: double
  mOffsetZ:
    annotation:
      - private
    type: double
    description: __MISSING__
  mPolyDataVec:
    description: __MISSING__
    annotation:
      - private
    type: std::vector<vtkSmartPointer<vtkPolyData>>
  mHullviewer:
    annotation:
      - private
    description: __MISSING__
    type: boost::shared_ptr<pcl::visualization::PCLVisualizer>
  mMaxNumPoints:
    annotation:
      - private
    description: __MISSING__
    type: uint32_t
  mStopCalled:
    annotation:
      - private
    description: __MISSING__
    type: bool
  mRwLock:
    annotation:
      - private
    type: QReadWriteLock
    description: __MISSING__
  mpProjDetails:
    type: ProjectDialog *
    description: __MISSING__
    annotation:
      - private
  staticMetaObject:
    description: __MISSING__
    type: const QMetaObject
  mOffsetY:
    annotation:
      - private
    type: double
    description: __MISSING__
  mUpdateMultiplier:
    type: double
    annotation:
      - private
    description: __MISSING__
tags:
  - class
owner: gwjensen
layout: class
---
