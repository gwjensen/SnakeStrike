---
owner: gwjensen
layout: class
brief: __MISSING__
fields:
  mStripContourMap:
    description: __MISSING__
    annotation:
      - protected
    type: std::vector<std::vector<StripContourMap>>
  mObjectContours:
    annotation:
      - protected
    type: std::vector<std::vector<contour>>
    description: __MISSING__
  mSilhouetteCameras:
    description: __MISSING__
    annotation:
      - protected
    type: std::vector<int>
  mGenerators:
    description: __MISSING__
    type: std::vector<std::vector<std::vector<Generator *>> >
    annotation:
      - protected
  mContourHierarchies:
    description: __MISSING__
    type: std::vector<std::vector<cv::Vec4i>>
    annotation:
      - protected
  mOffset:
    annotation:
      - protected
    description: __MISSING__
    type: std::vector<cv::Point2f>
  mInvScale:
    description: __MISSING__
    type: std::vector<double>
    annotation:
      - protected
  mIsOccludingContour:
    type: std::vector<std::vector<bool>>
    description: __MISSING__
    annotation:
      - protected
  mScale:
    annotation:
      - protected
    description: __MISSING__
    type: std::vector<double>
  mCalibration:
    type: CameraInfo *
    annotation:
      - protected
    description: __MISSING__
  mMostOrthogonalCamera:
    description: __MISSING__
    type: std::vector<int>
    annotation:
      - protected
  mBoundingBoxImages:
    type: std::vector<cv::Mat>
    annotation:
      - protected
    description: __MISSING__
  mIsCameraUsed:
    type: std::vector<uchar>
    description: __MISSING__
    annotation:
      - protected
  mGeneratorImages:
    annotation:
      - protected
    type: std::vector<cv::Mat>
    description: __MISSING__
tags:
  - class
defined-in-file: ""
title: BaseVH
namespace:
  - tr
declaration: "\nclass tr::BaseVH;"
---
