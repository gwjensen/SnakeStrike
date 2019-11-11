---
defined-in-file: ""
tags:
  - class
brief: __MISSING__
owner: gwjensen
fields:
  mProjectionMatrices:
    type: std::vector<Eigen::Matrix<double, 3, 4>>
    description: __MISSING__
  mContours:
    type: std::vector<std::vector<Eigen::Vector2d>>
    description: __MISSING__
  mDummySilhouettes:
    type: std::vector<tr::Silhouette>
    description: __MISSING__
  mInvProjectionMatrices:
    description: __MISSING__
    type: std::vector<Eigen::Matrix3d>
  mObjectSilhouettes:
    type: std::vector<tr::Silhouette> &
    description: __MISSING__
  mImageDims:
    type: std::vector<cv::Size>
    description: __MISSING__
  mObjectSilhouettePaths:
    type: std::vector<std::string>
    description: __MISSING__
  mCameraCenters:
    description: __MISSING__
    type: std::vector<Eigen::Vector3d>
declaration: "\nclass CameraInfo;"
layout: class
title: CameraInfo
---
