---
declaration: "\nclass SegmentClipper;"
fields:
  yMax:
    type: double
    annotation:
      - private
    description: __MISSING__
  xMax:
    type: double
    description: __MISSING__
    annotation:
      - private
  mVerticalStack:
    type: std::vector<std::vector<std::pair<int, int>> >
    description: __MISSING__
    annotation:
      - private
  xMin:
    annotation:
      - private
    description: __MISSING__
    type: double
  mPolygon:
    type: std::vector<std::vector<Eigen::Vector2d>>
    annotation:
      - private
    description: __MISSING__
  mContourHierarchy:
    description: __MISSING__
    annotation:
      - private
    type: std::vector<cv::Vec4i>
  mSelectedSegments:
    type: std::vector<std::pair<int, int>>
    description: __MISSING__
  mDisplayInfo:
    type: bool
    description: __MISSING__
  mUsedIndices:
    annotation:
      - private
    description: __MISSING__
    type: std::vector<std::vector<bool>>
  mUsedStack:
    type: std::vector<std::pair<int, int>>
    description: __MISSING__
    annotation:
      - private
  mHorizontalStack:
    description: __MISSING__
    type: std::vector<std::vector<std::pair<int, int>> >
    annotation:
      - private
  mStackTop:
    description: __MISSING__
    type: int
    annotation:
      - private
  mVerticalArray:
    description: __MISSING__
    annotation:
      - private
    type: std::vector<IndexValPair>
  mColors:
    type: std::vector<cv::Vec3f>
    description: __MISSING__
  mCvPolygon:
    description: __MISSING__
    type: std::vector<std::vector<cv::Point2f>>
    annotation:
      - private
  mHorizontalArray:
    type: std::vector<IndexValPair>
    description: __MISSING__
    annotation:
      - private
  yMin:
    annotation:
      - private
    description: __MISSING__
    type: double
title: SegmentClipper
defined-in-file: ""
layout: class
dtor: unspecified
tags:
  - class
brief: __MISSING__
owner: gwjensen
---
