---
defined-in-file: "image/PixelCluster.h"
tags:
  - class
owner: gwjensen
layout: class
declaration: "\nclass PixelCluster;"
title: PixelCluster
fields:
  mMean:
    description: __MISSING__
    annotation:
      - private
    type: cv::Point2f
  mStdDev:
    description: __MISSING__
    annotation:
      - private
    type: float
  mPoints:
    type: std::vector<SmtPixel>
    annotation:
      - private
    description: __MISSING__
  mCenterSum:
    annotation:
      - private
    type: cv::Point2f
    description: __MISSING__
  mSize:
    annotation:
      - private
    description: __MISSING__
    type: int
brief:
---
