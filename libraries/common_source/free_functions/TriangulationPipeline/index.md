---
fields:
  mIsCancelled:
    annotation:
      - private
    type: bool
    description: __MISSING__
  mpsInstance:
    description: __MISSING__
    type: TriangulationPipeline *
    annotation:
      - private
  mCamIndexesToExcludePair:
    annotation:
      - private
    description: __MISSING__
    type: std::pair<unsigned long, std::set<unsigned long>>
  mImageHeight:
    annotation:
      - private
    type: double
    description: __MISSING__
  mImageWidth:
    type: double
    annotation:
      - private
    description: __MISSING__
  mConfig:
    description: __MISSING__
    annotation:
      - private
    type: TrackerConfigFile
  mImagesRead:
    annotation:
      - private
    type: uint64_t
    description: __MISSING__
  mIsInit:
    annotation:
      - private
    type: bool
    description: __MISSING__
  mImageBufferLength:
    description: __MISSING__
    annotation:
      - private
    type: double
  mBinaryImages:
    type: ImageSet
    description: __MISSING__
    annotation:
      - private
  mTimestepImages:
    annotation:
      - private
    type: ImageSet
    description: __MISSING__
  mFirstMatchTimestep:
    type: uint64_t
    description: __MISSING__
    annotation:
      - private
  mPixelsToTrack:
    type: PixelSet
    annotation:
      - private
    description: __MISSING__
title: TriangulationPipeline
owner: gwjensen
brief: This class handles the processing of the images using user defined thresholds and then triangulates and correlates the found points in each image. This class is the main driver of the functionality of the common source library.
declaration: "\nclass TriangulationPipeline;"
layout: class
tags:
  - class
defined-in-file: "TriangulationPipeline.h"
---
