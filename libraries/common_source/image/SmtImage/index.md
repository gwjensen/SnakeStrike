---
tags:
  - class
declaration: "\nclass SmtImage;"
owner: gwjensen
defined-in-file: "image/SmtImage.h"
brief: This class is the base image object used in the library. It holds data pertaining to the capture of the image as well as functions for easier use.
title: SmtImage
dtor: unspecified
layout: class
fields:
  mLocation:
    description: __MISSING__
    type: std::string
    annotation:
      - private
  mTimestep:
    description: __MISSING__
    type: int
    annotation:
      - private
  mFileSuffix:
    type: std::string
    annotation:
      - private
    description: __MISSING__
  mIsBinary:
    annotation:
      - private
    description: __MISSING__
    type: bool
  mIsUndistorted:
    type: bool
    annotation:
      - private
    description: __MISSING__
  mCam:
    description: __MISSING__
    annotation:
      - private
    type: int
  mIsRGB:
    description: __MISSING__
    annotation:
      - private
    type: bool
---
