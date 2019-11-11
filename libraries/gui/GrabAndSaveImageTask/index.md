---
defined-in-file: "projects/gui/GrabAndSaveImageTask.h"
title: GrabAndSaveImageTask
brief: This is the object that gets put on a threadpool to pull an image from the buffer of images being filled by the camera, and save it to disk.
dtor: unspecified
layout: class
declaration: "\nclass GrabAndSaveImageTask;"
fields:
  mpCamera:
    description: __MISSING__
    annotation:
      - private
    type: GuiCamera *
  mpStream:
    description: __MISSING__
    annotation:
      - private
    type: QTextStream *
  mRecordCount:
    description: __MISSING__
    type: uint32_t
    annotation:
      - private
  mCamIdx:
    type: uint32_t
    annotation:
      - private
    description: __MISSING__
  mpMutex:
    annotation:
      - private
    description: __MISSING__
    type: QMutex *
  mMaxNumImages:
    type: uint32_t
    annotation:
      - private
    description: __MISSING__
  staticMetaObject:
    description: __MISSING__
    type: const QMetaObject
  mSaveDir:
    annotation:
      - private
    description: __MISSING__
    type: QString
owner: gwjensen
tags:
  - class
---
