---
owner: gwjensen
declaration: "\nclass GuiCamera;"
layout: class
fields:
  mIsConnected:
    type: bool
    annotation:
      - protected
    description: __MISSING__
  mpParent:
    description: __MISSING__
    annotation:
      - protected
    type: QObject *
  mCamInfo:
    annotation:
      - protected
    type: std::unique_ptr<GuiCamInfo>
    description: __MISSING__
  mImageTimestampsFile:
    type: QFile *
    annotation:
      - protected
    description: __MISSING__
  mIsStopping:
    type: bool
    annotation:
      - protected
    description: __MISSING__
  mImageRetrieveTimeout:
    type: int
    annotation:
      - protected
    description: __MISSING__
  mIsOpened:
    type: bool
    annotation:
      - protected
    description: __MISSING__
  mIdx:
    annotation:
      - protected
    type: int
    description: __MISSING__
  mRwLock:
    type: QReadWriteLock
    description: __MISSING__
    annotation:
      - protected
  mDropCount:
    description: __MISSING__
    type: uint64_t
    annotation:
      - protected
  mNumBurstImages:
    description: __MISSING__
    annotation:
      - protected
    type: int
  mRecordCount:
    annotation:
      - protected
    description: __MISSING__
    type: uint64_t
  mStream:
    description: __MISSING__
    annotation:
      - protected
    type: QTextStream *
  staticMetaObject:
    description: __MISSING__
    type: const QMetaObject
  mNumGrabThreadsRunning:
    annotation:
      - protected
    type: uint32_t
    description: __MISSING__
  mMutex:
    annotation:
      - protected
    description: __MISSING__
    type: QMutex
  mHardwareTriggered:
    type: bool
    description: __MISSING__
    annotation:
      - protected
brief: This is the abstract interface for all camera objects. Any new camera plugin has to follow this interface.
title: GuiCamera
defined-in-file: "projects/gui/GuiCamera.h"
tags:
  - class
---
