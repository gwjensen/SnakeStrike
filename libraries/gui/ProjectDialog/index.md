---
layout: class
declaration: "\nclass ProjectDialog;"
fields:
  mFilterIterations:
    type: uint32_t
    annotation:
      - private
    description: __MISSING__
  mSaveCalibDir:
    type: QString
    annotation:
      - private
    description: __MISSING__
  mNumPointsToTrack:
    type: uint32_t
    annotation:
      - private
    description: __MISSING__
  mSaveDataDir:
    type: QString
    description: __MISSING__
    annotation:
      - private
  mVRightBound:
    description: __MISSING__
    type: uint32_t
    annotation:
      - private
  mSaveUndistort:
    description: __MISSING__
    type: bool
    annotation:
      - private
  mCalibrationImageOrigLoc:
    type: QString
    description: __MISSING__
    annotation:
      - private
  mFilterThreshold:
    description: __MISSING__
    type: uint32_t
    annotation:
      - private
  mSaveDataFilenameBase:
    annotation:
      - private
    description: __MISSING__
    type: QString
  mSaveMaskDir:
    description: __MISSING__
    type: QString
    annotation:
      - private
  mProjDir:
    type: QString
    description: __MISSING__
    annotation:
      - private
  mThresholdParmsChanged:
    type: bool
    annotation:
      - private
    description: __MISSING__
  mName:
    annotation:
      - private
    description: __MISSING__
    type: QString
  mHRightBound:
    type: uint32_t
    description: __MISSING__
    annotation:
      - private
  mCamLibPath:
    annotation:
      - private
    description: __MISSING__
    type: QString
  mCamsToNotInclude:
    annotation:
      - private
    type: QVector<int>
    description: __MISSING__
  mCalibrationImageProjLoc:
    description: __MISSING__
    type: QString
    annotation:
      - private
  mSRightBound:
    description: __MISSING__
    annotation:
      - private
    type: uint32_t
  mpUi:
    type: Ui::ProjectDialog *
    annotation:
      - private
    description: __MISSING__
  mSLeftBound:
    annotation:
      - private
    description: __MISSING__
    type: uint32_t
  mHLeftBound:
    annotation:
      - private
    type: uint32_t
    description: __MISSING__
  mProjDirChanged:
    annotation:
      - private
    type: bool
    description: __MISSING__
  mSaveTriangDir:
    annotation:
      - private
    description: __MISSING__
    type: QString
  mTriggerScriptChanged:
    description: __MISSING__
    type: bool
    annotation:
      - private
  mRetrieveImageTimeout:
    description: __MISSING__
    type: uint64_t
    annotation:
      - private
  mExportConvertScriptLoc:
    type: QString
    annotation:
      - private
    description: __MISSING__
  mCamLibPathChanged:
    type: bool
    annotation:
      - private
    description: __MISSING__
  mTriggeringScriptProjLoc:
    description: __MISSING__
    type: QString
    annotation:
      - private
  mFilterSize:
    type: uint32_t
    annotation:
      - private
    description: __MISSING__
  mUseMask:
    annotation:
      - private
    description: __MISSING__
    type: bool
  staticMetaObject:
    description: __MISSING__
    type: const QMetaObject
  mTriggeringScriptOrigLoc:
    annotation:
      - private
    description: __MISSING__
    type: QString
  mVLeftBound:
    annotation:
      - private
    description: __MISSING__
    type: uint32_t
  mRetrieveImageTimeoutChanged:
    type: bool
    annotation:
      - private
    description: __MISSING__
  mProjOpened:
    description: __MISSING__
    annotation:
      - private
    type: bool
  mCalibImageChanged:
    type: bool
    annotation:
      - private
    description: __MISSING__
  mSaveTriangFilenameBase:
    description: __MISSING__
    annotation:
      - private
    type: QString
defined-in-file: "projects/gui/ProjectDialog.h"
brief: This dialog object holds all of the information related to the project. It is commonly passed around to other dialogs to give them access to this information.
owner: gwjensen
title: ProjectDialog
tags:
  - class
---
