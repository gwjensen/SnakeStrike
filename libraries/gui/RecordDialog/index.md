---
defined-in-file: "projects/gui/RecordDialog.h"
fields:
  mKeyPressed:
    type: bool
    description: __MISSING__
    annotation:
      - private
  mCancelled:
    description: __MISSING__
    annotation:
      - private
    type: bool
  mMutex:
    annotation:
      - private
    description: __MISSING__
    type: QMutex
  mNewCapturePath:
    type: QString
    description: __MISSING__
    annotation:
      - private
  staticMetaObject:
    description: __MISSING__
    type: const QMetaObject
  mpCamList:
    annotation:
      - private
    description: __MISSING__
    type: GuiCamArray *
  mpCapturingMsg:
    description: __MISSING__
    type: QMessageBox *
    annotation:
      - private
  mBackUpImageCount:
    type: int
    annotation:
      - private
    description: __MISSING__
  mType:
    annotation:
      - private
    type: const RecordType
    description: __MISSING__
  mpProjDetails:
    annotation:
      - private
    description: __MISSING__
    type: const ProjectDialog *
  mCamHerz:
    annotation:
      - private
    type: uint32_t
    description: __MISSING__
  mRunCount:
    annotation:
      - private
    type: int
    description: __MISSING__
  mpProcess:
    description: __MISSING__
    type: QProcess *
    annotation:
      - private
  mXmlAlreadyExists:
    annotation:
      - private
    description: __MISSING__
    type: bool
  mRecording:
    type: bool
    description: __MISSING__
    annotation:
      - private
  mpFileDialog:
    type: QFileDialog *
    annotation:
      - private
    description: __MISSING__
  mpUi:
    annotation:
      - private
    description: __MISSING__
    type: Ui::RecordDialog *
  mpBufDialog:
    annotation:
      - private
    description: __MISSING__
    type: BufferingDialog *
  mCaptureCount:
    annotation:
      - private
    description: __MISSING__
    type: uint32_t
owner: gwjensen
declaration: "\nclass RecordDialog;"
tags:
  - class
brief: The dialog window for recording images.
title: RecordDialog
layout: class
---
