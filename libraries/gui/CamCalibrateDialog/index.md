---
defined-in-file: "projects/gui/CamCalibrateDialog.h"
layout: class
title: CamCalibrateDialog
brief: The dialog for calibrating the camera setup.
declaration: "\nclass CamCalibrateDialog;"
fields:
  mNumCams:
    type: int
    annotation:
      - private
    description: __MISSING__
  staticMetaObject:
    description: __MISSING__
    type: const QMetaObject
  mpParent:
    annotation:
      - private
    type: QWidget *
    description: __MISSING__
  mpRecordDialog:
    annotation:
      - private
    type: RecordDialog *
    description: __MISSING__
  mMutex:
    annotation:
      - private
    description: __MISSING__
    type: QMutex
  mpProcess:
    description: __MISSING__
    annotation:
      - private
    type: QProcess *
  mImagesXml:
    annotation:
      - private
    type: QString
    description: __MISSING__
  mShellFilename:
    annotation:
      - private
    description: __MISSING__
    type: QString
  mpProjDetails:
    description: __MISSING__
    type: const ProjectDialog *
    annotation:
      - private
  mpUi:
    annotation:
      - private
    description: __MISSING__
    type: Ui::CamCalibrateDialog *
  mCalibrationImage:
    description: __MISSING__
    annotation:
      - private
    type: QString
tags:
  - class
owner: gwjensen
---
