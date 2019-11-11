---
owner: gwjensen
tags:
  - class
fields:
  mpProcess:
    type: QProcess *
    description: __MISSING__
    annotation:
      - private
  mpOutputFile:
    type: QFile *
    description: __MISSING__
    annotation:
      - private
  mRunning:
    type: bool
    description: __MISSING__
    annotation:
      - private
  mpProjInfo:
    description: __MISSING__
    type: ProjectDialog *
    annotation:
      - private
  mpParent:
    annotation:
      - private
    description: __MISSING__
    type: QWidget *
  mMutex:
    type: QMutex
    annotation:
      - private
    description: __MISSING__
  staticMetaObject:
    description: __MISSING__
    type: const QMetaObject
  mpUi:
    annotation:
      - private
    description: __MISSING__
    type: Ui::TrackingDialog *
  mNumCams:
    annotation:
      - private
    description: __MISSING__
    type: int
layout: class
declaration: "\nclass TrackingDialog;"
defined-in-file: "projects/gui/TrackingDialog.h"
title: TrackingDialog
brief: This is the dialog for doing the triangulation of the pixels one is trying to track. This dialog spawns the thresholding dialog when the user needs to fine-tune the constraints of the thresholding.
---
