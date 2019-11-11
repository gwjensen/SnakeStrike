---
brief: Holds the subwindows for each camera's preview.
title: CamPreviewWindow
declaration: "\nclass CamPreviewWindow;"
defined-in-file: "projects/gui/CamPreviewWindow.h"
layout: class
owner: gwjensen
tags:
  - class
fields:
  mIsUntitled:
    description: __MISSING__
    type: bool
    annotation:
      - private
  mCamIdx:
    description: __MISSING__
    annotation:
      - private
    type: int
  mShouldClose:
    annotation:
      - private
    description: __MISSING__
    type: bool
  staticMetaObject:
    type: const QMetaObject
    description: __MISSING__
  mpMyInstance:
    description: __MISSING__
    type: QMdiSubWindow *
    annotation:
      - private
  mpLabel:
    description: __MISSING__
    type: CamPreviewCanvas *
    annotation:
      - private
  mpParent:
    annotation:
      - private
    description: __MISSING__
    type: QMdiArea *
---
