---
declaration: "\nclass BufferingDialog;"
title: BufferingDialog
tags:
  - class
owner: gwjensen
brief: The class that describes the buffering dialog available in the GUI.
defined-in-file: "projects/gui/BufferingDialog.h"
fields:
  staticMetaObject:
    type: const QMetaObject
    description: __MISSING__
  mMutex:
    annotation:
      - private
    type: QMutex
    description: __MISSING__
  mImagesSaved:
    annotation:
      - private
    description: __MISSING__
    type: int
  mpUi:
    annotation:
      - private
    type: Ui::BufferingDialog *
    description: __MISSING__
  mImagesDropped:
    type: int
    description: __MISSING__
    annotation:
      - private
layout: class
---
