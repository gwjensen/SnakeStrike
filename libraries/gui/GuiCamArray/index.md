---
brief: A way for sending commands to all cameras at the same time rather than having to loop through them all the time.
fields:
  mStopped:
    description: __MISSING__
    type: bool
    annotation:
      - private
  mCamLibPath:
    description: __MISSING__
    annotation:
      - private
    type: QString
  mConnected:
    description: __MISSING__
    annotation:
      - private
    type: bool
  staticMetaObject:
    description: __MISSING__
    type: const QMetaObject
  mOpened:
    annotation:
      - private
    description: __MISSING__
    type: bool
  mCamThreads:
    description: __MISSING__
    annotation:
      - private
    type: QVector<QThread *>
  mCamList:
    annotation:
      - private
    description: __MISSING__
    type: std::vector<std::unique_ptr<GuiCamera>>
  mpParent:
    type: QObject *
    annotation:
      - private
    description: __MISSING__
  mpLibHndl:
    annotation:
      - private
    type: void *
    description: __MISSING__
defined-in-file: "projects/gui/GuiCamArray.h"
owner: gwjensen
tags:
  - class
declaration: "\nclass GuiCamArray;"
layout: class
title: GuiCamArray
---
