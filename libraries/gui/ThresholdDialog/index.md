---
defined-in-file: "projects/gui/ThresholdDialog.h"
declaration: "\nclass ThresholdDialog;"
owner: gwjensen
title: ThresholdDialog
layout: class
tags:
  - class
fields:
  mMaxTimestep:
    description: __MISSING__
    type: uint32_t
    annotation:
      - private
  mGrabColor:
    annotation:
      - private
    type: bool
    description: __MISSING__
  mpUi:
    description: __MISSING__
    annotation:
      - private
    type: Ui::ThresholdDialog *
  mImageList:
    annotation:
      - private
    description: __MISSING__
    type: QVector<GuiImage *>
  mpParent:
    type: TrackingDialog *
    annotation:
      - private
    description: __MISSING__
  mpProjInfo:
    annotation:
      - private
    type: ProjectDialog *
    description: __MISSING__
  mMutex:
    type: QMutex
    description: __MISSING__
    annotation:
      - private
  mThreshPreviewList:
    description: __MISSING__
    type: QVector<CamPreviewCanvas *>
    annotation:
      - private
  mNumCams:
    annotation:
      - private
    description: __MISSING__
    type: int
  mFileListMap:
    annotation:
      - private
    description: __MISSING__
    type: QMap<QString, QString>
  mImageLock:
    annotation:
      - private
    type: QReadWriteLock
    description: __MISSING__
  mImagesXml:
    annotation:
      - private
    type: QString
    description: __MISSING__
  mThreshWindowList:
    annotation:
      - private
    description: __MISSING__
    type: QVector<CamPreviewWindow *>
  staticMetaObject:
    description: __MISSING__
    type: const QMetaObject
brief: Dialog window used to help the user find the correct thresholds needed to track their marker points in the images.
---
