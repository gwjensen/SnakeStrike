---
owner: gwjensen
layout: class
brief: Items in the tree description of the cameras.
tags:
  - class
title: CameraTreeItem
declaration: "\nclass CameraTreeItem;"
fields:
  mChildItems:
    type: QList<CameraTreeItem *>
    description: __MISSING__
    annotation:
      - private
  mItemData:
    annotation:
      - private
    type: QVariant
    description: __MISSING__
  mpParentItem:
    annotation:
      - private
    type: CameraTreeItem *
    description: __MISSING__
defined-in-file: "projects/gui/CameraTreeItem.h"
---
