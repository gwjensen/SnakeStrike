---
overloads:
  QModelIndex index(int, int, const QModelIndex &) const:
    description: __MISSING__
    return: __OPTIONAL__
    signature_with_names: QModelIndex index(int iRow, int iColumn, const QModelIndex & iParent) const
    arguments:
      - name: iRow
        type: int
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: int
        name: iColumn
      - description: __OPTIONAL__
        name: iParent
        type: const QModelIndex &
tags:
  - method
title: index
layout: method
defined-in-file: "projects/gui/CameraInfoTree.h"
owner: gwjensen
brief: Overridden method from QAbstractItemModel. It returns the index of a row and column given a parent.
---
