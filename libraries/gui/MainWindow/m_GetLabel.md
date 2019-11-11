---
owner: gwjensen
overloads:
  bool GetLabel(const int, CamPreviewCanvas *&):
    signature_with_names: bool GetLabel(const int iLabelIdx, CamPreviewCanvas *& oLabelPtr)
    arguments:
      - description: __OPTIONAL__
        type: const int
        name: iLabelIdx
      - name: oLabelPtr
        description: __OPTIONAL__
        type: CamPreviewCanvas *&
    description: __MISSING__
    return: __OPTIONAL__
title: GetLabel
brief: Return a pointer to the canvas used to show camera preview images.
tags:
  - method
defined-in-file: "projects/gui/MainWindow.h"
layout: method
---
