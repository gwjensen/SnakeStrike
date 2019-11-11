---
defined-in-file: "projects/gui/MainWindow.h"
title: UpdateCamPreviewWindow
tags:
  - method
layout: method
overloads:
  void UpdateCamPreviewWindow(int, QPixmap):
    arguments:
      - type: int
        name: iCamIdx
        description: __OPTIONAL__
      - name: image
        description: __OPTIONAL__
        type: QPixmap
    signature_with_names: void UpdateCamPreviewWindow(int iCamIdx, QPixmap image)
    annotation:
      - private
    return: __OPTIONAL__
    description: __MISSING__
owner: gwjensen
brief: Update internal status of the preview window when something has changed that could affect it. For example the size of the images captured by a camera.
---
