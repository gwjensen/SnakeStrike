---
layout: method
title: FindPreviewWindow
defined-in-file: "projects/gui/MainWindow.h"
overloads:
  CamPreviewWindow * FindPreviewWindow(const int) const:
    return: __OPTIONAL__
    signature_with_names: CamPreviewWindow * FindPreviewWindow(const int iCamIdx) const
    description: __MISSING__
    arguments:
      - type: const int
        description: __OPTIONAL__
        name: iCamIdx
    annotation:
      - private
owner: gwjensen
brief: Given a camera index return a pointer to the preview window associated with that camera.
tags:
  - method
---
