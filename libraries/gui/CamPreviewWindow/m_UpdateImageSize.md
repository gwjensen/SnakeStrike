---
layout: method
owner: gwjensen
tags:
  - method
defined-in-file: "projects/gui/CamPreviewWindow.h"
overloads:
  void UpdateImageSize(int64_t, int64_t):
    description: __MISSING__
    return: __OPTIONAL__
    arguments:
      - name: iWidth
        type: int64_t
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: iHeight
        type: int64_t
    signature_with_names: void UpdateImageSize(int64_t iWidth, int64_t iHeight)
title: UpdateImageSize
brief: Makes sure that the size of the images corresponds to the correct size. Between experiments the resolution of the cameras could change, so this forces the windows to update at the next available moment.
---
