---
owner: gwjensen
overloads:
  void MaxBuffer(int64_t):
    signature_with_names: void MaxBuffer(int64_t iNewNum)
    return: __OPTIONAL__
    arguments:
      - type: int64_t
        name: iNewNum
        description: __OPTIONAL__
    description: __MISSING__
  int64_t MaxBuffer() const:
    return: __OPTIONAL__
    description: __MISSING__
    signature_with_names: int64_t MaxBuffer() const
brief: What is the maximum size of the image buffer used to hold images. If the image buffer is too small and the camera is taking images fast enough, there will be images that are dropped.
defined-in-file: "projects/gui/GuiCamera.h"
layout: method
title: MaxBuffer
tags:
  - method
---
