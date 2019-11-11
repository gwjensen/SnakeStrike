---
layout: method
title: MaxBuffer
brief: What is the maximum size of the image buffer used to hold images. If the image buffer is too small and the camera is taking images fast enough, there will be images that are dropped.
overloads:
  void MaxBuffer(int64_t):
    signature_with_names: void MaxBuffer(int64_t iNewNum)
    arguments:
      - name: iNewNum
        description: __OPTIONAL__
        type: int64_t
    description: __MISSING__
    return: __OPTIONAL__
  int64_t MaxBuffer() const:
    return: __OPTIONAL__
    signature_with_names: int64_t MaxBuffer() const
    description: __MISSING__
owner: gwjensen
defined-in-file: "basler/BaslerCamera.h"
tags:
  - method
---
