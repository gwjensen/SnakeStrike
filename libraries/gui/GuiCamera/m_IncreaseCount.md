---
owner: gwjensen
overloads:
  uint64_t IncreaseCount():
    signature_with_names: uint64_t IncreaseCount()
    return: __OPTIONAL__
    description:
tags:
  - method
brief: Increase the counter for the current image index value. Each image grabbed should have a unique number that represents where it should be placed in the sequence of images captured. If the user wants to capture 3000 images, then the images will have indexes from 0-2999 inclusive, and this function keeps track of that. When running in a multi-threaded environment this function needs the correct guards to ensure atomicity.
defined-in-file: "projects/gui/GuiCamera.h"
layout: method
title: IncreaseCount
---
