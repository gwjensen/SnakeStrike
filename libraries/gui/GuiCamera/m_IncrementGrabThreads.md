---
owner: gwjensen
layout: method
title: IncrementGrabThreads
overloads:
  uint32_t IncrementGrabThreads():
    description: __MISSING__
    return: __OPTIONAL__
    signature_with_names: uint32_t IncrementGrabThreads()
brief: While capturing images there are lots of threads running to take those images from the image buffer and save them (GrabAndSaveImageTask). Whenever a thread associated with this camera starts its work, it calls this function to increase the count of the number of threads accessing this object.
tags:
  - method
defined-in-file: "projects/gui/GuiCamera.h"
---
