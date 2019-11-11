---
title: DecrementGrabThreads
overloads:
  uint32_t DecrementGrabThreads():
    description: __MISSING__
    signature_with_names: uint32_t DecrementGrabThreads()
    return: __OPTIONAL__
defined-in-file: "projects/gui/GuiCamera.h"
layout: method
brief: While capturing images there are lots of threads running to take those images from the image buffer and save them (GrabAndSaveImageTask). Whenever a thread associated with this camera finishes its work, it calls this function to decrease the count of the number of threads accessing this object.
owner: gwjensen
tags:
  - method
---
