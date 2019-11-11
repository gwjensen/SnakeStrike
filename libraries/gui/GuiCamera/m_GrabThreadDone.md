---
tags:
  - method
defined-in-file: "projects/gui/GuiCamera.h"
overloads:
  void GrabThreadDone():
    description: __MISSING__
    return: __OPTIONAL__
    signature_with_names: void GrabThreadDone()
brief: Function called by GrabAndSaveImageTask when it has finished its work. It checks to see if all the threads are done processing, and if so, cleans up file writes and then signals that the camera is done capturing.
layout: method
title: GrabThreadDone
owner: gwjensen
---
