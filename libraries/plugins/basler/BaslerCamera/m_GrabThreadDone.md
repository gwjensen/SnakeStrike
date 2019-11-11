---
overloads:
  void GrabThreadDone():
    return: __OPTIONAL__
    description: __MISSING__
    signature_with_names: void GrabThreadDone()
tags:
  - method
defined-in-file: "basler/BaslerCamera.h"
brief: Function called by GrabAndSaveImageTask when it has finished its work. It checks to see if all the threads are done processing, and if so, cleans up file writes and then signals that the camera is done capturing.
title: GrabThreadDone
layout: method
owner: gwjensen
---
