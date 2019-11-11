---
layout: method
title: StartGrabbingPreview
owner: gwjensen
defined-in-file: "projects/gui/GuiCamera.h"
brief: Instruct the camera to start grabbing images, but these images can be lossy as we are going to use them for a preview and not a strict image collection. If your camera has the ability to grab from the top of the image buffer and discard the rest, this is where that functionality should be used as we don't care about dropped images since the human eye only see ~30 frames a second anyways.
overloads:
  void StartGrabbingPreview():
    signature_with_names: void StartGrabbingPreview()
    description: __MISSING__
    return: __OPTIONAL__
tags:
  - method
---
