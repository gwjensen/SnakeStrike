---
title: DeviceMaker
owner: gwjensen
overloads:
  GuiDevice * DeviceMaker():
    return: __OPTIONAL__
    description:
    signature_with_names: GuiDevice * DeviceMaker()
layout: function
defined-in-file: "basler/DeviceMaker.h"
tags:
  - function
brief: This function is part of the camera plugin API. The functions are defined by a generic interface so that many different types of cameras can work with SnakeStrike and so that new cameras can use SnakeStrike without requiring re-compilation of the SnakeStrike codebase. More specifically, this DeviceMaker function is a wrapper for the machinery needed to query found cameras on a system. A device object is an interface for accessing devices of a certain type on the system. For example, if I have 5 CameraX cameras, then the DeviceMaker function returns a pointer to an object that knows how to give me access to each of the individual CameraX cameras.
---
