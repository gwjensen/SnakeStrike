---
overloads:
  GuiCamera * CamMaker(std::unique_ptr<GuiCamInfo> &, const int, QObject *):
    arguments:
      - description:
        type: std::unique_ptr<GuiCamInfo> &
        name: iInfo
      - name: iIdx
        type: const int
        description: __OPTIONAL__
      - name: ipParent
        type: QObject *
        description: __OPTIONAL__
    description: This function returns a camera object that the SnakeStrike code knows how to use correctly.
    return: __OPTIONAL__
    signature_with_names: GuiCamera * CamMaker(std::unique_ptr<GuiCamInfo> & iInfo, const int iIdx, QObject * ipParent)
tags:
  - function
defined-in-file: "basler/CamMaker.h"
layout: function
owner: gwjensen
brief: This function is part of the camera plugin API. The functions are defined by a generic interface so that many different types of cameras can work with SnakeStrike and so that new cameras can use SnakeStrike without requiring re-compilation of the SnakeStrike codebase.
title: CamMaker
---
