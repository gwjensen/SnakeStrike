---
defined-in-file: "basler/BaslerCamera.h"
is_ctor: true
title: BaslerCamera
owner: gwjensen
tags:
  - method
brief:
overloads:
  BaslerCamera(std::unique_ptr<GuiCamInfo> &, const int, QObject *):
    signature_with_names: BaslerCamera(std::unique_ptr<GuiCamInfo> & iInfo, const int iIdx, QObject * ipParent)
    return: __OPTIONAL__
    description:
    arguments:
      - name: iInfo
        type: std::unique_ptr<GuiCamInfo> &
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: iIdx
        type: const int
      - type: QObject *
        description: __OPTIONAL__
        name: ipParent
  BaslerCamera(const BaslerCamera &):
    return: __OPTIONAL__
    description:
    arguments:
      - type: const BaslerCamera &
        unnamed: true
        description: __OPTIONAL__
        name: unnamed-0
    signature_with_names: BaslerCamera(const BaslerCamera &)
layout: method
---
