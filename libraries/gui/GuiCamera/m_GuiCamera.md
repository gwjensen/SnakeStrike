---
owner: gwjensen
layout: method
overloads:
  explicit GuiCamera(QObject *):
    return: __OPTIONAL__
    signature_with_names: explicit GuiCamera(QObject * ipParent)
    description:
    arguments:
      - type: QObject *
        name: ipParent
        description: __OPTIONAL__
  GuiCamera(const GuiCamera &):
    description:
    arguments:
      - name: unnamed-0
        unnamed: true
        description: __OPTIONAL__
        type: const GuiCamera &
    return: __OPTIONAL__
    signature_with_names: GuiCamera(const GuiCamera &)
  explicit GuiCamera(std::unique_ptr<GuiCamInfo> &, const int, QObject *):
    description:
    arguments:
      - type: std::unique_ptr<GuiCamInfo> &
        description: __OPTIONAL__
        name: iInfo
      - description: __OPTIONAL__
        type: const int
        name: iIdx
      - name: ipParent
        description: __OPTIONAL__
        type: QObject *
    return: __OPTIONAL__
    signature_with_names: explicit GuiCamera(std::unique_ptr<GuiCamInfo> & iInfo, const int iIdx, QObject * ipParent)
title: GuiCamera
is_ctor: true
defined-in-file: "projects/gui/GuiCamera.h"
tags:
  - method
brief:
---
