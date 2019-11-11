---
title: StartGrabbingRecord
overloads:
  void StartGrabbingRecord(QString, int):
    return: __OPTIONAL__
    signature_with_names: void StartGrabbingRecord(QString iDir, int iNumImages)
    description:
    arguments:
      - name: iDir
        description: __OPTIONAL__
        type: QString
      - description: __OPTIONAL__
        name: iNumImages
        type: int
owner: gwjensen
defined-in-file: "basler/BaslerCamera.h"
tags:
  - method
layout: method
brief: Start grabbing images that will be used for a capture. This tries not to drop any images.
---
