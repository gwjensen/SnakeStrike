---
defined-in-file: "projects/gui/GrabAndSaveImageTask.h"
layout: method
brief:
tags:
  - method
overloads:
  GrabAndSaveImageTask(GuiCamera *, QTextStream *, uint32_t, const QString &, bool):
    signature_with_names: GrabAndSaveImageTask(GuiCamera * iCam, QTextStream * iStream, uint32_t iMaxNumImages, const QString & iSaveDir, bool iMakeVideo)
    arguments:
      - description: __OPTIONAL__
        type: GuiCamera *
        name: iCam
      - name: iStream
        description: __OPTIONAL__
        type: QTextStream *
      - description: __OPTIONAL__
        name: iMaxNumImages
        type: uint32_t
      - name: iSaveDir
        type: const QString &
        description: __OPTIONAL__
      - name: iMakeVideo
        type: bool
        description: Should the grabbed images be saved as frames to a video, or as individual images on the hard disk.
    description:
    return: __OPTIONAL__
  GrabAndSaveImageTask(GrabAndSaveImageTask &&):
    description:
    signature_with_names: GrabAndSaveImageTask(GrabAndSaveImageTask &&)
    arguments:
      - type: GrabAndSaveImageTask &&
        name: unnamed-0
        unnamed: true
        description: __OPTIONAL__
    return: __OPTIONAL__
  GrabAndSaveImageTask(const GrabAndSaveImageTask &):
    signature_with_names: GrabAndSaveImageTask(const GrabAndSaveImageTask &)
    arguments:
      - unnamed: true
        name: unnamed-0
        type: const GrabAndSaveImageTask &
        description: __OPTIONAL__
    return: __OPTIONAL__
    description:
owner: gwjensen
title: GrabAndSaveImageTask
is_ctor: true
---
