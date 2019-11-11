---
brief:
owner: gwjensen
title: "Functions for Converting Between QImage and QPixmap"
tags:
  - method
defined-in-file: "projects/gui/GuiImage.h"
overloads:
  static QImage ToQImage(cv::Mat):
    signature_with_names: static QImage ToQImage(cv::Mat iMat)
    description: __MISSING__
    arguments:
      - description: __OPTIONAL__
        name: iMat
        type: cv::Mat
    return: __OPTIONAL__
  static QPixmap ToQPixmap(cv::Mat):
    description: __MISSING__
    return: __OPTIONAL__
    arguments:
      - name: iMat
        type: cv::Mat
        description: __OPTIONAL__
    signature_with_names: static QPixmap ToQPixmap(cv::Mat iMat)
  QPixmap ToQPixmap():
    description: __MISSING__
    signature_with_names: QPixmap ToQPixmap()
    return: __OPTIONAL__
  QImage ToQImage():
    signature_with_names: QImage ToQImage()
    description: __MISSING__
    return: __OPTIONAL__
layout: method
---
