---
tags:
  - method
title: ReadXMLImagesFile
defined-in-file: "projects/gui/ThresholdDialog.h"
owner: gwjensen
overloads:
  void ReadXMLImagesFile(const QString &, QVector<QString> &, int &, uint32_t &):
    return: __OPTIONAL__
    signature_with_names: void ReadXMLImagesFile(const QString & iLocation, QVector<QString> & oImageLocations, int & oNumCams, uint32_t & oNumImagesFound)
    arguments:
      - description: __OPTIONAL__
        name: iLocation
        type: const QString &
      - description: __OPTIONAL__
        type: QVector<QString> &
        name: oImageLocations
      - type: int &
        description: __OPTIONAL__
        name: oNumCams
      - type: uint32_t &
        description: __OPTIONAL__
        name: oNumImagesFound
    description: __MISSING__
    annotation:
      - protected
brief: Read the xml file that has the paths for the images to read in.
layout: method
---
