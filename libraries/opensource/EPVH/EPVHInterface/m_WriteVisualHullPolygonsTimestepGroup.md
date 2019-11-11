---
tags:
  - method
brief: __MISSING__
defined-in-file: ""
owner: gwjensen
layout: method
title: WriteVisualHullPolygonsTimestepGroup
overloads:
  static void WriteVisualHullPolygonsTimestepGroup(const int, const int, const std::string &, const ImageSet &, const ImageSet &, const CamMats *const):
    description: __MISSING__
    return: __OPTIONAL__
    annotation:
      - protected
    signature_with_names: static void WriteVisualHullPolygonsTimestepGroup(const int iStartTimestep, const int iEndTimestep, const std::string & iFileDirectory, const ImageSet & iImages, const ImageSet & iBackgroundImages, const CamMats *const iCamMatrices)
    arguments:
      - description: __OPTIONAL__
        name: iStartTimestep
        type: const int
      - description: __OPTIONAL__
        type: const int
        name: iEndTimestep
      - description: __OPTIONAL__
        type: const std::string &
        name: iFileDirectory
      - description: __OPTIONAL__
        name: iImages
        type: const ImageSet &
      - name: iBackgroundImages
        description: __OPTIONAL__
        type: const ImageSet &
      - description: __OPTIONAL__
        name: iCamMatrices
        type: const CamMats *const
---
