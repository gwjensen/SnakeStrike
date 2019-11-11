---
brief: Go through the images and find groups of pixels to create clusters of pixels.
overloads:
  void GetPixelClusters(const ImageSet &, bool (*)(), const std::pair<unsigned long, std::set<unsigned long>> &, const int, PixelClusterSet &):
    signature_with_names: void GetPixelClusters(const ImageSet & iImagesSet, bool (*)() CancelFunc, const std::pair<unsigned long, std::set<unsigned long>> & iCamIndexesToExclude, const int iMaxNumClusters, PixelClusterSet & oClusterSet)
    description: __MISSING__
    arguments:
      - type: const ImageSet &
        description: __OPTIONAL__
        name: iImagesSet
      - type: bool (*)()
        description: __OPTIONAL__
        name: CancelFunc
      - type: const std::pair<unsigned long, std::set<unsigned long>> &
        name: iCamIndexesToExclude
        description: __OPTIONAL__
      - name: iMaxNumClusters
        description: __OPTIONAL__
        type: const int
      - name: oClusterSet
        type: PixelClusterSet &
        description: __OPTIONAL__
    return: __OPTIONAL__
tags:
  - function
defined-in-file: "image/ImageSet.h"
title: GetPixelClusters
owner: gwjensen
layout: function
---
