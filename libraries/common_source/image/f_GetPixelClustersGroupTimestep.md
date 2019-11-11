---
brief: Helper function to do pixel cluster search multi-threaded. Shouldn't be directly called.
layout: function
defined-in-file: "image/ImageSet.h"
overloads:
  void GetPixelClustersGroupTimestep(int, int, const ImageSet &, bool (*)(), const int, const std::pair<unsigned long, std::set<unsigned long>> &, PixelClusterSet &):
    signature_with_names: void GetPixelClustersGroupTimestep(int iStartTimestep, int iEndTimestep, const ImageSet & iImageSet, bool (*)() CancelFunc, const int iMaxNumClusters, const std::pair<unsigned long, std::set<unsigned long>> & iCamIndexesToExclude, PixelClusterSet & oClusterSet)
    description: __MISSING__
    arguments:
      - type: int
        description: __OPTIONAL__
        name: iStartTimestep
      - name: iEndTimestep
        type: int
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: const ImageSet &
        name: iImageSet
      - description: __OPTIONAL__
        type: bool (*)()
        name: CancelFunc
      - name: iMaxNumClusters
        type: const int
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: const std::pair<unsigned long, std::set<unsigned long>> &
        name: iCamIndexesToExclude
      - type: PixelClusterSet &
        description: __OPTIONAL__
        name: oClusterSet
    return: __OPTIONAL__
title: GetPixelClustersGroupTimestep
owner: gwjensen
tags:
  - function
---
