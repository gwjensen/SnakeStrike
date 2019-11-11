---
tags:
  - method
owner: gwjensen
defined-in-file: "image/SmtImage.h"
title: GetPixelClusters
overloads:
  void GetPixelClusters(const unsigned int, std::vector<PixelCluster> &) const:
    signature_with_names: void GetPixelClusters(const unsigned int iMaxNumClusters, std::vector<PixelCluster> & oGroupedPoints) const
    return: __OPTIONAL__
    arguments:
      - description: __OPTIONAL__
        name: iMaxNumClusters
        type: const unsigned int
      - name: oGroupedPoints
        type: std::vector<PixelCluster> &
        description: __OPTIONAL__
    description:
brief: Get a list of the iMaxNumClusters largest clusters in this image. This really only makes sense when the image is a binary image. Garbage in, garbage out.
layout: method
---
