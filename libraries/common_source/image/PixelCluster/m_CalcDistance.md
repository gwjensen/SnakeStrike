---
layout: method
brief: Calculate euclidean distance between cluster center and pixel.
title: CalcDistance
owner: gwjensen
defined-in-file: "image/PixelCluster.h"
tags:
  - method
overloads:
  float CalcDistance(const PixelCluster &) const:
    return: __OPTIONAL__
    arguments:
      - type: const PixelCluster &
        description: __OPTIONAL__
        name: iCluster
    description:
    signature_with_names: float CalcDistance(const PixelCluster & iCluster) const
  float CalcDistance(const SmtPixel &) const:
    description:
    arguments:
      - type: const SmtPixel &
        description: __OPTIONAL__
        name: iPoint
    return: __OPTIONAL__
    signature_with_names: float CalcDistance(const SmtPixel & iPoint) const
---
