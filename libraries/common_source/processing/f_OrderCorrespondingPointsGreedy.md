---
owner: gwjensen
overloads:
  void OrderCorrespondingPointsGreedy(const PixelSet &, const CamMats &, PixelSet &):
    description:
    arguments:
      - name: iPixelSet
        type: const PixelSet &
        description: __OPTIONAL__
      - type: const CamMats &
        description: __OPTIONAL__
        name: iCamMatrix
      - description: __OPTIONAL__
        name: oPixelSet
        type: PixelSet &
    signature_with_names: void OrderCorrespondingPointsGreedy(const PixelSet & iPixelSet, const CamMats & iCamMatrix, PixelSet & oPixelSet)
    return: __OPTIONAL__
layout: function
brief: Different types of methods for ordering points in an unsupervised manner. No guarantee regarding which ones function and which ones don't. They are hanging around as examples and historical reasons.
defined-in-file: "processing/Correspondence.h"
tags:
  - function
title: OrderCorrespondingPointsGreedy
---
