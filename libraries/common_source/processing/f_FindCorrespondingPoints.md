---
defined-in-file: "processing/Correspondence.h"
owner: gwjensen
tags:
  - function
overloads:
  void FindCorrespondingPoints(const PixelSet &, std::vector<std::vector<std::vector<int32_t>> > &, std::vector<double> &):
    return: __OPTIONAL__
    description:
    signature_with_names: void FindCorrespondingPoints(const PixelSet & iPixelSet, std::vector<std::vector<std::vector<int32_t>> > & oBestFit, std::vector<double> & oBestFitErrors)
    arguments:
      - name: iPixelSet
        description: __OPTIONAL__
        type: const PixelSet &
      - type: std::vector<std::vector<std::vector<int32_t>> > &
        name: oBestFit
        description: __OPTIONAL__
      - name: oBestFitErrors
        type: std::vector<double> &
        description: __OPTIONAL__
layout: function
title: FindCorrespondingPoints
brief: Automatic method for finding point correspondences across camera images. Works when the number of points is low, like 5 or smaller. The number of combinations to calculate is given by (maxNumPoints!)^(numCam -1). For example, 3 points with 6 cams = ~8000 combinations, where as 5 points with 6 cams = ~24 billion combos)
---
