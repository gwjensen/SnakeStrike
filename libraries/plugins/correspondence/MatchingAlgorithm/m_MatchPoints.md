---
tags:
  - method
layout: method
defined-in-file: "correspondence/MatchingAlgorithm.h"
title: MatchPoints
owner: gwjensen
overloads:
  void MatchPoints(uint64_t &, PixelSet &, const TrackerConfigFile &, const ImageSet &, const std::pair<unsigned long, std::set<unsigned long>> &, bool (*)(), std::vector<std::vector<std::vector<int32_t>> > &, std::vector<double> &, PixelSet &):
    signature_with_names: void MatchPoints(uint64_t & oFirstMatchTimestep, PixelSet & ioPixelsToTrack, const TrackerConfigFile & iConfig, const ImageSet & iTimestepImages, const std::pair<unsigned long, std::set<unsigned long>> & iCamIndexesToExclude, bool (*)() CancelFunc, std::vector<std::vector<std::vector<int32_t>> > & oBestFit, std::vector<double> & oBestFitErrors, PixelSet & oFirstTimestepWithAllPoints)
    arguments:
      - description: __OPTIONAL__
        name: oFirstMatchTimestep
        type: uint64_t &
      - description: __OPTIONAL__
        type: PixelSet &
        name: ioPixelsToTrack
      - name: iConfig
        type: const TrackerConfigFile &
        description: __OPTIONAL__
      - type: const ImageSet &
        description: __OPTIONAL__
        name: iTimestepImages
      - description: __OPTIONAL__
        name: iCamIndexesToExclude
        type: const std::pair<unsigned long, std::set<unsigned long>> &
      - name: CancelFunc
        type: bool (*)()
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: oBestFit
        type: std::vector<std::vector<std::vector<int32_t>> > &
      - type: std::vector<double> &
        description: __OPTIONAL__
        name: oBestFitErrors
      - description: __OPTIONAL__
        name: oFirstTimestepWithAllPoints
        type: PixelSet &
    description: __MISSING__
    return: __OPTIONAL__
brief: This function is the function that will be called by the SnakeStrike code. You can't change the signature of the function, but you can change the internals to satisfy the needs of your algorithm.
---
