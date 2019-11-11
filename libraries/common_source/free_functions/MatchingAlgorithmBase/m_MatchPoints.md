---
overloads:
  void MatchPoints(uint64_t &, PixelSet &, const TrackerConfigFile &, const ImageSet &, const std::pair<unsigned long, std::set<unsigned long>> &, bool (*)(), std::vector<std::vector<std::vector<int32_t>> > &, std::vector<double> &, PixelSet &):
    arguments:
      - type: uint64_t &
        description: The index of the timestep where all of the cameras see the correct number of points.
        name: oFirstMatchTimestep
      - name: ioPixelsToTrack
        description: __OPTIONAL__
        type: PixelSet &
      - description: __OPTIONAL__
        type: const TrackerConfigFile &
        name: iConfig
      - type: const ImageSet &
        name: iTimestepImages
        description: The images used to get the thresholded pixels.
      - type: const std::pair<unsigned long, std::set<unsigned long>> &
        description: The user can decide not to use all cameras in the correspondence. Perhaps because one camera doesn't have a good view. This contains the camera indexes to not use.
        name: iCamIndexesToExclude
      - description: This function returns whether or not the user has attempted to cancel the action.
        type: bool (*)()
        name: CancelFunc
      - description: The ordering of
        name: oBestFit
        type: std::vector<std::vector<std::vector<int32_t>> > &
      - type: std::vector<double> &
        description: __OPTIONAL__
        name: oBestFitErrors
      - description: __OPTIONAL__
        name: oFirstTimestepWithAllPoints
        type: PixelSet &
    description: This is the interface function your correspondence plugin code needs to follow.
    signature_with_names: void MatchPoints(uint64_t & oFirstMatchTimestep, PixelSet & ioPixelsToTrack, const TrackerConfigFile & iConfig, const ImageSet & iTimestepImages, const std::pair<unsigned long, std::set<unsigned long>> & iCamIndexesToExclude, bool (*)() CancelFunc, std::vector<std::vector<std::vector<int32_t>> > & oBestFit, std::vector<double> & oBestFitErrors, PixelSet & oFirstTimestepWithAllPoints)
    return: __OPTIONAL__
layout: method
tags:
  - method
title: MatchPoints
owner: gwjensen
defined-in-file: "MatchingAlgorithmBase.h"
brief: This is the function that does all of the heavy lifting for pairing up points from different camera views.
---
