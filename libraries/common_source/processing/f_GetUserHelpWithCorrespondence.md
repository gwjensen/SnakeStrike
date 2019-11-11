---
tags:
  - function
brief: Since checking every possibility of combination for large numbers of cameras and points is computationally irresponsible, a quick and dirty workaround is to ask the user to provide us with the inital matching points for each image. The user is presented with a timestep where all cameras can see all of the points that are to be tracked. The pixel clusters that were found by the thresholding algorithm are shown by small markers. The user is asked to double click on the first point to track for the first camera, then the image for the second camera is shown and the user clicks the corresponding point in this image to what they clicked int the previous image. This proceeds through all of the cameras and then begins again with the first camera and the next point that is to be tracked.
owner: gwjensen
overloads:
  void GetUserHelpWithCorrespondence(const ImageSet &, const PixelSet &, bool (*)(), const std::pair<unsigned long, std::set<unsigned long>> &, const unsigned long, const std::vector<unsigned long> &, std::vector<std::vector<std::vector<int32_t>> > &, std::vector<std::vector<cv::Point2d>> &, const bool):
    arguments:
      - name: iRawImages
        description: __OPTIONAL__
        type: const ImageSet &
      - description: __OPTIONAL__
        type: const PixelSet &
        name: iPixelSet
      - type: bool (*)()
        name: CancelFunc
        description: __OPTIONAL__
      - name: iCamsToExclude
        type: const std::pair<unsigned long, std::set<unsigned long>> &
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: iNumPoints
        type: const unsigned long
      - name: iTimesteps
        description: __OPTIONAL__
        type: const std::vector<unsigned long> &
      - description: __OPTIONAL__
        type: std::vector<std::vector<std::vector<int32_t>> > &
        name: oUserFit
      - type: std::vector<std::vector<cv::Point2d>> &
        name: oMarkedPoints
        description: __OPTIONAL__
      - type: const bool
        description: __OPTIONAL__
        name: iAllowHiddenPointMarking
    return: __OPTIONAL__
    signature_with_names: void GetUserHelpWithCorrespondence(const ImageSet & iRawImages, const PixelSet & iPixelSet, bool (*)() CancelFunc, const std::pair<unsigned long, std::set<unsigned long>> & iCamsToExclude, const unsigned long iNumPoints, const std::vector<unsigned long> & iTimesteps, std::vector<std::vector<std::vector<int32_t>> > & oUserFit, std::vector<std::vector<cv::Point2d>> & oMarkedPoints, const bool iAllowHiddenPointMarking)
    description: __MISSING__
title: GetUserHelpWithCorrespondence
defined-in-file: "processing/Correspondence.h"
layout: function
---
