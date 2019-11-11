---
tags:
  - function
owner: gwjensen
title: OrderCorrespondingPoints
defined-in-file: "processing/Correspondence.h"
brief: Different types of methods for ordering points in an unsupervised manner. No guarantee regarding which ones function and which ones don't. They are hanging around as examples and historical reasons.
layout: function
overloads:
  void OrderCorrespondingPoints(const std::vector<SmtPixel> &, const int, const int, const std::vector<SmtPixel> &, const std::vector<int> &, const CamMats &, const int, const std::pair<double, std::vector<SmtPixel>> &, std::vector<std::pair<double, std::vector<SmtPixel>> > &):
    return: __OPTIONAL__
    description:
    signature_with_names: void OrderCorrespondingPoints(const std::vector<SmtPixel> & iCamFixedGroup, const int iFixedIdx, const int iFixedCamIdx, const std::vector<SmtPixel> & iCamUnorderedGroup, const std::vector<int> & iIndexesLeft, const CamMats & iCamMatrix, const int iCamIdx, const std::pair<double, std::vector<SmtPixel>> & iCamWorkingGroup, std::vector<std::pair<double, std::vector<SmtPixel>> > & oCamOrderedGroups)
    arguments:
      - name: iCamFixedGroup
        type: const std::vector<SmtPixel> &
        description: __OPTIONAL__
      - type: const int
        description: __OPTIONAL__
        name: iFixedIdx
      - type: const int
        description: __OPTIONAL__
        name: iFixedCamIdx
      - description: __OPTIONAL__
        name: iCamUnorderedGroup
        type: const std::vector<SmtPixel> &
      - name: iIndexesLeft
        type: const std::vector<int> &
        description: __OPTIONAL__
      - name: iCamMatrix
        type: const CamMats &
        description: __OPTIONAL__
      - type: const int
        description: __OPTIONAL__
        name: iCamIdx
      - description: __OPTIONAL__
        name: iCamWorkingGroup
        type: const std::pair<double, std::vector<SmtPixel>> &
      - name: oCamOrderedGroups
        description: __OPTIONAL__
        type: std::vector<std::pair<double, std::vector<SmtPixel>> > &
  void OrderCorrespondingPoints(const PixelSet &, const CamMats &, PixelSet &, ImageSet):
    description:
    signature_with_names: void OrderCorrespondingPoints(const PixelSet & iPixelSet, const CamMats & iCamMatrix, PixelSet & oPixelSet, ImageSet iImageSet)
    arguments:
      - description: __OPTIONAL__
        type: const PixelSet &
        name: iPixelSet
      - description: __OPTIONAL__
        type: const CamMats &
        name: iCamMatrix
      - type: PixelSet &
        name: oPixelSet
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: ImageSet
        name: iImageSet
    return: __OPTIONAL__
---
