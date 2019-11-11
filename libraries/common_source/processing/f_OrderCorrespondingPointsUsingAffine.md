---
layout: function
brief: Different types of methods for ordering points in an unsupervised manner. No guarantee regarding which ones function and which ones don't. They are hanging around as examples and historical reasons.
defined-in-file: "processing/Correspondence.h"
title: OrderCorrespondingPointsUsingAffine
overloads:
  void OrderCorrespondingPointsUsingAffine(const std::vector<SmtPixel> &, const int, const int, const std::vector<SmtPixel> &, const std::vector<int> &, const CamMats &, const int, const std::pair<double, std::vector<SmtPixel>> &, const std::map<std::string, cv::Mat> &, std::vector<std::pair<double, std::vector<SmtPixel>> > &):
    arguments:
      - name: iCamFixedGroup
        description: __OPTIONAL__
        type: const std::vector<SmtPixel> &
      - name: iFixedIdx
        type: const int
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: iFixedCamIdx
        type: const int
      - description: __OPTIONAL__
        name: iCamUnorderedGroup
        type: const std::vector<SmtPixel> &
      - description: __OPTIONAL__
        name: iIndexesLeft
        type: const std::vector<int> &
      - description: __OPTIONAL__
        name: iCamMatrix
        type: const CamMats &
      - name: iCamIdx
        type: const int
        description: __OPTIONAL__
      - name: iCamWorkingGroup
        type: const std::pair<double, std::vector<SmtPixel>> &
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: iAffineMats
        type: const std::map<std::string, cv::Mat> &
      - name: oCamOrderedGroups
        type: std::vector<std::pair<double, std::vector<SmtPixel>> > &
        description: __OPTIONAL__
    return: __OPTIONAL__
    signature_with_names: void OrderCorrespondingPointsUsingAffine(const std::vector<SmtPixel> & iCamFixedGroup, const int iFixedIdx, const int iFixedCamIdx, const std::vector<SmtPixel> & iCamUnorderedGroup, const std::vector<int> & iIndexesLeft, const CamMats & iCamMatrix, const int iCamIdx, const std::pair<double, std::vector<SmtPixel>> & iCamWorkingGroup, const std::map<std::string, cv::Mat> & iAffineMats, std::vector<std::pair<double, std::vector<SmtPixel>> > & oCamOrderedGroups)
    description: __MISSING__
  void OrderCorrespondingPointsUsingAffine(const PixelSet &, const CamMats &, const std::vector<std::map<std::string, cv::Mat>> &, PixelSet &, ImageSet):
    arguments:
      - type: const PixelSet &
        name: iPixelSet
        description: __OPTIONAL__
      - type: const CamMats &
        name: iCamMatrix
        description: __OPTIONAL__
      - name: iAffineMats
        description: __OPTIONAL__
        type: const std::vector<std::map<std::string, cv::Mat>> &
      - name: oPixelSet
        type: PixelSet &
        description: __OPTIONAL__
      - name: iImageSet
        description: __OPTIONAL__
        type: ImageSet
    signature_with_names: void OrderCorrespondingPointsUsingAffine(const PixelSet & iPixelSet, const CamMats & iCamMatrix, const std::vector<std::map<std::string, cv::Mat>> & iAffineMats, PixelSet & oPixelSet, ImageSet iImageSet)
    description: __MISSING__
    return: __OPTIONAL__
owner: gwjensen
tags:
  - function
---
