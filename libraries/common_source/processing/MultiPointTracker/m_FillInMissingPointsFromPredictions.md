---
defined-in-file: "processing/MultiPointTracker.h"
layout: method
brief:
overloads:
  bool FillInMissingPointsFromPredictions(const std::vector<SmtPixel> &, const uint32_t, std::vector<SmtPixel> &):
    return: __OPTIONAL__
    signature_with_names: bool FillInMissingPointsFromPredictions(const std::vector<SmtPixel> & iIncompleteList, const uint32_t iTimestep, std::vector<SmtPixel> & oFilledInList)
    description:
    arguments:
      - name: iIncompleteList
        description: __OPTIONAL__
        type: const std::vector<SmtPixel> &
      - type: const uint32_t
        name: iTimestep
        description: __OPTIONAL__
      - name: oFilledInList
        description: __OPTIONAL__
        type: std::vector<SmtPixel> &
tags:
  - method
title: FillInMissingPointsFromPredictions
owner: gwjensen
---
