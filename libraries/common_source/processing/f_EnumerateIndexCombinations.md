---
brief: Enumerates all possible combinations of a 2 tier nested list where the length of the first list is variable, and each list contained is of the same length.
owner: gwjensen
tags:
  - function
overloads:
  void EnumerateIndexCombinations(unsigned long, unsigned long, std::vector<std::vector<std::vector<int32_t>> > &):
    arguments:
      - name: iNumPoints
        description: __OPTIONAL__
        type: unsigned long
      - description: __OPTIONAL__
        name: iNumCams
        type: unsigned long
      - type: std::vector<std::vector<std::vector<int32_t>> > &
        name: oIndexesPerPoint
        description: __OPTIONAL__
    description:
    signature_with_names: void EnumerateIndexCombinations(unsigned long iNumPoints, unsigned long iNumCams, std::vector<std::vector<std::vector<int32_t>> > & oIndexesPerPoint)
    return: __OPTIONAL__
layout: function
defined-in-file: "processing/Correspondence.h"
title: EnumerateIndexCombinations
---
