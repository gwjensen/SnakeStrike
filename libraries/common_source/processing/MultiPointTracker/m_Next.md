---
owner: gwjensen
brief:
title: Next
defined-in-file: "processing/MultiPointTracker.h"
overloads:
  bool Next(const std::vector<SmtPixel> &, uint32_t, std::vector<SmtPixel> &):
    arguments:
      - description: __OPTIONAL__
        name: iUnorderedPoints
        type: const std::vector<SmtPixel> &
      - name: iTimestep
        description: __OPTIONAL__
        type: uint32_t
      - name: oCorresPoints
        description: __OPTIONAL__
        type: std::vector<SmtPixel> &
    return: __OPTIONAL__
    signature_with_names: bool Next(const std::vector<SmtPixel> & iUnorderedPoints, uint32_t iTimestep, std::vector<SmtPixel> & oCorresPoints)
    description:
tags:
  - method
layout: method
---
