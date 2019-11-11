---
layout: function
title: MergeThresholdHalves
brief: When thresholding using a color like red, the thresholding has to be done twice because red overlaps the 0/360 boundary of the HSV scale. So it has to be done once for X to 0 and then again for 360 to Y if the H value of the color range is X to Y. Then the two halves need to be merged. This function does that.
defined-in-file: "image/ImageSet.h"
overloads:
  void MergeThresholdHalves(const ImageSet &, const ImageSet &, ImageSet &):
    return: __OPTIONAL__
    signature_with_names: void MergeThresholdHalves(const ImageSet & iImageSet1, const ImageSet & iImageSet2, ImageSet & oImageSet)
    arguments:
      - name: iImageSet1
        type: const ImageSet &
        description: __OPTIONAL__
      - type: const ImageSet &
        description: __OPTIONAL__
        name: iImageSet2
      - type: ImageSet &
        description: __OPTIONAL__
        name: oImageSet
    description: __MISSING__
tags:
  - function
owner: gwjensen
---
