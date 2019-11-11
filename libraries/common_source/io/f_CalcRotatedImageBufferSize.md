---
title: CalcRotatedImageBufferSize
owner: gwjensen
overloads:
  double CalcRotatedImageBufferSize(const cv::Mat &):
    signature_with_names: double CalcRotatedImageBufferSize(const cv::Mat & iImage)
    description:
    arguments:
      - type: const cv::Mat &
        name: iImage
        description: __OPTIONAL__
    return: __OPTIONAL__
defined-in-file: "io/File.h"
layout: function
brief: Calculate the size of a buffer that would be needed to hold an image that is rotated. For example, if I rotate a 100x100 image by 45 degrees, it will no longer fit in a buffer allocated for 100x100 because now the corners would be outside of the original view.
tags:
  - function
---
