---
brief: Do a simple background subtraction of the current image by using iBackImage as the background.
defined-in-file: "image/SmtImage.h"
layout: method
tags:
  - method
title: BackgroundSubtraction
owner: gwjensen
overloads:
  void BackgroundSubtraction(const SmtImage &):
    signature_with_names: void BackgroundSubtraction(const SmtImage & iBackImage)
    description:
    return: __OPTIONAL__
    arguments:
      - description: __OPTIONAL__
        type: const SmtImage &
        name: iBackImage
  void BackgroundSubtraction(const cv::Mat &):
    description:
    arguments:
      - type: const cv::Mat &
        name: iBackImage
        description: __OPTIONAL__
    return: __OPTIONAL__
    signature_with_names: void BackgroundSubtraction(const cv::Mat & iBackImage)
---
