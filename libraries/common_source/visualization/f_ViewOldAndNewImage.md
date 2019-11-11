---
layout: function
defined-in-file: "visualization/opencv_viz.h"
title: ViewOldAndNewImage
owner: gwjensen
brief: View two images side by side. Typically this is done with the thresholded image and the original image for debug purposes.
overloads:
  void ViewOldAndNewImage(const cv::Mat &, const cv::Mat &):
    arguments:
      - type: const cv::Mat &
        name: iImageOld
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: const cv::Mat &
        name: iImageNew
    signature_with_names: void ViewOldAndNewImage(const cv::Mat & iImageOld, const cv::Mat & iImageNew)
    return: __OPTIONAL__
    description:
tags:
  - function
---
