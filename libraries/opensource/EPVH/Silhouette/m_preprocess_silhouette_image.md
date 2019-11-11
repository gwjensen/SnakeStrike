---
tags:
  - method
defined-in-file: ""
title: preprocess_silhouette_image
overloads:
  void preprocess_silhouette_image(cv::Mat &, uchar, uchar):
    arguments:
      - type: cv::Mat &
        description: __OPTIONAL__
        name: silhoutte_image
      - name: foreground_color
        description: __OPTIONAL__
        type: uchar
      - name: back_ground_color
        description: __OPTIONAL__
        type: uchar
    annotation:
      - protected
    description: __MISSING__
    return: __OPTIONAL__
    signature_with_names: void preprocess_silhouette_image(cv::Mat & silhoutte_image, uchar foreground_color, uchar back_ground_color)
  void preprocess_silhouette_image(cv::Mat &, cv::Vec3b, cv::Vec3b):
    description: __MISSING__
    return: __OPTIONAL__
    signature_with_names: void preprocess_silhouette_image(cv::Mat & silhoutte_image, cv::Vec3b foreground_color, cv::Vec3b back_ground_color)
    arguments:
      - name: silhoutte_image
        description: __OPTIONAL__
        type: cv::Mat &
      - description: __OPTIONAL__
        type: cv::Vec3b
        name: foreground_color
      - description: __OPTIONAL__
        type: cv::Vec3b
        name: back_ground_color
    annotation:
      - protected
brief: __MISSING__
layout: method
owner: gwjensen
---
