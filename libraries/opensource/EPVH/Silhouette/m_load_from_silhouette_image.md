---
defined-in-file: ""
title: load_from_silhouette_image
overloads:
  bool load_from_silhouette_image(cv::Mat &, std::string, cv::Vec3b, cv::Vec3b):
    arguments:
      - type: cv::Mat &
        name: image
        description: __OPTIONAL__
      - name: correspoding_image_path
        type: std::string
        description: __OPTIONAL__
      - type: cv::Vec3b
        description: __OPTIONAL__
        name: foreground_color
      - name: back_ground_color
        type: cv::Vec3b
        description: __OPTIONAL__
    description: __MISSING__
    signature_with_names: bool load_from_silhouette_image(cv::Mat & image, std::string correspoding_image_path, cv::Vec3b foreground_color, cv::Vec3b back_ground_color)
    return: __OPTIONAL__
  bool load_from_silhouette_image(cv::Mat &, std::string, uchar, uchar):
    return: __OPTIONAL__
    signature_with_names: bool load_from_silhouette_image(cv::Mat & image, std::string correspoding_image_path, uchar foreground_color, uchar back_ground_color)
    arguments:
      - type: cv::Mat &
        description: __OPTIONAL__
        name: image
      - description: __OPTIONAL__
        name: correspoding_image_path
        type: std::string
      - type: uchar
        name: foreground_color
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: uchar
        name: back_ground_color
    description: __MISSING__
owner: gwjensen
layout: method
brief: __MISSING__
tags:
  - method
---
