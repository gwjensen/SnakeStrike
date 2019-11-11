---
defined-in-file: "image/SmtImage.h"
layout: method
owner: gwjensen
tags:
  - method
overloads:
  SmtImage & operator=(const SmtImage &):
    return: __OPTIONAL__
    signature_with_names: SmtImage & operator=(const SmtImage & other)
    arguments:
      - type: const SmtImage &
        description: __OPTIONAL__
        name: other
    description:
  SmtImage & operator=(const cv::Mat &):
    signature_with_names: SmtImage & operator=(const cv::Mat & other)
    annotation:
      - private
    description:
    return: __OPTIONAL__
    arguments:
      - name: other
        description: __OPTIONAL__
        type: const cv::Mat &
title: operator=
brief:
---
