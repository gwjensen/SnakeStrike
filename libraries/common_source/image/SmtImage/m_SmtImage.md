---
layout: method
tags:
  - method
owner: gwjensen
title: SmtImage
brief:
is_ctor: true
overloads:
  SmtImage(const std::string, const int, const int, const cv::Mat &, const bool):
    description: Loads the image from a file.
    return: __OPTIONAL__
    signature_with_names: SmtImage(const std::string iLocation, const int iCam, const int iTimestep, const cv::Mat & iImage, const bool iRGB)
    arguments:
      - name: iLocation
        description: __OPTIONAL__
        type: const std::string
      - type: const int
        name: iCam
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: iTimestep
        type: const int
      - name: iImage
        type: const cv::Mat &
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: iRGB
        type: const bool
  SmtImage(const SmtImage &, const cv::Mat &, const bool):
    return: __OPTIONAL__
    signature_with_names: SmtImage(const SmtImage & iFrom, const cv::Mat & iNew, const bool iRGB)
    arguments:
      - description: __OPTIONAL__
        type: const SmtImage &
        name: iFrom
      - type: const cv::Mat &
        description: __OPTIONAL__
        name: iNew
      - description: __OPTIONAL__
        name: iRGB
        type: const bool
    description: Copies the attributes from iFrom while assigning the pixels of iNew to the object. iRGB is important as we can't check the encoding of the image after the fact.
  SmtImage(const SmtImage &):
    arguments:
      - type: const SmtImage &
        description: __OPTIONAL__
        name: iFrom
    signature_with_names: SmtImage(const SmtImage & iFrom)
    return: __OPTIONAL__
    description:
  SmtImage(const std::string, const int, const int):
    description: Load image from file.
    arguments:
      - name: iLocation
        type: const std::string
        description: __OPTIONAL__
      - type: const int
        description: __OPTIONAL__
        name: iCam
      - type: const int
        description: __OPTIONAL__
        name: iTimestep
    return: __OPTIONAL__
    signature_with_names: SmtImage(const std::string iLocation, const int iCam, const int iTimestep)
defined-in-file: "image/SmtImage.h"
---
