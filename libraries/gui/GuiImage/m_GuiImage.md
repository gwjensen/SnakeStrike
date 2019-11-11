---
overloads:
  GuiImage(const GuiImage &):
    return: __OPTIONAL__
    description: __MISSING__
    signature_with_names: GuiImage(const GuiImage &)
    arguments:
      - description: __OPTIONAL__
        type: const GuiImage &
        unnamed: true
        name: unnamed-0
  GuiImage(const std::string, const int, const int, const cv::Mat &, const bool, const uint64_t):
    description: __MISSING__
    return: __OPTIONAL__
    signature_with_names: GuiImage(const std::string iLocation, const int iCam, const int iTimestep, const cv::Mat & iImage, const bool iRGB, const uint64_t iTimestamp)
    arguments:
      - description: __OPTIONAL__
        name: iLocation
        type: const std::string
      - description: __OPTIONAL__
        name: iCam
        type: const int
      - name: iTimestep
        type: const int
        description: __OPTIONAL__
      - type: const cv::Mat &
        description: __OPTIONAL__
        name: iImage
      - description: __OPTIONAL__
        name: iRGB
        type: const bool
      - description: __OPTIONAL__
        name: iTimestamp
        type: const uint64_t
  GuiImage(const SmtImage &, const uint64_t):
    arguments:
      - description: __OPTIONAL__
        name: iFrom
        type: const SmtImage &
      - type: const uint64_t
        name: iTimestamp
        description: __OPTIONAL__
    return: __OPTIONAL__
    signature_with_names: GuiImage(const SmtImage & iFrom, const uint64_t iTimestamp)
    description: __MISSING__
  GuiImage(const std::string, const int, const int, const uint64_t):
    arguments:
      - description: __OPTIONAL__
        name: iLocation
        type: const std::string
      - description: __OPTIONAL__
        name: iCam
        type: const int
      - type: const int
        description: __OPTIONAL__
        name: iTimestep
      - name: iTimestamp
        type: const uint64_t
        description: __OPTIONAL__
    description: __MISSING__
    return: __OPTIONAL__
    signature_with_names: GuiImage(const std::string iLocation, const int iCam, const int iTimestep, const uint64_t iTimestamp)
  GuiImage(const SmtImage &, const cv::Mat &, const bool, const uint64_t):
    arguments:
      - description: __OPTIONAL__
        type: const SmtImage &
        name: iFrom
      - description: __OPTIONAL__
        type: const cv::Mat &
        name: iNew
      - type: const bool
        description: __OPTIONAL__
        name: iRGB
      - name: iTimestamp
        description: __OPTIONAL__
        type: const uint64_t
    description: __MISSING__
    signature_with_names: GuiImage(const SmtImage & iFrom, const cv::Mat & iNew, const bool iRGB, const uint64_t iTimestamp)
    return: __OPTIONAL__
layout: method
title: GuiImage
defined-in-file: "projects/gui/GuiImage.h"
tags:
  - method
owner: gwjensen
brief:
is_ctor: true
---
