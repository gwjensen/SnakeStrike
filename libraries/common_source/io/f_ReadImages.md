---
owner: gwjensen
tags:
  - function
title: ReadImages
layout: function
overloads:
  uint64_t ReadImages(const TrackerConfigFile &, ImageSet &, bool (*)()):
    arguments:
      - description: __OPTIONAL__
        name: iConfig
        type: const TrackerConfigFile &
      - name: oTimestepImages
        description: __OPTIONAL__
        type: ImageSet &
      - name: CancelFunc
        type: bool (*)()
        description: __OPTIONAL__
    return: __OPTIONAL__
    description:
    signature_with_names: uint64_t ReadImages(const TrackerConfigFile & iConfig, ImageSet & oTimestepImages, bool (*)() CancelFunc)
  void ReadImages(const std::string & , const TrackerConfigFile &, const uint64_t, const int, const int, const int, std::vector<SmtImage> &, bool (*)()):
    arguments:
      - description: __OPTIONAL__
        name: iFilename
        type: const std::string &
      - description: __OPTIONAL__
        name: iConfig
        type: const TrackerConfigFile &
      - name: iTotalFrames
        description: __OPTIONAL__
        type: const uint64_t
      - name: iImageWidth
        description: __OPTIONAL__
        type: const int
      - name: iImageHeight
        description: __OPTIONAL__
        type: const int
      - name: iCamNum
        description: __OPTIONAL__
        type: const int
      - name: oTimestepImages
        description: __OPTIONAL__
        type: ImageSet &
      - name: CancelFunc
        type: bool (*)()
    description: __OPTIONAL__
  return: __OPTIONAL__
  description:
  signature_with_names: void ReadImages(const std::string& iFilename, const TrackerConfigFile& iConfig, const uint64_t iTotalFrames, const int iImageWidth, const int iImageHeight, const int iCamNum,  std::vector<SmtImage>& oTimestepImages,  bool (*CancelFunc)() )
brief:
defined-in-file: "io/File.h"
---
