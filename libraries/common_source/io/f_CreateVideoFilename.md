---
title: CreateVideoFilename
layout: function
brief: Get a correctly formatted filename for the video file created from a data capture.
owner: gwjensen
tags:
  - function
defined-in-file: "io/File.h"
overloads:
  std::string CreateVideoFilename(const unsigned int &, const unsigned int &, const unsigned int &, const uint64_t &, const float & , const float & ):
    arguments:
      - name: iCamNum
        description: __OPTIONAL__
        type: const unsigned int &
      - name: iImageWidth
        description: __OPTIONAL__
        type:  const unsigned int &
      - name: iImageHeight
        description: __OPTIONAL__
        type: const unsigned int &
      - name: iTotalFrames
        description: __OPTIONAL__
        type: const uint64_t &
      - name: iOrigCamHz
        description: __OPTIONAL__
        type: const float &
      - name: iVideoPlaybackHz
        description: __OPTIONAL__
        type: const float &
    description:
    signature_with_names: std::string CreateVideoFilename( const unsigned int& iCamNum, const unsigned int& iImageWidth, const unsigned int& iImageHeight, const uint64_t& iTotalFrames, const float& iOrigCamHz,  const float& iVideoPlaybackHz)
    return: __OPTIONAL__
---

