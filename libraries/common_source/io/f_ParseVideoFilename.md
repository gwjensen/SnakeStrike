---
title: ParseVideoFilename
layout: function
brief: Parse a passed in string for relevant information from the filename of a video file that was created with CreateVideoFilename.
owner: gwjensen
tags:
  - function
defined-in-file: "io/File.h"
overloads:
  bool ParseVideoFilename(const unsigned int &, const unsigned int &, const unsigned int &, const uint64_t &, const float & , const float & ):
    arguments:
      - name: iFilename
        description: __OPTIONAL__
        type: const std::string &
      - name: oCamNum
        description: __OPTIONAL__
        type: unsigned int &
      - name: oImageWidth
        description: __OPTIONAL__
        type:  unsigned int &
      - name: oImageHeight
        description: __OPTIONAL__
        type: unsigned int &
      - name: oTotalFrames
        description: __OPTIONAL__
        type: uint64_t &
      - name: oOrigCamHz
        description: __OPTIONAL__
        type: float &
      - name: oVideoPlaybackHz
        description: __OPTIONAL__
        type: float &
    description:
    signature_with_names: bool ParseVideoFilename( const std::string& iFilename, unsigned int& oCamNum, unsigned int& oImageWidth, unsigned int& oImageHeight, uint64_t& oTotalFrames, float& oOrigCamHz, float& oVideoPlaybackHz)
    return: True on successful parsing, false otherwise.
---
