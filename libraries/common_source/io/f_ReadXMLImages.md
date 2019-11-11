---
layout: function
tags:
  - function
owner: gwjensen
brief: The list of images for a capture are stored in xml files. This function parses the files and returns a list of paths to the image files.
overloads:
  void ReadXMLImages(const std::string &, const std::string &, std::vector<std::string> &, uint32_t &, uint64_t &):
    arguments:
      - name: iLocation
        type: const std::string &
        description: __OPTIONAL__
      - type: const std::string &
        description: __OPTIONAL__
        name: iBaseDir
      - name: oImageLocations
        type: std::vector<std::string> &
        description: __OPTIONAL__
      - type: uint32_t &
        name: oNumCams
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: oNumImagesFound
        type: uint64_t &
    signature_with_names: void ReadXMLImages(const std::string & iLocation, const std::string & iBaseDir, std::vector<std::string> & oImageLocations, uint32_t & oNumCams, uint64_t & oNumImagesFound)
    return: __OPTIONAL__
    description:
defined-in-file: "io/File.h"
title: ReadXMLImages
---
