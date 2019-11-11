---
layout: function
defined-in-file: "image/ImageSet.h"
owner: gwjensen
tags:
  - function
brief:
title: UndistortImages
overloads:
  void UndistortImages(ImageSet &, bool (*)(), const std::pair<unsigned long, std::set<unsigned long>> &, const std::string &):
    return: __OPTIONAL__
    signature_with_names: void UndistortImages(ImageSet & ioImagesSet, bool (*)() CancelFunc, const std::pair<unsigned long, std::set<unsigned long>> & iCamIndexesToExclude, const std::string & iWriteLocation)
    description:
    arguments:
      - name: ioImagesSet
        description: __OPTIONAL__
        type: ImageSet &
      - description: __OPTIONAL__
        name: CancelFunc
        type: bool (*)()
      - description: __OPTIONAL__
        name: iCamIndexesToExclude
        type: const std::pair<unsigned long, std::set<unsigned long>> &
      - name: iWriteLocation
        type: const std::string &
        description: __OPTIONAL__
---
