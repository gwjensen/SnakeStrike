---
title: PopulateImagesFromVideo
layout: function
brief: Pull images from the provided videos.
owner: gwjensen
tags:
  - function
defined-in-file: "io/File.h"
overloads:
void PopulateImagesFromVideos( std::vector< std::string > , const TrackerConfigFile &, ImageSet &, bool (*)() );:
    arguments:
      - name: iVideoFileNames
        description: __OPTIONAL__
        type: std::vector< std::string >
      - name: iConfig
        description: __OPTIONAL__
        type: const TrackerConfigFile &
      - name: oTimestepImages
        description: __OPTIONAL__
        type: ImageSet &
      - name: CancelFunc
        description: __OPTIONAL__
        type: bool (*)()
    description:
    signature_with_names: void PopulateImagesFromVideos( std::vector< std::string > iVideoFileNames, const TrackerConfigFile& iConfig,  ImageSet& oTimestepImages, bool (*CancelFunc)() );
    return: __OPTIONAL__
---
