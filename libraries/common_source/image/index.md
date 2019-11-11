---
library-type: sourcefile
tags:
  - library
layout: library
brief: Classes and functions that deal with images, pixels, and groups of both.
owner: gwjensen
title: "image"
typedefs:
  PixelSet:
    description: For each timestep, for each camera, we have a list of pixels found in the image.
    definition: std::vector<std::vector<std::vector<SmtPixel>>>
  ImageSet:
    description: For each timestep, for each camera, we have an image.
    definition: std::vector<std::vector<SmtImage>>
  PixelClusterSet:
    description: For each timestep, for each camera, we have a list of PixelClusters.
    definition: std::vector<std::vector<std::vector<PixelCluster>>>
---
