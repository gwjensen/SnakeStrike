---
overloads:
  void ViewCorrespondence(const std::vector<std::vector<SmtPixel>> &, std::vector<SmtImage>, const std::string &):
    signature_with_names: void ViewCorrespondence(const std::vector<std::vector<SmtPixel>> & iPixelsForEachCam, std::vector<SmtImage> iCamImages, const std::string & iTitle)
    arguments:
      - name: iPixelsForEachCam
        type: const std::vector<std::vector<SmtPixel>> &
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: iCamImages
        type: std::vector<SmtImage>
      - type: const std::string &
        description: __OPTIONAL__
        name: iTitle
    description: __MISSING__
    return: __OPTIONAL__
  void ViewCorrespondence(const std::vector<SmtPixel> &, const std::vector<SmtPixel> &, SmtImage, SmtImage, const std::string &):
    signature_with_names: void ViewCorrespondence(const std::vector<SmtPixel> & iFixedPixels, const std::vector<SmtPixel> & iPixelForSpecificCam, SmtImage iFixedImage, SmtImage iImageForCam, const std::string & iTitle)
    description: __MISSING__
    return: __OPTIONAL__
    arguments:
      - name: iFixedPixels
        type: const std::vector<SmtPixel> &
        description: __OPTIONAL__
      - type: const std::vector<SmtPixel> &
        name: iPixelForSpecificCam
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: iFixedImage
        type: SmtImage
      - name: iImageForCam
        description: __OPTIONAL__
        type: SmtImage
      - name: iTitle
        type: const std::string &
        description: __OPTIONAL__
defined-in-file: "visualization/opencv_viz.h"
title: ViewCorrespondence
tags:
  - function
brief: Allows the user to view the point correspondence across images. Again, no guarantee this works. It is there for historical reasons.
layout: function
owner: gwjensen
---
