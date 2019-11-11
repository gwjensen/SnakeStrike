---
tags:
  - function
brief: Take the 3d triangulated points for each timestep and write them to a file.
overloads:
  void WriteTriangulationInfoToFile(cv::FileStorage &, const std::string &, std::vector<std::vector<cv::Point3d>> &, const int):
    signature_with_names: void WriteTriangulationInfoToFile(cv::FileStorage & iFs, const std::string & iName, std::vector<std::vector<cv::Point3d>> & iList, const int iMaxNumPoints)
    arguments:
      - type: cv::FileStorage &
        name: iFs
        description: __OPTIONAL__
      - type: const std::string &
        description: __OPTIONAL__
        name: iName
      - type: std::vector<std::vector<cv::Point3d>> &
        name: iList
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: const int
        name: iMaxNumPoints
    return: __OPTIONAL__
    description:
layout: function
defined-in-file: "io/File.h"
title: WriteTriangulationInfoToFile
owner: gwjensen
---
