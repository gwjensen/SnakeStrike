---
tags:
  - function
owner: gwjensen
overloads:
  void WriteMarkedPointsToFile(const std::string &, const std::vector<std::vector<cv::Point2d>> &, const std::vector<std::vector<std::vector<int32_t>> > &):
    arguments:
      - type: const std::string &
        name: iFilename
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: const std::vector<std::vector<cv::Point2d>> &
        name: iMarkedPoints
      - type: const std::vector<std::vector<std::vector<int32_t>> > &
        description: __OPTIONAL__
        name: iBestFit
    description:
    signature_with_names: void WriteMarkedPointsToFile(const std::string & iFilename, const std::vector<std::vector<cv::Point2d>> & iMarkedPoints, const std::vector<std::vector<std::vector<int32_t>> > & iBestFit)
    return: __OPTIONAL__
title: WriteMarkedPointsToFile
defined-in-file: "io/File.h"
brief: Write the points that the user marked to file for re-use if the user would like to run a slightly different version of triangulation.
layout: function
---
