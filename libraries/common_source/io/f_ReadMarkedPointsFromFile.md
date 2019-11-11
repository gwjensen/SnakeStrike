---
title: ReadMarkedPointsFromFile
defined-in-file: "io/File.h"
tags:
  - function
layout: function
overloads:
  void ReadMarkedPointsFromFile(const std::string &, std::vector<std::vector<cv::Point2d>> &, std::vector<std::vector<std::vector<int32_t>> > &):
    arguments:
      - type: const std::string &
        name: iFilename
        description: __OPTIONAL__
      - description: __OPTIONAL__
        name: oMarkedPoints
        type: std::vector<std::vector<cv::Point2d>> &
      - name: oBestFit
        description: __OPTIONAL__
        type: std::vector<std::vector<std::vector<int32_t>> > &
    return: __OPTIONAL__
    description:
    signature_with_names: void ReadMarkedPointsFromFile(const std::string & iFilename, std::vector<std::vector<cv::Point2d>> & oMarkedPoints, std::vector<std::vector<std::vector<int32_t>> > & oBestFit)
owner: gwjensen
brief: Marked points are user input that tells us which clusters across camera images are the same for a given timestep.
---
