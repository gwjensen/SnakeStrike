---
overloads:
  void GetPrincipalPointInfo(const int &, double &, cv::Point2d &) const:
    signature_with_names: void GetPrincipalPointInfo(const int & iCamIdx, double & oFocalLength, cv::Point2d & oOpticalCenter) const
    return: __OPTIONAL__
    arguments:
      - description: __OPTIONAL__
        name: iCamIdx
        type: const int &
      - type: double &
        name: oFocalLength
        description: __OPTIONAL__
      - type: cv::Point2d &
        name: oOpticalCenter
        description: __OPTIONAL__
    description:
  void GetPrincipalPointInfo(std::vector<double> &, std::vector<cv::Point2d> &) const:
    signature_with_names: void GetPrincipalPointInfo(std::vector<double> & oFocalLengths, std::vector<cv::Point2d> & oOpticalCenters) const
    return: __OPTIONAL__
    arguments:
      - name: oFocalLengths
        type: std::vector<double> &
        description: __OPTIONAL__
      - type: std::vector<cv::Point2d> &
        description: __OPTIONAL__
        name: oOpticalCenters
    description:
defined-in-file: "processing/CamMats.h"
layout: method
owner: gwjensen
tags:
  - method
title: GetPrincipalPointInfo
brief: These functions return the principle point for the camera. The principle point is the point in image coordinates where the principle ray intersects the image plane. The principle ray is the ray that goes through the pinhole of the camera.
---
