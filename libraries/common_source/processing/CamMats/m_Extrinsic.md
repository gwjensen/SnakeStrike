---
owner: gwjensen
title: Extrinsic
brief: Vector with matrices corresponding to each camera in the setup in corresponding index. Each matrix is the extrinsic matrix for that camera. The extrinsic matrix is the inverse of the pose matrix. The camera's extrinsic matrix describes the camera's location in the world, and what direction it's pointing. In other words, it takes world coordinates and transforms them into camera coordinates.
layout: method
tags:
  - method
overloads:
  const std::vector<cv::Mat_<double>> Extrinsic() const:
    return: __OPTIONAL__
    description:
    signature_with_names: const std::vector<cv::Mat_<double>> Extrinsic() const
defined-in-file: "processing/CamMats.h"
---
