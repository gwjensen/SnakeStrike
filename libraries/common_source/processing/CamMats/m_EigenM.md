---
title: EigenM
layout: method
owner: gwjensen
overloads:
  const std::vector<Eigen::Matrix<double, 3, 4>> EigenM() const:
    signature_with_names: const std::vector<Eigen::Matrix<double, 3, 4>> EigenM() const
    return: __OPTIONAL__
    description: __MISSING__
brief: Vector with matrices corresponding to each camera in the setup in corresponding index. Each matrix is a 3x4 projection matrix but stored as an Eigen matrix instead of an opencv type.
defined-in-file: "processing/CamMats.h"
tags:
  - method
---
