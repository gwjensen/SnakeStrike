---
brief: Convert between the eigen and opencv implemenations of matrices.
overloads:
  "template <typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>\nvoid eigen2cv(const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols> &, cv::Mat &)":
    arguments:
      - type: const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols> &
        name: src
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: cv::Mat &
        name: dst
    description:
    signature_with_names: "template <typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>\nvoid eigen2cv(const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols> & src, cv::Mat & dst)"
    return: __OPTIONAL__
layout: function
defined-in-file: "eigen_cv_conversions.h"
owner: gwjensen
tags:
  - function
title: eigen2cv
---
