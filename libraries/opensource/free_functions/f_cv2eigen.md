---
title: cv2eigen
defined-in-file: "eigen_cv_conversions.h"
overloads:
  "template <typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>\nvoid cv2eigen(const cv::Mat &, Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols> &)":
    arguments:
      - description: __OPTIONAL__
        type: const cv::Mat &
        name: src
      - description: __OPTIONAL__
        type: Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols> &
        name: dst
    description:
    signature_with_names: "template <typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>\nvoid cv2eigen(const cv::Mat & src, Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols> & dst)"
    return: __OPTIONAL__
  "template <typename _Tp>\nvoid cv2eigen(const cv::Mat &, Eigen::Matrix<_Tp, 1, Eigen::Dynamic> &)":
    return: __OPTIONAL__
    description:
    signature_with_names: "template <typename _Tp>\nvoid cv2eigen(const cv::Mat & src, Eigen::Matrix<_Tp, 1, Eigen::Dynamic> & dst)"
    arguments:
      - type: const cv::Mat &
        description: __OPTIONAL__
        name: src
      - type: Eigen::Matrix<_Tp, 1, Eigen::Dynamic> &
        description: __OPTIONAL__
        name: dst
  "template <typename _Tp, int _rows, int _cols, int _options>\nvoid cv2eigen(const cv::Mat &, Eigen::Matrix<_Tp, Eigen::Dynamic, Eigen::Dynamic> &)":
    return: __OPTIONAL__
    signature_with_names: "template <typename _Tp, int _rows, int _cols, int _options>\nvoid cv2eigen(const cv::Mat & src, Eigen::Matrix<_Tp, Eigen::Dynamic, Eigen::Dynamic> & dst)"
    description:
    arguments:
      - name: src
        type: const cv::Mat &
        description: __OPTIONAL__
      - type: Eigen::Matrix<_Tp, Eigen::Dynamic, Eigen::Dynamic> &
        description: __OPTIONAL__
        name: dst
  "template <typename _Tp>\nvoid cv2eigen(const cv::Mat &, Eigen::Matrix<_Tp, Eigen::Dynamic, 1> &)":
    arguments:
      - type: const cv::Mat &
        description: __OPTIONAL__
        name: src
      - name: dst
        type: Eigen::Matrix<_Tp, Eigen::Dynamic, 1> &
        description: __OPTIONAL__
    description:
    signature_with_names: "template <typename _Tp>\nvoid cv2eigen(const cv::Mat & src, Eigen::Matrix<_Tp, Eigen::Dynamic, 1> & dst)"
    return: __OPTIONAL__
owner: gwjensen
brief: Convert between the eigen and opencv implemenations of matrices.
layout: function
tags:
  - function
---
