---
namespace:
  - tr
title: Silhouette
layout: class
owner: gwjensen
defined-in-file: ""
tags:
  - class
brief: __MISSING__
declaration: "\nclass tr::Silhouette;"
fields:
  m_grab_cut_bb:
    type: cv::Rect
    annotation:
      - private
    description: __MISSING__
  contour_points:
    type: std::vector<cv::Point2f>
    description: __MISSING__
    annotation:
      - private
  height:
    annotation:
      - private
    description: __MISSING__
    type: int
  bounding_image:
    type: cv::Mat
    description: __MISSING__
    annotation:
      - private
  m_foreground_points:
    annotation:
      - private
    description: __MISSING__
    type: std::vector<cv::Point>
  mHierarchy:
    annotation:
      - private
    description: __MISSING__
    type: std::vector<cv::Vec4i>
  compression_ratio:
    description: __MISSING__
    annotation:
      - private
    type: float
  m_is_computed:
    description: __MISSING__
    type: bool
    annotation:
      - private
  image_path:
    annotation:
      - private
    description: __MISSING__
    type: std::string
  fast_access_enabled:
    type: bool
    description: __MISSING__
    annotation:
      - private
  boundary_points:
    annotation:
      - private
    type: std::vector<cv::Point2f>
    description: __MISSING__
  mIsForeground:
    annotation:
      - private
    type: std::vector<bool>
    description: __MISSING__
  width:
    annotation:
      - private
    type: int
    description: __MISSING__
  mHierarchyContours:
    description: __MISSING__
    type: std::vector<std::vector<cv::Point2f>>
    annotation:
      - private
  bb_left_corner:
    annotation:
      - private
    description: __MISSING__
    type: cv::Point2f
  pixel_margin:
    description: __MISSING__
    annotation:
      - private
    type: int
  m_background_points:
    description: __MISSING__
    annotation:
      - private
    type: std::vector<cv::Point>
  m_boundary_contours:
    description: __MISSING__
    annotation:
      - private
    type: std::vector<std::vector<cv::Point2f>>
  m_use_outer_boundary:
    annotation:
      - private
    description: __MISSING__
    type: bool
---
