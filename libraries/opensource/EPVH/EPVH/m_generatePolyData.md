---
brief: __MISSING__
title: generatePolyData
owner: gwjensen
layout: method
tags:
  - method
defined-in-file: ""
overloads:
  void generatePolyData(std::vector<std::vector<cv::Point2f>> &, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &, vtkSmartPointer<vtkPolyData> &, std::vector<std::pair<int, int>> &, std::vector<cv::Vec3f> &):
    annotation:
      - protected
    arguments:
      - type: std::vector<std::vector<cv::Point2f>> &
        description: __OPTIONAL__
        name: contours
      - description: __OPTIONAL__
        name: edges
        type: std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &
      - type: vtkSmartPointer<vtkPolyData> &
        description: __OPTIONAL__
        name: edgePolyData
      - name: selectedPolygonSegments
        description: __OPTIONAL__
        type: std::vector<std::pair<int, int>> &
      - name: selectedSegmentColors
        description: __OPTIONAL__
        type: std::vector<cv::Vec3f> &
    return: __OPTIONAL__
    signature_with_names: void generatePolyData(std::vector<std::vector<cv::Point2f>> & contours, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> & edges, vtkSmartPointer<vtkPolyData> & edgePolyData, std::vector<std::pair<int, int>> & selectedPolygonSegments, std::vector<cv::Vec3f> & selectedSegmentColors)
    description: __MISSING__
  void generatePolyData(std::vector<std::vector<cv::Point2f>> &, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &, vtkSmartPointer<vtkPolyData> &):
    annotation:
      - protected
    arguments:
      - type: std::vector<std::vector<cv::Point2f>> &
        name: contours
        description: __OPTIONAL__
      - type: std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &
        description: __OPTIONAL__
        name: edges
      - name: edgePolyData
        type: vtkSmartPointer<vtkPolyData> &
        description: __OPTIONAL__
    description: __MISSING__
    signature_with_names: void generatePolyData(std::vector<std::vector<cv::Point2f>> & contours, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> & edges, vtkSmartPointer<vtkPolyData> & edgePolyData)
    return: __OPTIONAL__
  void generatePolyData(std::vector<std::vector<tr::Edge>> &, vtkSmartPointer<vtkPolyData> &):
    signature_with_names: void generatePolyData(std::vector<std::vector<tr::Edge>> & edges, vtkSmartPointer<vtkPolyData> & edgePolyData)
    arguments:
      - name: edges
        description: __OPTIONAL__
        type: std::vector<std::vector<tr::Edge>> &
      - type: vtkSmartPointer<vtkPolyData> &
        name: edgePolyData
        description: __OPTIONAL__
    description: __MISSING__
    return: __OPTIONAL__
    annotation:
      - protected
---
