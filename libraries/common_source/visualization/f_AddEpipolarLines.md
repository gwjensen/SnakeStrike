---
defined-in-file: "visualization/pcl_viz.h"
owner: gwjensen
tags:
  - function
brief: Create the epipolar lines from cameras to points in 3d and view them.
title: AddEpipolarLines
overloads:
  void AddEpipolarLines(const std::vector<std::vector<std::pair<cv::Point3d, cv::Point3d>> > &, const std::vector<int> &, pcl::visualization::PCLVisualizer &, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &, int &):
    signature_with_names: void AddEpipolarLines(const std::vector<std::vector<std::pair<cv::Point3d, cv::Point3d>> > & iRays, const std::vector<int> & iCams, pcl::visualization::PCLVisualizer & ioViewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & ioCloud, int & ioCloudPointIdx)
    arguments:
      - description: __OPTIONAL__
        name: iRays
        type: const std::vector<std::vector<std::pair<cv::Point3d, cv::Point3d>> > &
      - type: const std::vector<int> &
        name: iCams
        description: __OPTIONAL__
      - type: pcl::visualization::PCLVisualizer &
        description: __OPTIONAL__
        name: ioViewer
      - name: ioCloud
        type: pcl::PointCloud<pcl::PointXYZRGB>::Ptr &
        description: __OPTIONAL__
      - name: ioCloudPointIdx
        type: int &
        description: __OPTIONAL__
    description:
    return: __OPTIONAL__
layout: function
---
