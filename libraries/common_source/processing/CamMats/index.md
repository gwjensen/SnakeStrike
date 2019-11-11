---
owner: gwjensen
layout: class
tags:
  - class
fields:
  mKH:
    type: std::vector<cv::Mat_<double>>
    description: __MISSING__
    annotation:
      - private
  mC:
    description: __MISSING__
    type: std::vector<cv::Point3d>
    annotation:
      - private
  mpsInstance:
    type: CamMats *
    annotation:
      - private
    description: __MISSING__
  mIsInit:
    type: bool
    description: __MISSING__
    annotation:
      - private
  mTInv:
    annotation:
      - private
    description: __MISSING__
    type: std::vector<cv::Mat_<double>>
  mK:
    type: std::vector<cv::Mat_<double>>
    description: __MISSING__
    annotation:
      - private
  mRInv:
    type: std::vector<cv::Mat_<double>>
    description: __MISSING__
    annotation:
      - private
  mDistortion:
    type: std::vector<cv::Mat_<double>>
    annotation:
      - private
    description: __MISSING__
  mMInv:
    annotation:
      - private
    description: __MISSING__
    type: std::vector<cv::Mat_<double>>
  mKInvH:
    description: __MISSING__
    annotation:
      - private
    type: std::vector<cv::Mat_<double>>
  mR:
    annotation:
      - private
    description: __MISSING__
    type: std::vector<cv::Mat_<double>>
  mT:
    description: __MISSING__
    type: std::vector<cv::Mat_<double>>
    annotation:
      - private
  mEigenM:
    description: __MISSING__
    annotation:
      - private
    type: std::vector<Eigen::Matrix<double, 3, 4>>
  mM:
    description: __MISSING__
    annotation:
      - private
    type: std::vector<cv::Mat_<double>>
  mExtrinsic:
    description: __MISSING__
    annotation:
      - private
    type: std::vector<cv::Mat_<double>>
  mPose:
    annotation:
      - private
    description: __MISSING__
    type: std::vector<cv::Mat_<double>>
title: CamMats
defined-in-file: "processing/CamMats.h"
dtor: unspecified
declaration: "\nclass CamMats;"
brief: This Class holds all of the information regarding camera matrices and transforms of them.
---
