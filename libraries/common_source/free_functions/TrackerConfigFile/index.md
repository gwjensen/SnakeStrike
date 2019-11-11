---
brief: Main structure used to pass around information the user has configured for the project.
defined-in-file: "common_types.h"
declaration: "\nstruct TrackerConfigFile;"
layout: class
fields:
  vizUndistortedImages:
    description: Number of undistorted images to visualize in pipeline. Default is zero. If value == -1, then all images shown. This is mainly for debuggin purposes.
    type: int64_t
  hLeftbound:
    type: unsigned int
    description: Left bound for the hue value of the threshold.
  sRightbound:
    type: unsigned int
    description: Right bound for the saturation of the threshold.
  vizCameraPose:
    description: Number of camera pose visualizations to show in pipeline. Default is zero.  If value == -1, then all images shown. This is mainly for debuggin purposes.
    type: int64_t
  vRightbound:
    type: unsigned int
    description: Right bound for the value of the threshold.
  hRightbound:
    type: unsigned int
    description: Right bound for the hue of the threshold.
  triangulationOutput:
    description: Path to store the results from the triangulation.
    type: std::string
  minNumCamsTriangulation:
    type: unsigned int
    description: Minimum number of cameras that need to see all of the possible tracking points to generate a triangulation for that timestep.
  noiseThreshold:
    description: Lower threshold for pixel noise thresholding in images.
    type: unsigned int
  dataFileLocation:
    type: std::string
    description: Path to the data file. This is the xml file with the lists of images for a capture.
  vLeftbound:
    description: Left bound for the value of the threshold.
    type: unsigned int
  tryToUseSavedMarkedPoints:
    description: Should the points that the user marks be saved so that if triangulation is done a second time on this set of images that the user doesn't need to click the corresponding points again?
    type: bool
  vizThresholds:
    description: Number of visualizations to show how the pipeline is thresholding the images. Default is zero.  If value == -1, then all images shown. This is mainly for debuggin purposes.
    type: int64_t
  writeUndistImages:
    type: std::string
    description: Path to a folder in which undistored images should be written after the pipeline undistorts individual camera images according to their intrinsic matrices.
  viz3dTriangulatedPoints:
    description: At the end of triangulation do you want to see a visualization of the triangulated points through time? This is more for debug than anything else. A better way to view the points is to use the triangulation viewer and not this option.
    type: bool
  camIndexesToExclude:
    description: The indexes of the cameras that should not be used to do triangulation in any of the timesteps.
    type: std::set<uint64_t>
  projDir:
    description: The base directory for the current project.
    type: std::string
  allowHiddenPointMarking:
    description: Should the user be allowed to click points on the images for marking initial starting positions that don't correspond with a thresholded set of pixels? In theory this should be possible, but in the actual implementation this isn't working.
    type: bool
  calibFileLocation:
    type: std::string
    description: Path to the image used for calibrating the setup.
  noiseFilterSize:
    description: Size of the filter to use in the pixel noise thresholding.
    type: unsigned int
  maskFileLocation:
    description: Path to the XML file that contains the list of files to be used as a background mask.
    type: std::string
  numCameras:
    type: unsigned int
    description: The number of cameras in the setup.
  sLeftbound:
    description: Left bound for the saturation of the threshold.
    type: unsigned int
  vizPointCorrespondences:
    description: Number of point correspondences to visualize in the pipeline.  Default is zero.  If value == -1, then all images shown. This is mainly for debuggin purposes.
    type: int64_t
  noiseIterations:
    description: Number of iterations to run a filter of size noiseFilterSize over the image to try to get rid of pixel noise.
    type: unsigned int
  maxNumPoints:
    description: The maximum number of points that the cameras in the setup are suppose to see. I.e. how many points are you trying to track in the setup.
    type: unsigned int
  undistortImagesBool:
    description: Should the images be undistorted before further processed in the pipeline.
    type: bool
owner: gwjensen
ctor: unspecified
tags:
  - class
title: TrackerConfigFile
dtor: unspecified
---
