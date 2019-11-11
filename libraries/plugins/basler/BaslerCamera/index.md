---
layout: class
defined-in-file: "basler/BaslerCamera.h"
declaration: "\nclass BaslerCamera;"
brief: API Interface implementation of a GuiCamera for Basler USB cameras. This follows the interface device by GuiCamera.h.
owner: gwjensen
tags:
  - class
fields:
  mNodeMap:
    annotation:
      - private
    description:
    type: GenApi::INodeMap *
  staticMetaObject:
    type: const QMetaObject
    description:
  mPoolTasks:
    type: std::vector<std::shared_ptr<GrabAndSaveImageTask>>
    annotation:
      - private
    description:
  mCamera:
    annotation:
      - private
    type: Camera_t *
    description:
  mTriggerSource:
    description:
    annotation:
      - private
    type: Basler_UsbCameraParams::TriggerSourceEnums
  mpGrabResult:
    description:
    type: GrabResultPtr_t
    annotation:
      - private
  mTriggerConfig:
    description:
    annotation:
      - private
    type: Pylon::CHardwareTriggerConfiguration *
title: BaslerCamera
---
