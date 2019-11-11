---
brief:
is_ctor: true
owner: gwjensen
tags:
  - method
overloads:
  BaslerCamInfo(BaslerCamInfo &&):
    arguments:
      - name: unnamed-0
        type: BaslerCamInfo &&
        description: __OPTIONAL__
        unnamed: true
    signature_with_names: BaslerCamInfo(BaslerCamInfo &&)
    return: __OPTIONAL__
    description:
  BaslerCamInfo(const BaslerCamInfo &):
    signature_with_names: BaslerCamInfo(const BaslerCamInfo &)
    description:
    return: __OPTIONAL__
    arguments:
      - type: const BaslerCamInfo &
        unnamed: true
        name: unnamed-0
        description: __OPTIONAL__
  BaslerCamInfo(const Pylon::CDeviceInfo &):
    return: __OPTIONAL__
    arguments:
      - description: __OPTIONAL__
        name: iInfo
        type: const Pylon::CDeviceInfo &
    description:
    signature_with_names: BaslerCamInfo(const Pylon::CDeviceInfo & iInfo)
layout: method
title: BaslerCamInfo
defined-in-file: "basler/BaslerCamInfo.h"
---
