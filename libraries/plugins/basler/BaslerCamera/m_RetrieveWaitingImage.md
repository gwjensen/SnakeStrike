---
layout: method
tags:
  - method
overloads:
  GuiImage RetrieveWaitingImage(std::string &):
    signature_with_names: GuiImage RetrieveWaitingImage(std::string & oError)
    return: __OPTIONAL__
    arguments:
      - name: oError
        type: std::string &
        description: __OPTIONAL__
    description: __MISSING__
brief: Retrieve an image that is currently in the image buffer. Or, if not currently there, will be there in less than the value for the Retrieval Timeout.
title: RetrieveWaitingImage
defined-in-file: "basler/BaslerCamera.h"
owner: gwjensen
---
