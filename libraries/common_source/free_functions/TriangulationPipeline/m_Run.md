---
layout: method
overloads:
  int Run(const TrackerConfigFile &):
    signature_with_names: int Run(const TrackerConfigFile & iConfig)
    description: Use the supplied user defined settings file and run the pipeline.
    arguments:
      - type: const TrackerConfigFile &
        name: iConfig
        description: __OPTIONAL__
    return: __OPTIONAL__
  int Run(const std::string &):
    arguments:
      - name: iConfigFile
        type: const std::string &
        description: __OPTIONAL__
    description: Read in the config file from the filesystem and run the pipeline.
    signature_with_names: int Run(const std::string & iConfigFile)
    return: __OPTIONAL__
defined-in-file: "TriangulationPipeline.h"
owner: gwjensen
tags:
  - method
title: Run
brief: Run the full pipeline from start to finish.
---
