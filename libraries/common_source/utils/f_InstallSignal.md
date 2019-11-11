---
layout: function
title: InstallSignal
overloads:
  void InstallSignal(int):
    signature_with_names: void InstallSignal(int __sig)
    description:
    arguments:
      - description: __OPTIONAL__
        name: __sig
        type: int
    return: __OPTIONAL__
tags:
  - function
brief: This function takes the signal value passed in and registers it with the CritErrHdlr callback so that CritErrHdlr will be called if a signal matching that value is caught. This is mainly used for catching things like SIGSEGV or SIGABRT signals.
defined-in-file: "utils/ErrorHandling.h"
owner: gwjensen
---
