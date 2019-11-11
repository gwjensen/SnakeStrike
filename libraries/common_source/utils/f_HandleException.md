---
layout: function
overloads:
  void HandleException(std::exception_ptr):
    return: __OPTIONAL__
    signature_with_names: void HandleException(std::exception_ptr eptr)
    arguments:
      - name: eptr
        type: std::exception_ptr
        description: __OPTIONAL__
    description:
tags:
  - function
defined-in-file: "utils/ErrorHandling.h"
title: HandleException
brief: A generic exception handler for catching exceptions that may occur from the interaction with libraries that are not part of the project.
owner: gwjensen
---
