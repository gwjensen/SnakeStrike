---
owner: gwjensen
brief: Print the current stacktrace. This is very helpful when debugging without a debugger. This function gets called by signal handling code.
overloads:
  static void print_stacktrace(FILE *, unsigned int):
    description: __MISSING__
    signature_with_names: static void print_stacktrace(FILE * out, unsigned int max_frames)
    return: __OPTIONAL__
    arguments:
      - type: FILE *
        description: __OPTIONAL__
        name: out
      - description: __OPTIONAL__
        name: max_frames
        type: unsigned int
layout: function
title: print_stacktrace
tags:
  - function
defined-in-file: ""
---
