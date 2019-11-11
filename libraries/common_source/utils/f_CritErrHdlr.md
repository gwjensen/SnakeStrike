---
tags:
  - function
owner: gwjensen
overloads:
  void CritErrHdlr(int, siginfo_t *, void *):
    arguments:
      - type: int
        name: sig_num
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: siginfo_t *
        name: info
      - name: ucontext
        type: void *
        description: __OPTIONAL__
    signature_with_names: void CritErrHdlr(int sig_num, siginfo_t * info, void * ucontext)
    description: __MISSING__
    return: __OPTIONAL__
defined-in-file: "utils/ErrorHandling.h"
brief: This is the callback function for processing signal events. For each installed signal, this function will be called if that signal gets caught. In practice, this just prints the stacktrace, though.
title: CritErrHdlr
layout: function
---
