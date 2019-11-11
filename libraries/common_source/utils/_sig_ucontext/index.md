---
owner: gwjensen
fields:
  uc_flags:
    type: unsigned long
    description:
  uc_stack:
    description:
    type: stack_t
  uc_mcontext:
    description:
    type: struct sigcontext
  uc_sigmask:
    description:
    type: sigset_t
  uc_link:
    description:
    type: struct ucontext *
title: _sig_ucontext
declaration: "\nstruct _sig_ucontext;"
layout: class
dtor: unspecified
brief: This structure mirrors the one found in /usr/include/asm/ucontext.h. It is needed as part of the signal handling code.
tags:
  - class
ctor: unspecified
defined-in-file: "utils/ErrorHandling.h"
---
