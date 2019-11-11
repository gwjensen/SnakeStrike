---
owner: gwjensen
layout: class
defined-in-file: "ThreadPool.h"
tags:
  - class
title: ThreadPool
declaration: "\nstruct ThreadPool;"
fields:
  mG:
    type: boost::thread_group
    annotation:
      - private
    description: Used to keep track of threads for the join that is used on exit.
  mWorking:
    description: Unique_ptr to the boost::asio::io_service::work object.
    type: asio_worker
    annotation:
      - private
  mService:
    description:
    type: boost::asio::io_service
    annotation:
      - private
brief: Implementation of a ThreadPool using Boost.
---
