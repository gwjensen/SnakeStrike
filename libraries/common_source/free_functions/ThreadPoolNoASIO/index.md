---
defined-in-file: "ThreadPool.h"
title: ThreadPoolNoASIO
declaration: "\nclass ThreadPoolNoASIO;"
fields:
  mAvailable:
    type: std::size_t
    description:
    annotation:
      - private
  mCondition:
    annotation:
      - private
    description:
    type: boost::condition_variable
  mThreads:
    type: boost::thread_group
    annotation:
      - private
    description:
  mTasks:
    type: std::queue<boost::function<void ()>>
    description:
    annotation:
      - private
  mIsRunning:
    annotation:
      - private
    description:
    type: bool
  mMutex:
    annotation:
      - private
    description:
    type: boost::mutex
brief: Implementation of a ThreadPool in boost without ASIO.
owner: gwjensen
tags:
  - class
layout: class
---
