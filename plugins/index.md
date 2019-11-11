---
layout: usage
title: Plugin Documentation
owner: gwjensen
brief:
library-type: usage
tags:
  - usage_page
layout: usage_page
---

If you would like to use a non-Basler camera with SnakeStrike or if you want to use a different correspondence algorithm for assigning points across cameras, you need to write a plugin.

The best place to start looking into this is to go to the code source and look in the projects/plugins directory. Here you will find a single instantitation of each of the currently available
interfaces. Copy the object and function parameters from the files in the basler folder to create a new camera plugin. Likewise, copy the functions and parameters from the correspondence folder
if you would like to use a different method of assigning corresponding points to each other across camera images.

The function signatures need to stay the same as SnakeStrike dynamically loads these plugins and expects them to conform to the interface it knows.


