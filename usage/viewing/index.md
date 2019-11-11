---
declaration: "Viewing Triangulated Points"
title: Viewing Triangulated Points
tags:
  - usage_page
layout: usage_page
---


## **Viewing Points**

![alt text](https://gwjensen.github.io/SnakeStrike/images/site/VisualizeScreen_updated.png )

The 3D viewer always you to view the points from a triangulation from any angle you wish.

It plays a slow-motion replay of the 3D points through time. You can speed up or slow down this by changing
the value in the Playback Controls.

### **Trace Trajectory**
These dials allow one to have the markers leave behind lines connecting their previous timesteps. This can
be useful for seeing the trajectory over time. Though, it is very expensive and slows down the visualization.
The number of points is always taken from the first point clicked in the point correspondence done by the user.

For example, I said I liked to click the first point of correspondence when doing snake strikes as the head of the
snake, and the subsequent points as the points that follow along the body as we head towards the tail.
This means that if I do trace trajectory with a single point, then a line will start to form showing the trajectory that
the head took through time. With two points, it will be the head marker and the first marker behind the head, and so on.

### **Render Offset**
This option puts the triangulated points near the origin for easier viewing. The origin is actually one of the cameras
in the setup.

## **Changing Viewpoint**

1. Mouse wheel - zooms in and out
2. Click and drag - moves the clicked point around the axis
3. Ctrl + Click and drag - moves the clicked point around a single axis
4. Shift + Click and drag - shifts the view relative to the axis

These are all kinda hard to describe, best to just try them out.
