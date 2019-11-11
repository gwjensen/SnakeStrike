---
declaration: "Recording Images"
title: Recording Images
tags:
  - usage_page
layout: usage_page
---

## **Triggers**

When recording there are two types of triggers that can be selected.

### **Software Trigger**
Assuming it is supported by the camera that is being used, a software trigger is sent by the program to all cameras.
Typically, this signal is not synchronous for all cameras. This means that each camera will be started at a different
timestep from each other. Even with low camera capture rates, the cameras can be out of sync because the software
trigger is dependent on the computational load on the computer at the time as well as how the process is scheduled by
the kernel.

Software triggers are useful when testing out setups for lighting or practicing captures. They are used for the
preview function of SnakeStrike as well. However, it is not recommended to use them if one is recording at a high
frame rate with multiple cameras as the cameras can easily differ by hundreds of frames.

### **Hardware Trigger**
This trigger is done by an external device and it is just a TTL pulse that can be used to start all of the cameras
synchronously. As this is dependent on the hardware one has available, there are no specifics that can be provided here. The only
thing to mention is that SnakeStrike expects the triggering of the TTL pulse to happen via a python script. The script called
is easy to change in the project settings. If the TTL device uses an external plunger or other manual device, then a dummy script
can be provided. In order to trigger the capture, the plunger would need to be manually pressed and one of the two options for triggering
in the dialog will need to be used. If a countdown in the dialog is used, the system will countdown and then start waiting to grab
incoming images. The system will wait until all requested images have been captured. Similarly with the key press start, the system will
immediately start to wait on images and continue to wait until all images have been collected.

In my setups I use a homemade TTL pulse device using a U3 DAQ from LabJack. It has a python API which allows me to integrate
it into SnakeStrike very easily.

### **Homemade TTL**

If you want a solution to the TTL problem that works well with the SnakeStrike system, you can create
the following by buying a U3 DAQ from LabJack. The 3D printed case is also nice, but not a strict necessity
for functionality.

![alt text](https://gwjensen.github.io/SnakeStrike/images/site/P1000218.JPG)

![alt text](https://gwjensen.github.io/SnakeStrike/images/site/P1000219.JPG)

![alt text](https://gwjensen.github.io/SnakeStrike/images/site/P1000221.JPG)

![alt text](https://gwjensen.github.io/SnakeStrike/images/site/P1000222.JPG)

You can get the .stl [3D print files](https://gwjensen.github.io/SnakeStrike/images/3d/TriggerBoxFiles.tar.gz). It is modified from an original file by FB Aka Heartman/Hearty from 2016.

## **Saving video or single images**
The default action for collecting images during a capture is the save them as a video with a slower frame
rate than with which the original frames of the video were taken. This is done for two main reasons. First,
the action of the object/subject may be too fast to see well with the naked eye at the original frame rate, so
it makes sense to slow it down for the user to see.

By saving the images in the format of a video, compression techniques that do not dramatically change the content of the images can drastically reduce the final
footprint of the capture. In most cases, using a video format to save the images can reduce the footprint of
the final capture by a factor of ~5 times. This makes a lot of sense when 5 cameras running at 750Hz using a resolution of 640 x 480
pixels for 10 seconds would generate 37500 images files that consume ~21 GBs of space.


## **Directory cleanup**
When cancelled, SnakeStrike doesn't delete the files that may have been created because
it assumes that there will be another capture shortly thereafter which will overwrite the files anyways. However,
this means that if a capture is aborted and then the recording dialog is exited, the capture folder numbering
will continue the next time the recording dialog is brought up as if the last capture was successful and not aborted.
It isn't until one tries to process the data that it is obvious the data is corrupted. This is usually because not
all cameras will have images for all timesteps, or the xml file that lists the files may be incomplete.




[ Recording is done, lets Triangulate our markers !](https://gwjensen.github.io/SnakeStrike/usage/triangulation/)


{% include links.html %}
