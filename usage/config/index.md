---
declaration: "Creating a new project"
title: " Creating a new project"
tags:
  - usage_page
layout: usage_page
---

SnakeStrike relies on a fairly rigid structure of files and folders. Luckily, it creates and manages
these for you.

To start, click on the button in the setup tab labeled "Create Project". Here you select the name for your
project folder and where it will live.

{% include note.html content="Be careful not to create projects inside of projects!" %}



Files that SnakeStrike needs are stored relatively in this directory structure. That means that if you would like
to move the location of a project folder after it has been created, you also need to update the file location information
either in SnakeStrike by clicking the "Edit Project Settings" button, or by manually editing the file proj_config.spt in
the main folder of the project.


Once you have a project created, or opened, and you have cameras attached to the computer that are recognized you can use the
preview functionality to set up the aim of the cameras. There is more description about how to set up the cameras in the section
on [Calibration](https://gwjensen.github.io/SnakeStrike/usage/calibration).

If you are using Basler cameras you should also already have GenApi camera config files created by using PylonViewerApp ( already installed
in the Docker image). This application allows you to fine tune the setups needed for your cameras and generate a file that SnakeStrike knows
how to use.

There are a minimum of two camera configurations that you will need for each project.
1. A slow ( preferably less than 15 Hz ) config file to use while collecting the images needed for camera calibration.
2. The file for the faster camera speed you will use during normal captures.

If you're not using a Basler USB 3 camera, then read information on plugins and how you can incorporate your camera into SnakeStrike.

[Let's start calibrating cameras!](https://gwjensen.github.io/SnakeStrike/usage/calibration)


{% include links.html %}
