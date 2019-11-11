---
layout: help
title: Frequently Asked Questions
owner: gwjensen
tags:
  - help
library-type: help
---
<div class="panel-group" id="accordion">
    <div class="panel panel-default">
        <div class="panel-heading">
            <h4 class="panel-title">
                <a class="noCrossRef accordion-toggle" data-toggle="collapse" data-parent="#accordion" href="#collapseThree">Why was SnakeStrike developed?</a>
            </h4>
        </div>
        <div id="collapseThree" class="panel-collapse collapse noCrossRef">
            <div class="panel-body">
            SnakeStrike was created to solve 3 main problems:<br>
            1. Tracking systems on the market are either too slow to capture the predatory strike of a snake, or they are insanely expensive.<br>
            2. 3D infrared markers don't stay on a snake very long as they will squirm and remove the markers.<br>
            3. Tracking systems on the marker don't let you keep the original images if you are doing a capture. You either get the tracking points
            or you get the actual images. This is because the cameras have special hardware to do processing on camera.
            </div>
        </div>
    </div>
    <!-- /.panel -->
    <div class="panel panel-default">
        <div class="panel-heading">
            <h4 class="panel-title">
                <a class="noCrossRef accordion-toggle" data-toggle="collapse" data-parent="#accordion" href="#collapseTwo">Is this software free to use?</a>
            </h4>
        </div>
        <div id="collapseTwo" class="panel-collapse collapse noCrossRef">
            <div class="panel-body">
                SnakeStrike is covered by a GPL v3 license.<br><br><br>


                _TL;DR_ Here's what the license entails:

                ```markdown
                1. Anyone can copy, modify and distribute this software.
                2. You have to include the license and copyright notice with each and every distribution.
                3. You can use this software privately.
                4. You can use this software for commercial purposes.
                5. If you dare build your business solely from this code, you risk open-sourcing the whole code base.
                6. If you modify it, you have to indicate changes made to the code.
                7. Any modifications of this code base MUST be distributed with the same license, GPLv3.
                8. This software is provided without warranty.
                9. The software author or license can not be held liable for any damages inflicted by the software.
                ```

                More information on about the [LICENSE can be found here](http://choosealicense.com/licenses/gpl-3.0/)
                All of the other licenses for the open source code used in SnakeStrike can be found in their respective directory in the 'opensource' folder of the SnakeStrike source.<br>
                If you use SnakeStrike in your research please cite it the following way:<br><br>

                 TO BE ADDED....
            </div>
        </div>
    </div>
    <!-- /.panel -->
    <div class="panel panel-default">
        <div class="panel-heading">
            <h4 class="panel-title">
                <a class="noCrossRef accordion-toggle" data-toggle="collapse" data-parent="#accordion" href="#collapseOne">How does this software compare to DeepLabCut</a>
            </h4>
        </div>
        <div id="collapseOne" class="panel-collapse collapse noCrossRef">
            <div class="panel-body">
                Although SnakeStrike and DeepLabCut are both concerned with the tracking of points on an object/subject, they
                differ from each other in very major ways. DeepLabCut excels at mono-camera markerless tracking with subjects/objects
                that have well differentiated body structure. We have yet to see it work with an animal similar to a snake. 3d positioning
                using DeepLabCut is also possible, but the main advantage of DeepLabCut is the marker position information.
                <br>
                SnakeStrike was developed in order to unify reasonably priced commodity cameras for high-speed image capture. It uses markers
                because they are easiest to track from multiple camera angles especially on a difficult to track animal such as a snake. 3d markers
                do not work well with snakes as they are easily removed, thus 2d color markers were more appropriate.
                <br>
                There is no reason that one couldn't use SnakeStrike to collect the images, then DeepLabCut to get position markers, they would just need
                to create a plugin to work with the SnakeStrike program framework. As with every tracking problem, the real issues in quality of the output
                will be related to how noisy the marker/markerless position points are presented, and what to do about occlusions.
            </div>
        </div>
    </div>
    <!-- /.panel -->
    <div class="panel panel-default">
        <div class="panel-heading">
            <h4 class="panel-title">
                <a class="noCrossRef accordion-toggle" data-toggle="collapse" data-parent="#accordion" href="#collapseFour">I moved the project folder location for a project that I've already calibrated, but SnakeStrike says the calibration is missing.</a>
            </h4>
        </div>
        <div id="collapseFour" class="panel-collapse collapse noCrossRef">
            <div class="panel-body">
                This error occurs because SnakeStrike can't find the calibration file that it expects to find. <br>
                I would bet that after moving the project folder to a new location, the proj_config.spt file was not modified to reflect the new information.<br>
                You can update this information by clicking "Edit Project Settings" in the Setup tab.
            </div>
        </div>
    </div>
    <!-- /.panel -->
</div>
<!-- /.panel-group -->

{% include links.html %}
