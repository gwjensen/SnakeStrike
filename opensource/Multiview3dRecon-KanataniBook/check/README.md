Note: the included sample data ``points.dat`` and ``projmat.dat`` are
dummy files.

(1) Sample multiview data can be obtained from 

       http://www.robots.ox.ac.uk/~vgg/data/data-mview.html .

(2) You need to change the downloaded data formats.

For example, for ``Dinosaur`` data, please convert the 2D tracked
points data format of ``viff.xy``

--------
x1 y1 x2 y2 ...
...
--------

to

--------
x1, y1, x2, y2, ...
...
--------

at each row.

For camera matrices projection ``dino_Ps.mat``, please convert the
MATLAB format to the following text format:

--------
p11 p12 p13 p14
p21 p22 p23 p24
p31 p32 p33 p34
.....
.....
.....
--------

Otherwise, please modify ``fileio.cc`` in this directory to be able to
read the original formats.
