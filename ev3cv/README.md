About
-----

ev3cv is a library for building lego robots with stereo machine vision. It analyzes and 
processes information from pairs of 
[NXTcam](http://www.mindsensors.com/index.php?module=pagemaster&PAGE_user_op=view_page&PAGE_id=78) 
sensors. It is a C++ library for use with [ev3dev](http://www.ev3dev.org).

Here is a brief overview of the features provided in the library:

- Stereo vision using dual NXTcam cameras (camera models, calibration).
- ev3dev support library (continuous motor controller, NXTcam support).
- Math support library (linear algebra, quaternions, optimization).
- Command line parsing library.

Stereo vision
=============

The main challenge with using NXTcam for stereo vision is calibration. Stereo 
calibration is the name for the task of learning the intrinsic and extrinsic parameters for 
a pair of stereo cameras. The standard techniques for stereo calibration involve placing a 
pattern convenient for extracting features of a known relative position.

Unfortunately, these techniques are difficult to use with NXTcam, because NXTcam only provides the 
position of objects it detects within its field of view. Therefore, we need to find another
technique to calibrate stereo NXTcam devices. 

The method ev3cv uses is to collect observations of an object subject to a known
constraint. The simplest constraint to realize is that the object must lie on the surface of a
sphere. This constraint can be realized by attaching a string to the object, and the other end of 
the string to a known point relative to the cameras. By keeping the string taught, observations of
the object lying on a sphere can be collected and used for calibration.

