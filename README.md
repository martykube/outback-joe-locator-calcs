# Outback Joe Locator Calculations

This repo has matlab/octave calculations for translating a region of interest on an image into 3D world coordinates.  

The use case is a camera attached to a quadcopter.  After an image is collected from the camera, the object classifier runs and identifies a ROI in an image.  These calculations show how to turn the image location into real world 3D coordinates.

These calculations can be used to cross check the ROS code.  

camera_model.m has round trip calculations for 3d -> image -> 3D.

cross_check.m is used to verify the python calculations.  Input values are read from ROS and the calculations are used to verify the target's location on the ground plane.





  


