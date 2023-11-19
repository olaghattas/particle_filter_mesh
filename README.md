# particle_filter

To run Particle Filter, you will need to make sure you have all the following running.
1) intrinsic parameters. Ensure that the camera in publishing a cameraInfo topic that contains K matrix (intrinsic parameters)
  Real_life: get camera info from cv2.

2) the script requires that you publish the transformation of the cameras to the world coordinate frame to /tf topic. Add the names to the list with the location as the last word in the string, for example, "camra_kitchen", cause the last word is used as the name of te camera and whenever we get an observation we need the name of the camera in order to get its extrinsic parameters so make sure that every cam has a unique name and consistently use it throughout the script.
   In unity, you need to add the camera to TFbridge script.
   on the real robot add them to the broadcast.cpp script.

4) 
