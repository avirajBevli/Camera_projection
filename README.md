# Camera Projection (Mapping the 3d points in the world to points in the image)
* Project 3D points in the world to the image, thereby predicting where each point in the world would lie in the actual image taken by the camera.
* Imagine we have a robot moving in on the plane and rotating simulataneously. Our task is to predict the image at any point of time given that we know the exact location of the robot and all points in the world
 
### Setup :
* We have two walls, one along y=0, the other along y=10m
* Given the 3d coordinates of the door corners, our goal is to detect where those corners will project to in the image and draw the boundries of the corresponding doors in the image
* The image of the doors in the image will keep on changing as the position and orientation of the robot change

##### Note about assumptions used : 
* I have assumed K(intrinsic camera matrix) to be same as the matrix of my webcamera(obtained by camera calibration)
* The world coordinate system I have assumed has been shown in the image "world.jpg"
* Also, assumed that the camera lies 5m above the robot's base and look in the same direction as the robot's base frame.
* Also, each door's width and height has been taken to be the same
