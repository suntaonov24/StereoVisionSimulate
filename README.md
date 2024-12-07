# StereoVision Simulation code

## 1. Generate camera instances;
    The main camera parameters are focal length, optical point position projected on image plane, those parameters can be
	descriped by internal matrix, and where the camera located in real space, such as where the camera facing at, those features
	also can be represented by another matrix, the external matrix.
## 2. Rendering those generated camera;
    The next step is to render those generated camera, the purpose is to visualize the process of simulation directly.
## 3. Calculate disparity image.
## 4. Calculate the depth image.
	
# Dependency libraries
## 1. VTK
## 2. EIGEN
## 3. Opencv
## 4. ITK

# The interface of StereoVision Simulation system
![Image](https://github.com/suntaonov24/StereoVisionSimulate/blob/master/Images/image1.png)

#The snapshot of left and right cameras
![Image](https://github.com/suntaonov24/StereoVisionSimulate/blob/master/Images/image2.png)