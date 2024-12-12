# StereoVision Simulation Code

## 1. Generate camera instances.
   The main camera parameters are focal length, optical point projected at the center of image plane, those parameters can be descriped 
   by internal matrix, and where the camera located in real world space, such as where the camera facing at, those features can be represented 
   by another matrix, which is external matrix.
## 2. Rendering those generated camera.
   The next step is to render those generated camera, the purpose is to visualize the whole process of stereovision simulation directly.
## 3. Calculate disparity image.
   Two images of some scenery, captured by left and right virtual cameras, to generate a disparity image. Then the depth infomation 
   can also be calculated as there is specific relation between disparity and depth.
   between depth and disparity image.
## 4. Calculate the depth image.
   Depth information can be predicated by disparity image from left and right cameras.
	
# Dependency libraries
## 1. VTK
## 2. EIGEN
## 3. Opencv

## The interface of StereoVision Simulation system
![Image](https://github.com/suntaonov24/StereoVisionSimulate/blob/master/Images/image1.png)

## The snapshot images of left and right cameras
![Image](https://github.com/suntaonov24/StereoVisionSimulate/blob/master/Images/image2.png)

## The snapshot of depth image
![Image](https://github.com/suntaonov24/StereoVisionSimulate/blob/master/Images/image3.png)

## Overlapping the captured scenery and it's corresponding 3D depth point cloud data
![Image](https://github.com/suntaonov24/StereoVisionSimulate/blob/master/Images/image4.png)

# Run the compiled binary files
## It is easy to run the code in binary folder, just open the comand or powellshell interface in windows system, and type the folling command.
 .\StereoVision.exe C:\practice\CameraModel\data\Amsterdam.obj 50 0 0 45 0 0 10 10 10 10
 params: 1. 3D dataset, 2. position of left camera: x coordinate, 3. y coordinate, 4. z coordinate, 5. position of right camera: x coordinate,6. y coordinate, 7. z coordinate,
8. physical size of left camera: width, 9. height, 10. physical size of right camera: width, 11. height.

