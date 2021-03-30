[//]: # (Image References)
 
[image1]: ./images/CAS_Systems.jpg
[image2]: ./210330_output/Object_Detector_2D/img0_ObjectDetection2D.jpg
[image3]: ./210330_output/Lidar_ROI_3D_View/img0_LIDAR_ROI.jpg
[image4]: ./images/Lidar_Cluster.png
[image5]: ./images/Detector_AKAZE.jpg
[image6]: ./images/Extract_descriptor.jpg
[image7]: ./images/Matching_Descriptors.jpg
[image8]: ./images/lidar_ttc.png
[image9]: ./images/camera1_ttc.png
[image10]: ./images/camera2_ttc.png
[image11]: ./images/camera3_ttc.png
[image12]: ./images/Mathing_Detector_FAST_Descriptor_BRIEF.gif
[image13]: ./images/Detector_SIFT.gif




# SFND 3D Object Tracking


## Overview


Throughout the Camera course, we learned perspectives about computer vision + Sensor fusion with [Timo Rehfeld](https://www.linkedin.com/in/timo-rehfeld/de).(Engineer at Mercedes-Benz R&D, Autonomous Driving) professional from [MBRDNA](https://www.mbrdna.com/) (Mercedes-Benz Reasearch & Development North America, Inc) team. and the software approach was teached by [Prof. Dr Andreas Haja](https://de.linkedin.com/in/andreas-haja) (professor and autonomous vehicle expert).

In this project lets Engineering a Collision Detection System, working with the collision detection basics, and estimating the time-to-collision (TTC) with Lidar and Camera.
A collision avoidance system (CAS) is an active safety feature that warns drivers or even triggers the brake in the event of an imminent collision with an object in the path of driving. If a preceding vehicle is present, the CAS continuously estimates the TTC. When the TTC falls below a lower threshold, the CAS can then decide to either warn the driver of the imminent danger or - depending on the system - apply the vehicle brakes autonomously. 

![alt text |width=450px | align="middle"][image1]
resorce: https://www.mes-insights.com/what-is-lidar-and-why-do-self-driving-cars-need-it-a-908214/


## Goals

The goal is dentify the most suitable detector/descriptor combination for TTC (time-to-collision) estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor.
In this project we will implement the schematic below.

The mains taks it is available in [rubric](https://review.udacity.com/#!/rubrics/2550/view) file.





## Main Files 

### Implementation Code:
- [FinalProject_Camera.cpp](src/FinalProject_Camera.cpp) main file.
- [matching2D_Student.cpp](src/matching2D_Student.cpp) support functions to 2D features Matchings file;
- [camFusion_Student.cpp](src/camFusion_Student.cpp) support functions to Camera + Lidar file;.
- [objectDetection2D.cpp](src/objectDetection2D.cpp) support functions to Object detection file;
- [lidarData.cpp](src/lidarData.cpp) support functions to Lidar file;

### Running the code:


1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking m1`.

In order to debug and test the code during the development process, it was created 3 modes to run the main code ,and you need pass an argument as explained below:

* Argument m1. - To run the code with  pre fixed Setup Detector and Descritor ( BRISK + SIFT ).
* Argument m2. - To run the code with  pre fixed Setup Detectorlist (FAST , ORB ) and Descritor list ( BRISK , FREAK ).
* Argument m3 - To run the code with  all Detectorlist (SHITOMASI,HARRIS,FAST,BRISK,ORB, AKAZE, SIFT)  and  Descritor list ( BRISK,BRIEF,ORB,FREAK,AKAZE,SIFT).

The code will generate automatically the folder `output` containing all the information necessary to evaluate all the tasks required to achieve the Project goals.

On the freeze [210330_output](210330_output/) folder you will find all the images results and the .txt files with the report used to write the README file.


## Code Pipeline.

The next image is the schematic block of the code.
The orange rectangle was the Midterm Project from the Camera course. In this [repository](CAMERA_2D_Feature_Matching) has a dedicated study about features 2D matchings with a lot of performances evaluation. 

The blue rectangles is the new challenges that was implemented to achieve the objective that is Calculate the TTC.

<img src="images/course_code_structure.png" width="779" height="414" />


### 1. Load images into ring buffer:

Implemented on the file [FinalProject_Camera.cpp](src/FinalProject_Camera.cpp) from lines 206 to 217.
The image dataset is available [here](images/KITTI/2011_09_26/image_02/data/).

### 2. Detect & classify objects:

Applied deep-learning algorithm named YOLO (“You Only Look Once”) that is able to detect a range of various objects, including vehicles, pedestrians and several others.
Implemented on the file [objectDetection2D.cpp](src/bjectDetection2D.cpp) from lines 24 to 155 and used on the main [code](src/FinalProject_Camera.cpp) at line 226.

![alt text |width=450px | align="middle"][image2]

In this [folder](210330_output/Object_Detector_2D/) you find the results for all image dataset.

### 3. Crop LIDAR points:

Implemented on the file [lidarData.cpp](src/lidarData.cpp) from lines 12 to 27.
and used on the main [code](src/FinalProject_Camera.cpp) at line 241.


### 4. Cluster Lidar point Cloud:

Implemented on the file [camFusion_Student.cpp](src/camFusion_Student.cpp) from lines 21 to 130.And used on the main [code](src/FinalProject_Camera.cpp) at line 252.
The next image show the example of the results.

![alt text |width=250px | align="middle"][image3]  

In this [folder](210330_output/Lidar_ROI_3D_View/) you find the results for all Lidar point cloud dataset.

Interminal it is possible check the partial results.
![alt text |width=250px | align="middle"][image4]

More details is available on file: [ObjectDetc2D_lidar_ROI.txt](./210330_output/ObjectDetc2D_lidar_ROI.txt)


### 5. Detect image keypoints :

Implemented on the file [matching2D_Student.cpp](src/matching2D_Student.cpp) from lines 9 to 251.And used on the main [code](src/FinalProject_Camera.cpp) from lines 277 to 351.

The next image show the example of the results.

![alt text |width=250px | align="middle"][image5]

More details, please check the [repository](CAMERA_2D_Feature_Matching).

### 6. Extract Keypoints Descriptor :

Implemented on the file [matching2D_Student.cpp](src/matching2D_Student.cpp) from lines 254 to 344.And used on the main [code](src/FinalProject_Camera.cpp) from lines 358 to 374.
The next image show the example of the results.

![alt text |width=250px | align="middle"][image6]

This image is an example with the Descriptors Keypoints cropped on the preceding vehicle area. In this project we are not cropping , because we have functions to match the descriptor keypoints to the bounding Boxes extracted in the Object detection function.

More details, please check the [repository](CAMERA_2D_Feature_Matching).

### 7. Match Keypoints Descriptor :

Implemented on the file [matching2D_Student.cpp](src/matching2D_Student.cpp) from lines 347 to 469.And used on the main [code](src/FinalProject_Camera.cpp) from lines 379 to 424.
The next image show the example of the results.

![alt text |width=250px | align="middle"][image7]

This image is an example with the Descriptors Keypoints cropped on the preceding vehicle area. In this project we are not cropping , because we have functions to match the descriptor keypoints to the bounding Boxes extracted in the Object detection function.

More details, please check the [repository](CAMERA_2D_Feature_Matching).


### 8. Track 3D Object Bounding Boxes : ( Task FP.1).

Implemented on the file [camFusion_Student.cpp](src/camFusion_Student.cpp) from lines 393 to 447.And used on the main [code](src/FinalProject_Camera.cpp) at line 436.

The strategy here was iterate through Matches vector extracting the Bounding box ID that enclose the keypoints on the previous and current frame.
It was created a vector named final_result with the same size of current frame Bounding boxes quantity.
So , the ID of the Bounding Boxes previous was storage in the final_result vector on the position of the ID of the Bounding Boxes  current.

To finish the function, it was used a complementary funcion named `FindVectorOccurrences` to count the occurrences of each ID Bounding boxes previous for each Bounding boxes current . The ID that had more occurences was storage in a  map structure with the pair < bBox previous , bBox Current> as the best Bounding boxes mathes.

The `FindVectorOccurrences` function is available in [camFusion_Student.cpp](src/camFusion_Student.cpp) from lines 408 to 430 and was created followins this Stakeoverflow [answer](https://stackoverflow.com/questions/2488941/find-which-numbers-appears-most-in-a-vector/55478213#5547821)

The image below ilustrate the strategy.

![alt text |width=250px | align="middle"][image1]  


### 9. Compute TTC on Object in front : ( Task FP.2 & Task FP.3 & FP.4 ).

#### 9.1 - Compute Lidar-based TTC: (Task FP.2).

Implemented on the file [camFusion_Student.cpp](src/camFusion_Student.cpp) from lines 323 to 390.And used on the main [code](src/FinalProject_Camera.cpp) at line 476.

Working with the dataset already cropped on the `step3`. The strategy here to avoid outliers and increase the reliability was create 3 zones in order to track the x min coordinate value in the center of the Ego car Y=0 , 1m to left and 1m to right. The  setup of zone width is a variable at line 333.With the 3 Xmin values in previous and current frames we have calculated 3 TTC values. and the final result is te mean of those values.

The next image shows schematically the strategy implemented.

![alt text |width=250px | align="middle"][image8] 

#### 9.2 - Associate Keypoint Correspondences with Bounding Boxes (Task FP.3).


Implemented on the file [camFusion_Student.cpp](src/camFusion_Student.cpp) from lines 222 to 264.And used on the main [code](src/FinalProject_Camera.cpp) at line 487.

The strategy here was as Step1, iterate through Matches vector. As 2 step we iterate through the bounding boxes. If the Keypoint is enclosed by the bounding box, we calculate the Euclidean distance between the pair (current and Previous) keypoints, and store the distances in a vector named `DistanceEuclideanCoord`, At the end, we calculate the mean and Standard Deviation from the Euclidean distances vector.

In the next Step ,we repeat the steps 1 and 2 . But considering as acceptable only the pairs where the distance betwwen (current and Previous) keypoints are inside of the statistical acceptable interval.

#### 9.3 - Compute Camera-based TTC (Task FP.4).

Implemented on the file [camFusion_Student.cpp](src/camFusion_Student.cpp) from lines 270 to 318.And used on the main [code](src/FinalProject_Camera.cpp) at line 488.


iterating through Matches vector, for each step we calculate the distance from the current point for all point of the vector individualy, for both current and previous 
, if the distance is greather than the min acceptable (variable at line 285) ,we get a rate from current/previous, At the end of this step we calculate the mean and standar deviatiom , and have filtered the the values inside of the statistical acceptable interval and store the values inside a new vector named `filtered_dist_ratio`. As final step, we sort the vector and used the Median Value as impute to calculate the TTC.

The computational vision techniques used to calculate distances in images are explained with the 2 pictures below.

![alt text |width=250px | align="middle"][image10] 
image source:UDACITY Sensor fusion Nanodegree Class.

Set Equations:

![alt text |width=250px | align="middle"][image11] 
image source:UDACITY Sensor fusion Nanodegree Class.


## Performances.

### FP5 - Performance Evaluation 1.




### FP6 - Performance Evaluation 6.




## Conclusion.
    
The BRIEF descriptor feature shows the best choice for the most combined Detectors also regard matching points between the scenes(MP.8). On the last plot we can see that ,the combined BRISK+BRIEF has the highest number of descriptor points detected on de preceding vehicle and the processing time is something around 50 mili seconds maximum. Depends of the application this combination could be chosen, but we have a group of combinations that are capable to process the descriptor points under the 25 millisecond. If the application request a high frequency to read the data and this quantity of the descriptors keypoints is enough sufficient, the combination of FAST+BRIEF could be chosen. The results of AKAZE+BRIEF and SIFT+BRIEF must be avoided because the elapsed time necessary is to large comparing with others and HARRIS + BRIEF also must be avoided, it has a good performance regard time , but the number of descriptor keypoints processed is very small.  
    
    ## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
  * sudo apt-get install git-lfs
  * git lfs install
  * git clone https://github.com/udacity/SFND_3D_Object_Tracking.git
  * sudo apt-get install libcanberra-gtk-module ( to fix error message)



* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
  * tutorial: https://forums.developer.nvidia.com/t/process-to-install-opencv-4-1-on-nano/75801
   * in cmake step: run the next line in terminal: 
  
  cmake -D WITH_CUDA=OFF \
    	 -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.1.0/modules \
    	-D WITH_GSTREAMER=OFF \
    	-D WITH_LIBV4L=ON \
    	-D BUILD_opencv_python2=OFF \
    	-D BUILD_opencv_python3=OFF \
    	-D BUILD_TESTS=OFF \
    	-D BUILD_PERF_TESTS=OFF \
    	-D BUILD_EXAMPLES=OFF \
     -D OPENCV_ENABLE_NONFREE=ON \
    	-D CMAKE_BUILD_TYPE=RELEASE \
    	-D CMAKE_INSTALL_PREFIX=/usr/local ..
     
          
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
