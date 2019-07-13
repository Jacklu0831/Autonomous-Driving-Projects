# Self Driving Car Projects

A number of autonomous vehicle projects I coded along with the [Udacity Self Driving Car Nanodegree program](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013 "Udacity Self Driving Car"). Main topics include computer vision, behavior cloning, deep learning, sensor, localization, and path planning. With some prior knowledge in computer vision and robotics, I came into the nanodegree for the exciting projects it offers and I certainly gained experience in implementing my code for solving this real world problem. 

# Show before Tell

<!-- <table style="width:100%">
  <tr>
    <th>
      <p align="center">
           <a href="https://www.youtube.com/watch?v=KlQ-8iD1EFM"><img src="./project_1_lane_finding_basic/img/overview.gif" alt="Overview" width="60%" height="60%"></a>
           <br>P1: Basic Lane Finding
           <br><a href="./project_1_lane_finding_basic" name="p1_code">(code)</a>
      </p>
    </th>
        <th><p align="center">
           <a href="./project_2_traffic_sign_classifier/Traffic_Sign_Classifier.ipynb"><img src="./project_2_traffic_sign_classifier/img/softmax.png" alt="Overview" width="60%" height="60%"></a>
           <br>P2: Traffic Signs
           <br><a href="./project_2_traffic_sign_classifier" name="p2_code">(code)</a>
        </p>
    </th>
       <th><p align="center">
           <a href="https://www.youtube.com/watch?v=gXkMELjZmCc"><img src="./project_3_behavioral_cloning/img/overview.gif" alt="Overview" width="60%" height="60%"></a>
           <br>P3: Behavioral Cloning
           <br><a href="./project_3_behavioral_cloning" name="p3_code">(code)</a>
        </p>
    </th>
        <th><p align="center">
           <a href="https://www.youtube.com/watch?v=g5BhDtoheE4"><img src="./project_4_advanced_lane_finding/img/overview.gif" alt="Overview" width="60%" height="60%"></a>
           <br>P4: Adv. Lane Finding
           <br><a href="./project_4_advanced_lane_finding" name="p4_code">(code)</a>
        </p>
    </th>
  </tr>
  <tr>
    <th><p align="center">
           <a href="https://www.youtube.com/watch?v=Cd7p5pnP3e0"><img src="./project_5_vehicle_detection/img/overview.gif" alt="Overview" width="60%" height="60%"></a>
           <br>P5: Vehicle Detection
           <br><a href="./project_5_vehicle_detection" name="p5_code">(code)</a>
        </p>
    </th>
        <th><p align="center">
           <a href="./project_6_extended_kalman_filter"><img src="./project_6_extended_kalman_filter/img/overview.jpg" alt="Overview" width="60%" height="60%"></a>
           <br>P6: Ext. Kalman Filter
           <br><a href="./project_6_extended_kalman_filter" name="p6_code">(code)</a>
        </p>
    </th>
    <th><p align="center">
           <a href="./project_7_unscented_kalman_filter"><img src="./project_7_unscented_kalman_filter/img/overview.jpg" alt="Overview" width="60%" height="60%"></a>
           <br>P7: Unsc. Kalman Filter
           <br><a href="./project_7_unscented_kalman_filter" name="p7_code">(code)</a>
        </p>
    </th>
    <th><p align="center">
           <a href="./project_8_kidnapped_vehicle"><img src="./project_8_kidnapped_vehicle/img/overview.gif" alt="Overview" width="60%" height="60%"></a>
           <br>P8: Kidnapped Vehicle
           <br><a href="./project_8_kidnapped_vehicle" name="p8_code">(code)</a>
        </p>
    </th>
  </tr>
  <tr>
    <th><p align="center">
           <a href="https://www.youtube.com/watch?v=w9CETKuJcVM"><img src="./project_9_PID_control/img/overview.gif" alt="Overview" width="60%" height="60%"></a>
           <br>P9: PID Controller
           <br><a href="./project_9_PID_control" name="p9_code">(code)</a>
        </p>
    </th>
    <th><p align="center">
           <a href="./project_10_MPC_control"><img src="./project_10_MPC_control/img/overview.gif" alt="Overview" width="60%" height="60%"></a>
           <br>P10: MPC Controller
           <br><a href="./project_10_MPC_control" name="p10_code">(code)</a>
        </p>
    </th>
   <th><p align="center">
           <a href="./project_11_path_planning"><img src="./project_11_path_planning/img/overview.jpg" alt="Overview" width="60%" height="60%"></a>
           <br>P11: Path Planning
           <br><a href="./project_11_path_planning" name="p11_code">(code)</a>
        </p>
    </th>
    <th><p align="center">
          <a href="./project_12_road_segmentation"><img src="./project_12_road_segmentation/img/overview.jpg" alt="Overview" width="60%" height="60%"></a>
           <br>P12: Road Segmentation
           <br><a href="./project_12_road_segmentation" name="p12_code">(code)</a>
        </p>
    </th>
  </tr>
</table> -->

# Table of Contents

#### [1 - Detecting Lane Lines](1_lane_finding_basic)
 - Built a simplistic lane detection pipeline with **color selection**, **region masking**, **Canny edge detection** and **Hough transforms**.
 - Computer Vision

#### [2 - Advanced Lane Finding](2_advanced_lane_finding)
 - Built an advanced lane-finding algorithm using **distortion correction**, **image rectification**, **color transforms**, **view warping**, **sliding window**, **gradient thresholding**, and **curvature measure**. Colored the detected curved lane area in green. Calculated lane curvature and vehicle displacement. Overcame environmental challenges such as shadows and pavement changes.
 - Computer Vision, OpenCV
 
#### [3 - Traffic Sign Classification](3_traffic_sign_classifier)
 - Built, trained, and tuned a **convolutional neural network** to classify traffic signs. Tried out a different neural network architectures. Performed **image augmentation** to combat overfitting.
 - Deep Learning, TensorFlow, Keras, Computer Vision
 
#### [4 - Behavioral Cloning](4_behavioral_cloning)
 - Built, trained. and tuned a **deep convolutional neural network** for **end-to-end driving** in Udacity simulator with limited data. Used techniques such as self created image augmentation generator with the **ImgAug library** and dropout layers to combat overfitting. 
 - Deep Learning, Keras, Convolutional Neural Networks, NVIDIA CNN, ImgAug

#### [5 - Extended Kalman Filter](5_extended_kalman_filter)
 - Implement the **extended Kalman filter** in C++. Simulated noisy lidar and noisy radar measurements were used to predict where the vehicle in simulator would be in each timestamp. **Kalman filter** was used to perform **sensor fusion** on the lidar measurements and radar measurements to track the vehicle's position and velocity.
 - C++, Kalman Filter, Extended Kalman Filter
 
<!-- #### [8 - Kidnapped Vehicle](8_kidnapped_vehicle)
 - Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data. In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.
 - C++, Particle Filter
 
#### [9 - PID Control](9_PID_control)
 - Implement a PID controller for keeping the car on track by appropriately adjusting the steering angle.
 - C++, PID Controller
 
#### [10 - MPC Control](10_MPC_control)
 - Implement an MPC controller for keeping the car on track by appropriately adjusting the steering angle. Differently from previously implemented PID controller, MPC controller has the ability to anticipate future events and can take control actions accordingly. Indeed, future time steps are taking into account while optimizing current time slot.
 - C++, MPC Controller

#### [11 - Path Planning](11_path_planning)
 - The goal in this project is to build a path planner that is able to create smooth, safe trajectories for the car to follow. The highway track has other vehicles, all going different speeds, but approximately obeying the 50 MPH speed limit. The car transmits its location, along with its sensor fusion data, which estimates the location of all the vehicles on the same side of the road.
 - C++, Path Planning

#### [12 - Road Segmentation](12_road_segmentation)
 - Implement the road segmentation using a fully-convolutional network.
 - Python, TensorFlow, Semantic Segmentation -->

# Disclaimer

The git commit history time is quite inaccurate as some projects were already done before this repo existed. 
Most media files (videos and images) were provided by Udacity. 
The driving simulators were developed by Udacity. 
The formatting of this file is from both Udacity's given template and [Andrea Palazzi](https://github.com/ndrplz). 

