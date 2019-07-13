<h1 align="center">Advanced Lane Finding</h1>

<p align="center">
  <a><img src="result.gif" alt=""></a>
</p>

### Summary

Created a more complex pipline to detect road lines from a roof-mounted camera. The result not only detects the curved lane in real time, but also calculates the lane's curvature radius how off-center the car is (very useful data for autonomous driving). 

Despite the satisfying result shown above, the algorithm does not perform well with strong glare or obstacles. This is why CNN architectures ultimately needs to be employed. Here are my CNN related projects: [INSERT PROJECTS].

### Pipeline

Since I used Jupyter Notebook for this project, [this file](Advanced_Lane_Detection.ipynb) has the algorithm broken down step by step and the visualization of what each step does to the input media.

[//]: # (Image References)
[image1]: ./assets/undistort_output.png "Undistorted"
[image2]: ./test_images/test1.jpg "Road Transformed"
[image3]: ./assets/binary_combo_example.jpg "Binary Example"
[image4]: ./assets/warped_straight_lines.jpg "Warp Example"
[image5]: ./assets/color_fit_lines.jpg "Fit Visual"
[image6]: ./assets/example_output.jpg "Output"
[video1]: ./project_video.mp4 "Video"

#### Use camera calibration matrix and distortion from chessboard images to correct distortion of road images
![alt text][image1]

#### Use color transforms, gradients, etc., to create a thresholded binary image
![alt text][image3]

#### Apply a perspective transform to rectify binary image ("birds-eye view")
![alt text][image4]

#### Detect lane pixels and fit to find the lane boundary
![alt text][image5]

#### Determine the curvature of the lane and vehicle position with respect to center and warp result back to original image
![alt text][image6]

### Try it Yourself

You will need Python 3, numpy, matplotlib, OpenCV, moviepy, and os install in your environment to run the entire jupyter notebook. 
I highly recommend creating a virtual environment with conda to organize the dependencies.

### Success Criteria

The lane is detected in green with red and blue warped parabolas indicating the lane boundaries. The curvature and offset should have reasonable values. 
