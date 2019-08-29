<h1 align="center">Finding Lane Lines</h1>
  
<p align="center">
  <a><img src="result.gif" alt=""></a>
</p>

### Summary

Created a simple pipeline to detect road lines from a roof-mounted camera. No neural network was involved with this project and the straight lines do a poor job detecting curved lines. Refer to [this continuation](https://github.com/Jacklu0831/Self-Driving-Car/tree/master/2_Advanced_Lane_Finding) for a more advanced pipeline that detects curvature and maps out curved lanes. 

### Pipeline

- Color selection to start preprocessing the image
- Region masking to get rid of background noise
- Canny edge detection, detect edges through gradient (sobel) thresholding
- Hough transform to extract the most likely slope and intercept of lane lines
- Parameter tuning

### Try it Yourself

You will need Python 3, numpy, matplotlib, and OpenCV. 
I highly recommend creating a virtual environment with conda to organize the dependencies.

### Success Criteria

The lane should be detected with two straight red lines on each side. Human-level performance in finding the lanes perfect enough for you to see if the algorithm is working.

### Documents
- [Gaussian blur](https://docs.opencv.org/2.4/doc/tutorials/imgproc/gausian_median_blur_bilateral_filter/gausian_median_blur_bilateral_filter.html)
