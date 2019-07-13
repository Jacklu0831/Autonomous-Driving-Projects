grayscale
canny detection (gaussian + calculate gradient)
impose triangle mask on the original image with bitwise & operation
hough transform (gets lines array)
display lines on the black image
add the line image onto the original lane_image

-

Gaussian blur
https://docs.opencv.org/2.4/doc/tutorials/imgproc/gausian_median_blur_bilateral_filter/gausian_median_blur_bilateral_filter.html
https://towardsdatascience.com/tutorial-build-a-lane-detector-679fd8953132

talk about the fix for disappearing lines (compare no fix and quick fix videos)