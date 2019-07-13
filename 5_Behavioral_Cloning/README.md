# 4 - Behavioral Cloning

The automated vehicle learns to steer a car on simulated tracks from seeing how I drive it with WASD buttons. This project demonstrates the power of deep learning. I also employed self-made image augmentation generator with ImgAug on top of the Udacity project to increase the performance. I purposefully fed in less data than given to demonstrate the usefulness of image augmentation.

Performance with Image Augmentation
[INSERT MEDIA]

As can be seen, the autonomous car drives smoothly without failure on both the track it was trained on (track 1) and the track it has never seen (track 2). If the model fails on more difficult tracks, other useful techniques such as learning rate decay and a recovery lap could be employed.

Performance without Image Augmentation 
[INSERT MEDIA]

Without image augmentation, the neural network does not have enough training data and is very prone to overfitting. Therefore, it did not take long to crash on track 1 and didn't even have a chance on track 2.

### Summary

The goal of this project is to train the [NVIDIA neural network](https://arxiv.org/pdf/1604.07316v1.pdf) to replicate human steering behavior while driving. It takes three pictures from camera: roof center, roof left, and roof right as input data. I first drove the vehicle myself in the training mode, then deployed the trained model in the autonomous mode to qualitatively measure the output performance. The car is able to steer not only on the road it was trained on, but also track 2 of the Udacity simulator. 

### Pipeline

The project is written in jupyter notebook. If you are interest, give [INSERT FILE] a visit to see more visualization of process

Dataset
- Play the game! I drove the car for 2 rounds with WASD buttons instead of using the Udacity dataset (I purposefully kept the training set low to show the power of data augmentation)
- Get and visualize the training data. which has 3 frame from different cameras as well as the steering direction
- Plot histogram of steering angles and find out that the steering angle of 0 dominates all other angles, it will cause the car to move straight when there is a turn
- Reduce the skew toward driving straight by reducing the cases of steering angle with 0 to 200

Neural Network
- Network architecture is the aforementioned NVIDIA paper in which they tackle the same problem of steering angle direction (but much more real world)
- The architecture
[INSERT IMAGE]
- The input is normalized and ELU is chosen instead of ReLU to mitigate the effect of neuron deaths
- The convolutional layers were followed by 3 fully-connected layers and a single neuron regressing to the correct steering angle value
- Dropout layers showed good performance in reducing overfitting but the accuracy of the vehicle could be improved using image augmentation techniques instead
- Model compiled with Adam optimizer (default) and root mean square error loss w.r.t. the ground truth steering angle. 

### Try it Yourself

You will need Python 3, numpy, matplotlib, OpenCV, tensorflow, keras, sklearn, time, imgaug, pillow, and of course, the Udacity simulator for running this project.
I highly recommend creating a virtual environment with conda to organize the dependencies.

### Success Criteria

The autonomated driving in the Udacity simulator has to make smooth turns and always stay on the road. As a challenge, it has to be able to drive on the 2nd track without ever "seeing it". This is a qualitative evaluation, but frenet coordinates could be implemented to keep track of how off-centered the vehicle is. 