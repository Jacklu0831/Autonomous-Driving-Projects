<h1 align="center">Traffic Sign Classifier</h1>

<p align="center">
  <a><img src="result.png" alt=""></a>
</p>

### Summary

Modified the simple convolutional neural network architecture [LeNet](http://yann.lecun.com/exdb/publis/pdf/lecun-98.pdf) to recognize traffic signs in various angles and lightings. Due to non-evenly distributed dataset for each type of traffic sign, image augmentaion was heavily used to combat overfitting. For more advanced CNN implementations such as YOLO or SSD by me, refer to [INSERT PROJECTS].

### Pipeline

The source code is written in Jupyter Notebook, please refer to [this](TrafficSignRecognizer.ipynb) for a more graphical presentation of the process.
- Load a data set of labeled traffic signs
- Explore the data set to make sure things are loaded correctly
- Preprocess the images, grayscaling since color does not matter in this context (also normalize)
- Design, train, test the model architecture
- Plot the history of fitting model to monitor the accuracy and loss parameters
- If the result indicate signs of overfitting, employ image augmentation techniques
- Use the model to make predictions on new images taken from the web (make sure to grayscale)
- If the history plot and predictions are not to standard, modify the CNN architecture or tweak hyperparameters, most important one being the learning rate

### Try it Yourself

You will need Python 3, numpy, matplotlib, pickle, OpenCV, tensorflow, keras, sklearn, time, and pillow.
I highly recommend creating a virtual environment with conda to organize the dependencies.

### Success Criteria

The model prediction on new data (images) of traffic signs are correct.
