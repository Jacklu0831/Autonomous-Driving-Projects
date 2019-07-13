<h1 align="center">SSD Object Detection</h1>

<p align="center">
  <a><img src="result.gif" alt=""></a>
</p>

### Summary

I Learned about *MobileNets* and the computational efficiency of separable depthwise convolutions, implemented the SSD (Single Shot Detection) architecture used for object detection, used pretrained TensorFlow object detection inference models to detect objects, and constructed and applied object detection pipeline to a driving video. For more detailed pipeline of this project, the please refer to [this file](Object_Detection.ipynb).

### Try it Yourself

Install environment with [Anaconda](https://www.continuum.io/downloads):

```sh
conda env create -f environment.yml
```

Change TensorFlow pip installation from `tensorflow-gpu` to `tensorflow` if you don't have a GPU available.

The environment should be listed via `conda info --envs`:

```sh
# conda environments:
#
carnd-advdl-odlab        /usr/local/anaconda3/envs/carnd-advdl-odlab
root                  *  /usr/local/anaconda3
```

Now you can download and open [this file](Object_Detection.ipynb) and play with it on your local machine.

### Documents
[An indepth paper on the speed/accuracy trade-offs of object detection algorithms](https://arxiv.org/pdf/1611.10012.pdf)
