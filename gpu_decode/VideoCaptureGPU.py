import numpy as np
from .video import VideoCaptureGPU as Video
import mxnet as mx

class VideoCapture(object):
  def __init__(self,file_name, gpu_id, resize=(0, 0)):
    self.video = Video(file_name, gpu_id, resize)
    self.is_stop = False

  def read(self):
    img = np.array(self.video.read(), copy=False)
    self.is_stop = self.video.is_stop
    if self.is_stop:
      return None
    return self.is_stop, img
