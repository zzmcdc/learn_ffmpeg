import mxnet as mx
from .video import VideoCapture as Video

class VideoCapture(object):
  def __init__(self,file_name, gpu_id, resize=(0, 0), gpu_output=False):
    self.video = Video(file_name, gpu_id, resize, gpu_output)
    self.is_stop = False

  def read(self):
    handle = self.video.read()
    dltensor = self.video.to_capsule(handle)
    img = mx.nd.from_dlpack(dltensor)
    self.is_stop = self.video.is_stop
    if self.is_stop:
      return None
    return self.is_stop, img
