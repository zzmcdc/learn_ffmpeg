import numpy as np
from video import VideoCapture as video

class VideoCapture(object):
  def __init__(file_name, gpu_id, resize=(0, 0)):
    self.video = video(file_name, gpu_id, resize)
    self.is_stop = False

  def read():
    img = np.array(video.read(), copy=False)
    self.is_stop = self.video.is_stop
    if not self.is_stop:
      return None
    return self.is_stop, img
