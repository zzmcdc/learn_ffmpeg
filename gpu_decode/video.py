import numpy as np
from video import VideoCapture as video

class VideoCapture(object):
    def __init__(file_name, gpu_id, resize=(0,0)):
        self.video = video(file_name, gpu_id, resize)
    def read():
        return np.array(video.read(), copy=False)

