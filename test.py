import numpy as np
import cv2
from gpu_decode.VideoCapture import VideoCapture
v = VideoCapture('test.mp4',0)
for _ in range(1):
    img = np.array(v.read(), copy=False)

cv2.imshow('dd',img[1]), cv2.waitKey(0), cv2.destroyAllWindows()
