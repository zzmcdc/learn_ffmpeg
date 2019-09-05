import numpy as np
import cv2
import video

v = video.VideoCapture('../test.mp4',0,(0,0))

for _ in range(100):
    img = np.array(v.read(), copy=False)

cv2.imshow('dd',img), cv2.waitKey(0), cv2.destroyAllWindows()
