#include <ctime>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <string>

using namespace std;
using namespace cv;

int main() {
  VideoCapture video("../J502头_20190818180000_20190818190000.mp4");
  Mat frame;
  int count = 0;
  clock_t start = clock();
  for (;;) {
    // wait for a new frame from camera and store it into 'frame'
    video.read(frame);
    // check if we succeeded
    if (frame.empty()) {
      break;
    }
    count += 1;
    if ((count % 100) == 0)
      cout << count << endl;
  }
  clock_t end = clock();

  std::cout << "cost :" << (double)(end - start) / CLOCKS_PER_SEC << std::endl;
}