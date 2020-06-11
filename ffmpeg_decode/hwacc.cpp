#ifdef __cplusplus
extern "C" {
#endif

#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include <libavfilter/avfilter.h>
#include <libavformat/avformat.h>
#include <libavutil/avassert.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>

#ifdef __cplusplus
}
#endif
#include <cinttypes>
#include <cstdio>
#include <exception>
#include <functional>
#include <iostream>
#include <string>
#include <utilty>
#include <vector>
#include <opencv4/opencv2/opencv.hpp>
#include <thread>

class HWVideo {
 public:
  HWVideo(const std::string file_path, const std::string hwacc_type);
  std::pair<bool, cv::Mat> decode();
  int get_width();
  int get_height();
  bool is_opened();
};
