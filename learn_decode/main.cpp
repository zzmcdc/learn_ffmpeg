#include <iostream>
#include <string>

#include <gflags/gflags.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <opencv2/opencv.hpp>
#include <functional>
#include <memory>

using namespace std;

DEFINE_string(filename, "test.avi", "input file name");

class Video
{
  public:
  std::shared_ptr<AVCodecContext> dec_ctx_;
  std::shared_ptr<AVFrame> frame_;
  std::shared_ptr<AVPacket> pkt_;
  size_t rate_;
}

static void decode(AVCodecContext *dec_ctx, AVFrame *frame, AVPacket *pkt, function<void()> callback) {
  char buf[1024];
  ret = avcodec_send_packet(dec_ctx, pkt);
  if (ret < 0) {
    std::cout << "Error sending a packet for decoding" << std::endl;
    exit(0);
  }
  while (ret >= 0) {
    ret = avcodec_receive_frame(dec_ctx, frame);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
      return;
    else if (ret < 0) {
      std::cout << "error during decodeing" << std::endl;
      exit(1);
    }

  }
}