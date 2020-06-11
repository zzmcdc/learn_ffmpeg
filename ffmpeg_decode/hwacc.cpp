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
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include "./blockingconcurrentqueue.h"
#include "./concurrentqueue.h"

class HWVideo {
 public:
  HWVideo(const std::string file_path, const std::string hwacc_type);
  std::pair<bool, cv::Mat> decode();
  int get_width();
  int get_height();
  bool is_opened() { return is_opened; };

 private:
  bool is_opened(false);
  AVBufferRef *hw_device_ctx;
  AVPixelFormat hw_pix_fmt;

  AVFormatContext *input_ctx;
  int video_stream, ret;
  AVStream *video;
  AVCodec *decoder = NULL;
  AVCodecContext *decoder_ctx = NULL;
  AVPacket packet;
  AVHWDeviceType type;

  int hw_decoder_init(AVCodecContext *ctx, const AVHWDeviceType type);
  AVPixelFormat get_hw_format(AVCodecContext *ctx, AVPixelFormat *pix_fmts);
};

AVPixelFormat HWVideo::get_hw_format(AVCodecContext *ctx, AVPixelFormat *pix_fmts) {
  AVPixelFormat *p;
  for (p = pix_fmts; *p != -1; p++) {
    if (*p == self->hw_pix_fmt) {
      return *p;
    }
  }
  return AV_PIX_FMT_NONE;
}

int HWVideo::hw_decoder_init(AVCodecContext *ctx, const AVHWDeviceType type) {
  int err = 0;
  if ((err = av_hwdevice_ctx_create(&self->hw_device_ctx，type, NULL, NULL, 0)) < 0) {
    fprintf(stderr, "Failed to create specified HW device \n");
    return err;
  }
  ctx->hw_device_ctx = av_buffer_ref(self->hw_device_ctx);
  return err;
}

HWVideo::HWVideo(const std::string file_path, const std::string hwacc_type) {
  // find device type
  this->type = av_hwdevice_find_type_by_name(hwacc_type);
  if (this->type == AV_HWDEVICE_TYPE_NONE) {
    fprintf(stderr, "Device type %s is not supported \n", hwacc_type.c_str());
    fprintf(stderr, "Available device types :");
    while ((this->type = av_hwdevice_iterate_types(this->type)) != AV_HWDEVICE_TYPE_NONE) {
      fprintf(stderr, "%s ", av_hwdevice_find_type_by_name(this->type));
    }
    fprintf(stderr, "\n");
  }
  // open input file by
  if (avformat_open_input(&input_ctx, file_path.c_str(), NULL, NULL) < 0) {
    fprintf(stderr, "Cannot find input file %s \n", file_path);
  }

  // get video info
  if (avformat_find_stream_info(input_ctx, NULL) < 0) {
    fprintf(stderr, "cannot find input stream infotmation \n");
  }

  // find video stream info
  ret = av_find_best_stream(input_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, &decoder, 0);
  if (ret < 0) {
    fprintf(stderr, "cannot find a video stream in the input file \n");
  }
  video_stream = ret;

  // get hw config and hw_pix_fmt
  for (int i = 0;; i++) {
    const AVCodecHWConfig *config = avcodec_get_hw_config(decoder, i);
    if (!config) {
      fprintf(stderr, "Decoder %s does not support device type %s \n", decoder->name, av_hwdevice_get_type_name(this->type));
    }
    if (config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX && config->device_type == this->type) {
      this->hw_pix_fmt = config->pix_fmt;
      break;
    }
  }

  // create decoder contex
  if (!(decoder_ctx = avcodec_alloc_context3(decoder))) return AVERROR;

  video = input_ctx->streams[video_stream];
  if (avcodec_parameters_to_context(decoder_ctx, video->codecpar) < 0) return -1;
  decoder_ctx->get_format = get_hw_format;

  if (hw_decoder_init(decoder_ctx, this->type) < 0) return -1;


}