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
#include <utility>
#include <vector>
#include "./blockingconcurrentqueue.h"

class HWVideo {
 public:
  HWVideo(const std::string file_path, const std::string hwacc_type);
  std::pair<bool, cv::Mat> decode();
  int get_width();
  int get_height();
  bool is_opened() { return is_ready; }

 private:
  bool is_ready{false};
  AVBufferRef *hw_device_ctx;
  static AVPixelFormat hw_pix_fmt;

  AVFormatContext *input_ctx;
  int video_stream, ret;
  AVStream *video;
  AVCodec *decoder = NULL;
  AVCodecContext *decoder_ctx = NULL;
  AVPacket packet;
  AVHWDeviceType type;
  moodycamel::BlockingConcurrentQueue<cv::Mat> image_queue;

  int hw_decoder_init(AVCodecContext *ctx, const AVHWDeviceType type);
  static AVPixelFormat get_hw_format(AVCodecContext *ctx, const AVPixelFormat *pix_fmts);
  static void decode_fun(HWVideo *hw_video);
};

void HWVideo::decode_fun(HWVideo *hw_video) {
  int ret = 0;
  AVFrame *frame = NULL, *sw_frame = NULL;
  AVFrame *tmp_frame = NULL;
  uint8_t *buff = NULL;
  uint64_t size;
  // start to read packet and decode it in a thread
  while (ret >= 0) {
    if ((ret = av_read_frame(hw_video->input_ctx, &hw_video->packet)) < 0) {
      break;
    }

    if (hw_video->video_stream == hw_video->packet.stream_index) {
      // decode the frame and send to block queue
      ret = avcodec_send_packet(hw_video->decoder_ctx, &hw_video->packet);
      if (ret < 0) {
        fprintf(stderr, "Error during decoding \n");
        exit(1);
      }

      while (1) {
        if (!(frame = av_frame_alloc()) !!!(sw_frame = av_frame_alloc()))) {
            fprintf(stderr, "cannot not alloc frame \n");
            goto fail;
          }
      }
    }

    av_packet_unref(&hw_video->packet);
  }
}

AVPixelFormat HWVideo::get_hw_format(AVCodecContext *ctx, const AVPixelFormat *pix_fmts) {
  const AVPixelFormat *p;
  for (p = pix_fmts; *p != -1; p++) {
    if (*p == HWVideo::hw_pix_fmt) {
      return *p;
    }
  }
  return AV_PIX_FMT_NONE;
}

int HWVideo::hw_decoder_init(AVCodecContext *ctx, const AVHWDeviceType type) {
  int err = 0;
  if ((err = av_hwdevice_ctx_create(&this->hw_device_ctx, this->type, NULL, NULL, 0)) < 0) {
    fprintf(stderr, "Failed to create specified HW device \n");
    return err;
  }
  ctx->hw_device_ctx = av_buffer_ref(this->hw_device_ctx);
  return err;
}

HWVideo::HWVideo(const std::string file_path, const std::string hwacc_type) {
  // find device type
  this->type = av_hwdevice_find_type_by_name(hwacc_type.c_str());
  if (this->type == AV_HWDEVICE_TYPE_NONE) {
    fprintf(stderr, "Device type %s is not supported \n", hwacc_type.c_str());
    fprintf(stderr, "Available device types :");
    while ((this->type = av_hwdevice_iterate_types(this->type)) != AV_HWDEVICE_TYPE_NONE) {
      fprintf(stderr, "%s ", av_hwdevice_get_type_name(this->type));
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
  if (!(decoder_ctx = avcodec_alloc_context3(decoder))) {
  }

  video = input_ctx->streams[video_stream];
  if (avcodec_parameters_to_context(decoder_ctx, video->codecpar) < 0) {
  }
  decoder_ctx->get_format = HWVideo::get_hw_format;

  if (hw_decoder_init(decoder_ctx, this->type) < 0) {
  }

  // Initialize the AVCodecContext to use the given AVCodec. Prior to using this  function the context has to be allocated with avcodec_alloc_context3().
  if ((ret = avcodec_open2(decoder_ctx, decoder, NULL)) < 0) {
    fprintf(stderr, "failed to open codec for stream %d \n", video_stream);
  }

  // now you can read packet and decode the video and get frame.
  // we can do it by a thread
}