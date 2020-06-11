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
#include <functional>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <vector>

AVPixelFormat get_hw_format(AVCodecContext *ctx, const enum AVPixelFormat *pix_fmts, const AVPixelFormat &hw_pix_fmt) {
  const AVPixelFormat *p;
  for (p = pix_fmts; *p != -1; p++) {
    if (*p == hw_pix_fmt) return *p;
  }
  fprintf(stderr, "Failed to get HW surface format.\n");
  return AV_PIX_FMT_NONE;
}

int main(int argc, char *argv[]) {
  AVBufferRef *hw_device_ctx = NULL;
  AVPixelFormat hw_pix_fmt;
  AVFormatContext *input_ctx = NULL;
  int video_stream, ret;
  AVStream *video = NULL;
  AVCodecContext *decoder_ctx = NULL;
  AVCodec *decoder = NULL;
  AVPacket packet;
  AVHWDeviceType type;
  int i;
  if (argc < 3) {
    fprintf(stderr, "Usage:%s <device type> <input file> \n", argv[0]);
  }

  type = av_hwdevice_find_type_by_name(argv[1]);

  if (type == AV_HWDEVICE_TYPE_NONE) {
    fprintf(stderr, "Device type %s is not supported. \n", argv[1]);
    fprintf(stderr, "Available device types:");
    while ((type = av_hwdevice_iterate_types(type)) != AV_HWDEVICE_TYPE_NONE) {
      fprintf(stderr, " %s", av_hwdevice_get_type_name(type));
    }
    fprintf(stderr, "\n");
    return -1;
  }

  if (avformat_open_input(&input_ctx, argv[2], NULL, NULL) < 0) {
    fprintf(stderr, "Cannot open input file %s\n", argv[2]);
    return -1;
  }

  if (avformat_find_stream_info(input_ctx, NULL) < 0) {
    fprintf(stderr, "Cannot find input stream information. \n");
    return -1;
  }

  ret = av_find_best_stream(input_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, &decoder, 0);
  if (ret < 0) {
    fprintf(stderr, "Cannot find a video stream in the input file \n");
    return -1;
  }
  video_stream = ret;

  for (i = 0;; i++) {
    const AVCodecHWConfig *config = avcodec_get_hw_config(decoder, i);
    if (!config) {
      fprintf(stderr, "Decoder %s does not support device type %s.\n", decoder->name, av_hwdevice_get_type_name(type));
      return -1;
    }
    if (config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX && config->device_type == type) {
      hw_pix_fmt = config->pix_fmt;
      break;
    }
  }

  if (!(decoder_ctx = avcodec_alloc_context3(decoder))) return AVERROR(ENOMEM);
  video = input_ctx->streams[video_stream];

  if (avcodec_parameters_to_context(decoder_ctx, video->codecpar) < 0) return -1;

  decoder_ctx->get_format =  std::bind(get_hw_format, std::placeholders::_1, std::placeholders::_2, hw_device_ctx);
}
