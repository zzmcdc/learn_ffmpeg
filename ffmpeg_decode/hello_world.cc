#ifdef __cplusplus
extern "C" {
#endif

#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include <libavfilter/avfilter.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>

#ifdef __cplusplus
}
#endif
#include <cinttypes>
#include <cstdio>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <vector>

using std::cin;
using std::cout;
using std::endl;
using std::string;
using std::vector;

void logging(const char *fmt, ...) {
  va_list args;
  fprintf(stderr, "LOG: ");
  va_start(args, fmt);
  vfprintf(stderr, fmt, args);
  va_end(args);
  fprintf(stderr, "\n");
}

int main(int argc, char **argv) {
  AVFormatContext *fmt_ctx = avformat_alloc_context();
  avformat_open_input(&fmt_ctx, "/home/zhao/work/learn/learn_ffmpeg/bunny_1080p_60fps.mp4", NULL, NULL);
  cout << "Format: [" << fmt_ctx->iformat->long_name << "] duration: [" << fmt_ctx->duration << "]" << endl;
  avformat_find_stream_info(fmt_ctx, NULL);
  AVCodec *codec = NULL;
  AVCodecParameters *param_code = NULL;
  int video_stream_index = -1;
  for (int i = 0; i < fmt_ctx->nb_streams; ++i) {
    AVCodecParameters *code_param_local = NULL;
    code_param_local = fmt_ctx->streams[i]->codecpar;
    logging("[%d] AVStream->time_base before open coded %d/%d", i, fmt_ctx->streams[i]->time_base.num, fmt_ctx->streams[i]->time_base.den);
    logging("[%d] AVStream->r_frame_rate before open coded %d/%d", i, fmt_ctx->streams[i]->r_frame_rate.num, fmt_ctx->streams[i]->r_frame_rate.den);
    logging("[%d] AVStream->start_time %" PRId64, i, fmt_ctx->streams[i]->start_time);
    logging("[%d] AVStream->duration %" PRId64, i, fmt_ctx->streams[i]->duration);

    AVCodec *codec_local = NULL;
    codec_local = avcodec_find_decoder(code_param_local->codec_id);

    if (code_param_local->codec_type == AVMEDIA_TYPE_VIDEO) {
      video_stream_index = i;
      codec = codec_local;
      param_code = code_param_local;
      logging("Video Codec: resolution %d x %d", param_code->width, param_code->height);
      break;
    }
  }

  AVCodecContext *code_ctx = avcodec_alloc_context3(codec);
  avcodec_parameters_to_context(code_ctx, param_code);
  avcodec_open2(code_ctx, codec, NULL);

  AVFrame *frame = av_frame_alloc();
  AVPacket *pkt = av_packet_alloc();
  int respond = 0;
  int need_process = 8;
  uint8_t *data[4];
  int linesize[4];
  while (av_read_frame(fmt_ctx, pkt) >= 0) {
    if (pkt->stream_index == video_stream_index) {
      avcodec_send_packet(code_ctx, pkt);
      respond = avcodec_receive_frame(code_ctx, frame);
      int ret = av_image_alloc(data, linesize, frame->width, frame->height, AV_PIX_FMT_BGR24, 1);
      if (respond >= 0) {
        SwsContext *sws = sws_getContext(frame->width, frame->height, (AVPixelFormat)frame->format, frame->width, frame->height, AV_PIX_FMT_BGR24, SWS_BICUBIC,
                                         NULL, NULL, NULL);

        sws_scale(sws, (const uint8_t *const *)frame->data, frame->linesize, 0, frame->height, data, linesize);
        cv::Mat img(frame->height, frame->width, CV_8UC3, data[0]);
        cv::imshow("img", img);
        cv::waitKey(1);
      }
    }
    av_packet_unref(pkt);
  }
}
