
#include "NvDecoder/NvDecoder.h"
#include "Utils/FFmpegDemuxer.h"
#include "Utils/NvCodecUtils.h"
#include <algorithm>
#include <cuda.h>
#include <iostream>
#include <memory>
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vector>

struct VideoCapture {
  int gpu_id_ = 0;
  Dim resizeDim_ = {};
  std::string input_file_;
  float fps;
  std::shared_ptr<FFmpegDemuxer> demuxer;
  NvDecoder *dec;
  CUdevice cuDevice;
  CUcontext cuContext;
  Rect cropRect = {};
  int nVideoBytes = 0, nFrameReturned = 0, nFrame = 0;
  uint8_t *pVideo = NULL, **ppFrame;
  bool bDecodeOutSemiPlanar = false;
  static std::vector<std::string> aszDecodeOutFormat;

  VideoCapture(std::string input_file, int gpu_id, std::vector<int> new_size = std::vector<int>())
      : gpu_id_(gpu_id), input_file_(input_file) {
    if (new_size.size() > 0) {
      resizeDim_.h = new_size[0];
      resizeDim_.w = new_size[1];
    } else {
      resizeDim_.h = 0;
      resizeDim_.w = 0;
    }

    ck(cuInit(0));
    int nGpu = 0;
    ck(cuDeviceGetCount(&nGpu));
    if (gpu_id_ < 0 || gpu_id_ >= nGpu) {
      std::cout << "GPU ordinal out of range. Should be within [" << 0 << ", " << nGpu - 1 << "]" << std::endl;
      exit(0);
    }

    cuDevice = 0;
    ck(cuDeviceGet(&cuDevice, gpu_id_));
    cuContext = NULL;
    ck(cuCtxCreate(&cuContext, 0, cuDevice));
    demuxer = std::make_shared<FFmpegDemuxer>(input_file_.c_str());

    dec = new NvDecoder(cuContext, demuxer->GetWidth(), demuxer->GetHeight(), false,
                        FFmpeg2NvCodecId(demuxer->GetVideoCodec()), NULL, false, false, &cropRect, &resizeDim_);

    demuxer->Demux(&pVideo, &nVideoBytes);
    dec->Decode(pVideo, nVideoBytes, &ppFrame, &nFrameReturned);
    nFrame += nFrameReturned;
  }

  cv::Mat read() {
    if (!nVideoBytes)
      return cv::Mat();
    dec->Decode(pVideo, nVideoBytes, &ppFrame, &nFrameReturned);
    nFrame += nFrameReturned;
    return cv::Mat(resizeDim_.h, resizeDim_.w, cv::CV_8UC3, ppFrame[0]);
  }
};

int main() {}
