#pragma once

#include "NvDecoder/NvDecoder.h"
#include "Utils/ColorSpace.h"
#include "Utils/FFmpegDemuxer.h"
#include "Utils/NvCodecUtils.h"
#include "include/concurrentqueue.h"
#include <algorithm>
#include <chrono>
#include <cuda.h>
#include <deque>
#include <functional>
#include <iostream>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/parseutils.h>
#include <libswscale/swscale.h>
#include <memory>
#include <mutex>
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#define USE_GPU_OUTPUT 1

simplelogger::Logger *logger =
    simplelogger::LoggerFactory::CreateConsoleLogger();

void copy_image(CUdeviceptr dpSrc, uint8_t *dDst, int nWidth, int nHeight) {
  CUDA_MEMCPY2D m = {0};
  m.WidthInBytes = nWidth;
  m.Height = nHeight;

  m.srcMemoryType = CU_MEMORYTYPE_DEVICE;
  m.srcDevice = (CUdeviceptr)dpSrc;
  m.srcPitch = m.WidthInBytes;
#if USE_GPU_OUTPUT
  m.dstMemoryType = CU_MEMORYTYPE_DEVICE;
#else
  m.dstMemoryType = CU_MEMORYTYPE_HOST;
#endif
  m.dstDevice = (CUdeviceptr)(m.dstHost = dDst);

  m.dstPitch = m.WidthInBytes;
  cuMemcpy2D(&m);
}

struct VideoCaptureBase {
  int gpu_id_ = 0;
  Dim resizeDim_ = {};
  std::string input_file_;
  float fps;
  std::shared_ptr<FFmpegDemuxer> demuxer;
  std::shared_ptr<NvDecoder> dec;
  CUdevice cuDevice;
  CUcontext cuContext;
  Rect cropRect = {};
  int nVideoBytes = 0, nFrameReturned = 0, nFrame = 0;
  uint8_t *pVideo = NULL, **ppFrame;
  int nWidth, nHeight;
  int nFrameSize;
  bool stop;
  CUdeviceptr pTmpImage;
  std::function<void()> callback_;

  ~VideoCaptureBase() { cuMemFree(pTmpImage); }

  VideoCaptureBase(std::string input_file, int gpu_id,
                   std::vector<int> new_size = std::vector<int>())
      : gpu_id_(gpu_id), input_file_(input_file) {
    if (new_size.size() > 0) {
      resizeDim_.h = new_size[0];
      resizeDim_.w = new_size[1];
    } else {
      resizeDim_.h = 0;
      resizeDim_.w = 0;
    }
    stop = false;

    ck(cuInit(0));
    int nGpu = 0;
    ck(cuDeviceGetCount(&nGpu));
    if (gpu_id_ < 0 || gpu_id_ >= nGpu) {
      std::cout << "GPU ordinal out of range. Should be within [" << 0 << ", "
                << nGpu - 1 << "]" << std::endl;
      exit(0);
    }

    cuDevice = 0;
    ck(cuDeviceGet(&cuDevice, gpu_id_));
    cuContext = NULL;
    ck(cuCtxCreate(&cuContext, 0, cuDevice));
    demuxer = std::make_shared<FFmpegDemuxer>(input_file_.c_str());

    nWidth = demuxer->GetWidth();
    nHeight = demuxer->GetHeight();
    nFrameSize = nWidth * nHeight * 3;

    cuMemAlloc(&pTmpImage, nWidth * nHeight * 3);
    pImage = new uint8_t[nFrameSize];
    dec = std::make_shared<NvDecoder>(
        cuContext, demuxer->GetWidth(), demuxer->GetHeight(), true,
        FFmpeg2NvCodecId(demuxer->GetVideoCodec()), nullptr, false, false,
        &cropRect, &resizeDim_);
  }

  void decode() {
    do {
      nFrameReturned = 0;
      demuxer->Demux(&pVideo, &nVideoBytes);
      dec->Decode(pVideo, nVideoBytes, &ppFrame, &nFrameReturned);
      nFrame += nFrameReturned;

      for (int i = 0; i < nFrameReturned; i++) {
        if (dec->GetBitDepth() == 8) {
          if (dec->GetOutputFormat() == cudaVideoSurfaceFormat_YUV444)
            YUV444ToColorPlanar<BGRA32>(ppFrame[i], dec->GetWidth(),
                                        (uint8_t *)pTmpImage, dec->GetWidth(),
                                        dec->GetWidth(), dec->GetHeight());
          else
            Nv12ToColorPlanar<BGRA32>(ppFrame[i], dec->GetWidth(),
                                      (uint8_t *)pTmpImage, dec->GetWidth(),
                                      dec->GetWidth(), dec->GetHeight());
          // GetImage(pTmpImage, pImage, 3 * dec->GetWidth(), dec->GetHeight());
        } else {
          if (dec->GetOutputFormat() == cudaVideoSurfaceFormat_YUV444_16Bit)
            YUV444P16ToColorPlanar<BGRA32>(
                ppFrame[i], 2 * dec->GetWidth(), (uint8_t *)pTmpImage,
                dec->GetWidth(), dec->GetWidth(), dec->GetHeight());
          else
            P016ToColorPlanar<BGRA32>(ppFrame[i], 2 * dec->GetWidth(),
                                      (uint8_t *)pTmpImage, dec->GetWidth(),
                                      dec->GetWidth(), dec->GetHeight());
          // GetImage(pTmpImage, pImage, 3 * dec->GetWidth(), dec->GetHeight());
        }
        callback_(pTmpImage);
      }
    } while (nVideoBytes);
    stop = true;
  }
};
class VideoCapture() {
public:
  std::string filename;
  int gpu_id;
  std::vector<int> resize;
  std::shared_ptr<std::thread> worker;
  int height;
  int width;

#if USE_GPU_OUTPUT
  std::vector<unsigned char *> image;
#else
  std::vector<cv::Mat> image
#endif
  std::shared_ptr<VideoCaptureBase> video;

  VideoCapture(std::string file_name, int gpu_id, std::vector<int> new_size)
      : filename(file_name), gpu_id(gpu_id), resize(new_resize) {

    video = std::make_shared<VideoCaptureBase>(filename, gpu_id, resize);
    height = video->nWidth;
    width = video->nHeight;
#if USE_GPU_OUTPUT // use gpu data
    auto call_back = [this]() {
      unsigned char *data;
      size_t size = height * width * 3;
      cudaSetDevice(gpu_id);
      cudaMalloc((void **)&data, size * sizeof(uint8_t));
      copy_image(this->pTmpImage, data, 3 * this->dec->GetWidth(),
                 this->dec->GetHeight());
      this->image.push_back(data);
    }
  
#else

#endif
  }
}