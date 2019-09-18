#pragma once

#include "NvDecoder/NvDecoder.h"
#include "Utils/ColorSpace.h"
#include "Utils/FFmpegDemuxer.h"
#include "Utils/NvCodecUtils.h"
#include "include/blockingconcurrentqueue.h"
#include <algorithm>
#include <chrono>
#include <cuda.h>
#include <deque>
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

#define USE_GPU_OUTPUT 0

simplelogger::Logger *logger = simplelogger::LoggerFactory::CreateConsoleLogger();

void copy_image_cpu(CUdeviceptr dpSrc, uint8_t *dDst, int nWidth, int nHeight) {
  CUDA_MEMCPY2D m = {0};
  m.WidthInBytes = nWidth;
  m.Height = nHeight;

  m.srcMemoryType = CU_MEMORYTYPE_DEVICE;
  m.srcDevice = (CUdeviceptr)dpSrc;
  m.srcPitch = m.WidthInBytes;
  m.dstMemoryType = CU_MEMORYTYPE_HOST;
  m.dstDevice = (CUdeviceptr)(m.dstHost = dDst);

  m.dstPitch = m.WidthInBytes;
  cuMemcpy2D(&m);
}

void copy_image_gpu(CUdeviceptr dpSrc, uint8_t *dDst, int nWidth, int nHeight) {
  CUDA_MEMCPY2D m = {0};
  m.WidthInBytes = nWidth;
  m.Height = nHeight;

  m.srcMemoryType = CU_MEMORYTYPE_DEVICE;
  m.srcDevice = (CUdeviceptr)dpSrc;
  m.srcPitch = m.WidthInBytes;
  m.dstMemoryType = CU_MEMORYTYPE_DEVICE;
  m.dstDevice = (CUdeviceptr)(m.dstHost = dDst);

  m.dstPitch = m.WidthInBytes;
  cuMemcpy2D(&m);
}

struct VideoCapture {
  int gpu_id_ = 0;
  Dim resizeDim_ = {};
  std::string input_file_;
  float fps;
  std::shared_ptr<FFmpegDemuxer> demuxer;
  // std::shared_ptr<NvDecoder> dec;
  NvDecoder *dec;
  CUdevice cuDevice;
  CUcontext cuContext;
  Rect cropRect = {};
  int nVideoBytes = 0, nFrameReturned = 0, nFrame = 0;
  uint8_t *pVideo = NULL, **ppFrame;
  int nWidth, nHeight;
  int nFrameSize;
  bool stop;
  CUdeviceptr pTmpImage;
  uint8_t *pImage;
  ~VideoCapture() {
    cuMemFree(pTmpImage);
    delete[] pImage;
    delete dec;
    ck(cuCtxDestroy(cuContext));
  }

  VideoCapture(std::string input_file, int gpu_id, std::vector<int> new_size = std::vector<int>()) : gpu_id_(gpu_id), input_file_(input_file) {
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
      std::cout << "GPU ordinal out of range. Should be within [" << 0 << ", " << nGpu - 1 << "]" << std::endl;
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
    dec = new NvDecoder(cuContext, demuxer->GetWidth(), demuxer->GetHeight(), true, FFmpeg2NvCodecId(demuxer->GetVideoCodec()), nullptr, false, false, &cropRect, &resizeDim_);
  }

  cv::Mat read() {
    // if (!nVideoBytes)
    // return cv::Mat();

    nFrameReturned = 0;

    while (!nFrameReturned) {
      demuxer->Demux(&pVideo, &nVideoBytes);
      dec->Decode(pVideo, nVideoBytes, &ppFrame, &nFrameReturned);
      if (!nVideoBytes) {
        stop = true;
        return cv::Mat();
      }
    }
    nFrame += nFrameReturned;

    if (dec->GetBitDepth() == 8) {
      if (dec->GetOutputFormat() == cudaVideoSurfaceFormat_YUV444)
        YUV444ToColorPlanar<BGRA32>(ppFrame[0], dec->GetWidth(), (uint8_t *)pTmpImage, dec->GetWidth(), dec->GetWidth(), dec->GetHeight());
      else
        Nv12ToColorPlanar<BGRA32>(ppFrame[0], dec->GetWidth(), (uint8_t *)pTmpImage, dec->GetWidth(), dec->GetWidth(), dec->GetHeight());
      copy_image_cpu(pTmpImage, pImage, dec->GetWidth(), 3 * dec->GetHeight());
    } else {
      if (dec->GetOutputFormat() == cudaVideoSurfaceFormat_YUV444_16Bit)
        YUV444P16ToColorPlanar<BGRA32>(ppFrame[0], 2 * dec->GetWidth(), (uint8_t *)pTmpImage, dec->GetWidth(), dec->GetWidth(), dec->GetHeight());
      else
        P016ToColorPlanar<BGRA32>(ppFrame[0], 2 * dec->GetWidth(), (uint8_t *)pTmpImage, dec->GetWidth(), dec->GetWidth(), dec->GetHeight());
      copy_image_cpu(pTmpImage, pImage, dec->GetWidth(), 3 * dec->GetHeight());
    }

    // std::cout << "output format: " << dec->GetOutputFormat() << std::endl;
    // std::cout << "depth : " << dec->GetBitDepth() << std::endl;
    // std::cout << "height:" << dec->GetHeight() << std::endl;
    // std::cout << "width :" << dec->GetWidth() << std::endl;
    // std::cout << "size :" << dec->GetFrameSize() << std::endl;
    // std::cout << "nFramesize: " << nFrameSize << std::endl;
    // std::cout << "nFrame Retured: " << nFrameReturned << std::endl;
    // std::cout << "nVideoBytes:" << nVideoBytes << std::endl;

    cv::Mat img(nHeight, nWidth, CV_8UC3);

#pragma omp parallel for
    for (int i = 0; i < img.rows; ++i) {
      cv::Vec3b *p = img.ptr<cv::Vec3b>(i);
      for (int j = 0; j < img.cols; ++j) {
        int index = 3 * i * img.cols + 3 * j;
        p[j][0] = pImage[index];
        p[j][1] = pImage[index + 1];
        p[j][2] = pImage[index + 2];
      }
    }
    return img;
  }
};



struct VideoCaptureGPU {
  int gpu_id_ = 0;
  Dim resizeDim_ = {};
  std::string input_file_;
  float fps;
  std::shared_ptr<FFmpegDemuxer> demuxer;
  // std::shared_ptr<NvDecoder> dec;
  NvDecoder *dec;
  CUdevice cuDevice;
  CUcontext cuContext;
  Rect cropRect = {};
  int nVideoBytes = 0, nFrameReturned = 0, nFrame = 0;
  uint8_t *pVideo = NULL, **ppFrame;
  int nWidth, nHeight;
  int nFrameSize;
  bool stop;
  CUdeviceptr pTmpImage;
  ~VideoCaptureGPU() {
    cuMemFree(pTmpImage);
    delete dec;
    ck(cuCtxDestroy(cuContext));
  }

  VideoCaptureGPU(std::string input_file, int gpu_id, std::vector<int> new_size = std::vector<int>()) : gpu_id_(gpu_id), input_file_(input_file) {
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
      std::cout << "GPU ordinal out of range. Should be within [" << 0 << ", " << nGpu - 1 << "]" << std::endl;
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
    dec = new NvDecoder(cuContext, demuxer->GetWidth(), demuxer->GetHeight(), true, FFmpeg2NvCodecId(demuxer->GetVideoCodec()), nullptr, false, false, &cropRect, &resizeDim_);
  }

  uint8_t *read() {
    // if (!nVideoBytes)
    // return cv::Mat();

    nFrameReturned = 0;

    while (!nFrameReturned) {
      demuxer->Demux(&pVideo, &nVideoBytes);
      dec->Decode(pVideo, nVideoBytes, &ppFrame, &nFrameReturned);
      if (!nVideoBytes) {
        stop = true;
        return nullptr;
      }
    } 
    uint8_t * pImage;
    cudaMalloc((void **)&pImage,nFrameSize);
    nFrame += nFrameReturned;

    if (dec->GetBitDepth() == 8) {
      if (dec->GetOutputFormat() == cudaVideoSurfaceFormat_YUV444)
        YUV444ToColorPlanar<BGRA32>(ppFrame[0], dec->GetWidth(), (uint8_t *)pTmpImage, dec->GetWidth(), dec->GetWidth(), dec->GetHeight());
      else
        Nv12ToColorPlanar<BGRA32>(ppFrame[0], dec->GetWidth(), (uint8_t *)pTmpImage, dec->GetWidth(), dec->GetWidth(), dec->GetHeight());
      copy_image_gpu(pTmpImage, pImage, dec->GetWidth(), 3 * dec->GetHeight());
    } else {
      if (dec->GetOutputFormat() == cudaVideoSurfaceFormat_YUV444_16Bit)
        YUV444P16ToColorPlanar<BGRA32>(ppFrame[0], 2 * dec->GetWidth(), (uint8_t *)pTmpImage, dec->GetWidth(), dec->GetWidth(), dec->GetHeight());
      else
        P016ToColorPlanar<BGRA32>(ppFrame[0], 2 * dec->GetWidth(), (uint8_t *)pTmpImage, dec->GetWidth(), dec->GetWidth(), dec->GetHeight());
      copy_image_gpu(pTmpImage, pImage, dec->GetWidth(), 3 * dec->GetHeight());
    }

    

    // std::cout << "output format: " << dec->GetOutputFormat() << std::endl;
    // std::cout << "depth : " << dec->GetBitDepth() << std::endl;
    // std::cout << "height:" << dec->GetHeight() << std::endl;
    // std::cout << "width :" << dec->GetWidth() << std::endl;
    // std::cout << "size :" << dec->GetFrameSize() << std::endl;
    // std::cout << "nFramesize: " << nFrameSize << std::endl;
    // std::cout << "nFrame Retured: " << nFrameReturned << std::endl;
    // std::cout << "nVideoBytes:" << nVideoBytes << std::endl;

    return pImage;
  }
};