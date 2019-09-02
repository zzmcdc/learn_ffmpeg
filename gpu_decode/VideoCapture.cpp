
#include "NvDecoder/NvDecoder.h"
#include "Utils/ColorSpace.h"
#include "Utils/FFmpegDemuxer.h"
#include "Utils/NvCodecUtils.h"
#include <algorithm>
#include <cuda.h>
#include <iostream>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/parseutils.h>
#include <libswscale/swscale.h>
#include <memory>
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vector>

simplelogger::Logger *logger = simplelogger::LoggerFactory::CreateConsoleLogger();

void GetImage(CUdeviceptr dpSrc, uint8_t *pDst, int nWidth, int nHeight) {
  CUDA_MEMCPY2D m = {0};
  m.WidthInBytes = nWidth;
  m.Height = nHeight;
  m.srcMemoryType = CU_MEMORYTYPE_DEVICE;
  m.srcDevice = (CUdeviceptr)dpSrc;
  m.srcPitch = m.WidthInBytes;
  m.dstMemoryType = CU_MEMORYTYPE_HOST;
  m.dstDevice = (CUdeviceptr)(m.dstHost = pDst);
  m.dstPitch = m.WidthInBytes;
  cuMemcpy2D(&m);
}

int anSize[] = {0, 3, 3, 4, 4, 8, 8};
enum OutputFormat { native = 0, bgrp, rgbp, bgra, rgba, bgra64, rgba64 };

std::vector<std::string> vstrOutputFormatName = {"native", "bgrp", "rgbp", "bgra", "rgba", "bgra64", "rgba64"};

struct VideoCapture {
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
  OutputFormat eOutputFormat = bgrp;

  VideoCapture(std::string input_file, int gpu_id, std::vector<int> new_size = std::vector<int>()) : gpu_id_(gpu_id), input_file_(input_file) {
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

    dec = std::make_shared<NvDecoder>(cuContext, demuxer->GetWidth(), demuxer->GetHeight(), true, FFmpeg2NvCodecId(demuxer->GetVideoCodec()), nullptr, false, false, &cropRect, &resizeDim_);
  }

  cv::Mat read() {
    // if (!nVideoBytes)
    // return cv::Mat();
    nWidth = demuxer->GetWidth();
    nHeight = demuxer->GetHeight();
    nFrameSize = nWidth * nHeight * anSize[eOutputFormat];
    nFrameReturned = 0;
    uint8_t *pImage = new uint8_t[nFrameSize];
    CUdeviceptr pTmpImage = 0;
    cuMemAlloc(&pTmpImage, nWidth * nHeight * anSize[eOutputFormat]);

    while (!nFrameReturned) {
      demuxer->Demux(&pVideo, &nVideoBytes);
      dec->Decode(pVideo, nVideoBytes, &ppFrame, &nFrameReturned);
    }
    nFrame += nFrameReturned;

    if (dec->GetBitDepth() == 8) {
      if (dec->GetOutputFormat() == cudaVideoSurfaceFormat_YUV444)
        YUV444ToColorPlanar<BGRA32>(ppFrame[0], dec->GetWidth(), (uint8_t *)pTmpImage, dec->GetWidth(), dec->GetWidth(), dec->GetHeight());
      else
        Nv12ToColorPlanar<BGRA32>(ppFrame[0], dec->GetWidth(), (uint8_t *)pTmpImage, dec->GetWidth(), dec->GetWidth(), dec->GetHeight());
      GetImage(pTmpImage, pImage, dec->GetWidth(), 3 * dec->GetHeight());
    } else {
      if (dec->GetOutputFormat() == cudaVideoSurfaceFormat_YUV444_16Bit)
        YUV444P16ToColorPlanar<BGRA32>(ppFrame[0], 2 * dec->GetWidth(), (uint8_t *)pTmpImage, dec->GetWidth(), dec->GetWidth(), dec->GetHeight());
      else
        P016ToColorPlanar<BGRA32>(ppFrame[0], 2 * dec->GetWidth(), (uint8_t *)pTmpImage, dec->GetWidth(), dec->GetWidth(), dec->GetHeight());
      GetImage(pTmpImage, pImage, dec->GetWidth(), 3 * dec->GetHeight());
    }
    nFrame += nFrameReturned;

    std::cout << "output format: " << dec->GetOutputFormat() << std::endl;
    std::cout << "depth : " << dec->GetBitDepth() << std::endl;
    std::cout << "height:" << dec->GetHeight() << std::endl;
    std::cout << "width :" << dec->GetWidth() << std::endl;
    std::cout << "size :" << dec->GetFrameSize() << std::endl;
    std::cout << "nFramesize: " << nFrameSize << std::endl;
    std::cout << "nFrame Retured: " << nFrameReturned << std::endl;
    std::cout << "nVideoBytes:" << nVideoBytes << std::endl;

    std::vector<cv::Mat> channels;
    cv::Mat b(nHeight, nWidth, CV_8UC1, pImage);
    pImage += nHeight * nWidth;
    cv::Mat g(nHeight, nWidth, CV_8UC1, pImage);
    pImage += nHeight * nWidth;
    cv::Mat r(nHeight, nWidth, CV_8UC1, pImage);

    channels.push_back(b);
    channels.push_back(g);
    channels.push_back(r);

    cv::Mat fig;
    cv::merge(channels, fig);
    cuMemFree(pTmpImage);
    return fig;
  }
};

int main() {

  VideoCapture video("../test.mp4", 0);
  cv::Mat result;
  for (int i = 0; i < 100; ++i)
    result = video.read();

  cv::imwrite("test.jpg", result);
}
