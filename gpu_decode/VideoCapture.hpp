
#include "NvDecoder/NvDecoder.h"
#include "Utils/ColorSpace.h"
#include "Utils/FFmpegDemuxer.h"
#include "Utils/NvCodecUtils.h"
#include "include/blockingconcurrentqueue.h"
#include <algorithm>
#include <chrono>
#include <cuda.h>
#include <deque>
#include <dlpack/dlpack.h>
#include <functional>
#include <iostream>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/parseutils.h>
#include <libswscale/swscale.h>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

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
  bool gpu_output;
  std::shared_ptr<FFmpegDemuxer> demuxer;
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
  ~VideoCapture() {
    cuMemFree(pTmpImage);
    delete dec;
    ck(cuCtxDestroy(cuContext));
  }

  VideoCapture(std::string input_file, int gpu_id, std::vector<int> new_size = std::vector<int>(),
               bool gpu_output = false)
      : gpu_id_(gpu_id), input_file_(input_file), gpu_output(gpu_output) {
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
    dec = new NvDecoder(cuContext, demuxer->GetWidth(), demuxer->GetHeight(), true,
                        FFmpeg2NvCodecId(demuxer->GetVideoCodec()), nullptr, false, false, &cropRect, &resizeDim_);
  }

  DLManagedTensor *read() {

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
        YUV444ToColorPlanar<BGRA32>(ppFrame[0], dec->GetWidth(), (uint8_t *)pTmpImage, dec->GetWidth(), dec->GetWidth(),
                                    dec->GetHeight());
      else
        Nv12ToColorPlanar<BGRA32>(ppFrame[0], dec->GetWidth(), (uint8_t *)pTmpImage, dec->GetWidth(), dec->GetWidth(),
                                  dec->GetHeight());
      // copy_image_cpu(pTmpImage, pImage, dec->GetWidth(), 3 * dec->GetHeight());
    } else {
      if (dec->GetOutputFormat() == cudaVideoSurfaceFormat_YUV444_16Bit)
        YUV444P16ToColorPlanar<BGRA32>(ppFrame[0], 2 * dec->GetWidth(), (uint8_t *)pTmpImage, dec->GetWidth(),
                                       dec->GetWidth(), dec->GetHeight());
      else
        P016ToColorPlanar<BGRA32>(ppFrame[0], 2 * dec->GetWidth(), (uint8_t *)pTmpImage, dec->GetWidth(),
                                  dec->GetWidth(), dec->GetHeight());
      // copy_image_cpu(pTmpImage, pImage, dec->GetWidth(), 3 * dec->GetHeight());
    }
    uint8_t *pImage;
    if (gpu_output) {
      cudaMalloc((void **)&pImage, nFrameSize);
      copy_image_gpu(pTmpImage, pImage, 3 * nHeight, nWidth);
      DLManagedTensor *dltensor = new DLManagedTensor;
      dltensor->dl_tensor.data = pImage;
      dltensor->dl_tensor.ctx.device_type = kDLGPU;
      dltensor->dl_tensor.ctx.device_id = gpu_id_;
      dltensor->dl_tensor.ndim = 3;
      dltensor->dl_tensor.dtype.code = kDLUInt;
      dltensor->dl_tensor.dtype.bits = 8;
      dltensor->dl_tensor.shape = new int64_t[3];
      dltensor->dl_tensor.shape[0] = nHeight;
      dltensor->dl_tensor.shape[1] = nWidth;
      dltensor->dl_tensor.shape[2] = 3;
      dltensor->dl_tensor.strides = new int64_t[3];
      dltensor->dl_tensor.strides[0] = nWidth * 3;
      dltensor->dl_tensor.strides[1] = 3;
      dltensor->dl_tensor.strides[2] = 1;

      return dltensor;

    } else {
      pImage = new uint8_t[nFrameSize];
      copy_image_cpu(pTmpImage, pImage, 3 * nHeight, nWidth);
      DLManagedTensor *dltensor = new DLManagedTensor;
      dltensor->dl_tensor.data = pImage;
      dltensor->dl_tensor.ctx.device_type = kDLCPU;
      // dltensor->dl_tensor.ctx.device_id = gpu_id_;
      dltensor->dl_tensor.ndim = 3;
      dltensor->dl_tensor.dtype.code = kDLUInt;
      dltensor->dl_tensor.dtype.bits = 8;
      dltensor->dl_tensor.shape = new int64_t[3];
      dltensor->dl_tensor.shape[0] = nHeight;
      dltensor->dl_tensor.shape[1] = nWidth;
      dltensor->dl_tensor.shape[2] = 3;
      dltensor->dl_tensor.strides = new int64_t[3];
      dltensor->dl_tensor.strides[0] = nWidth * 3;
      dltensor->dl_tensor.strides[1] = 3;
      dltensor->dl_tensor.strides[2] = 1;
    }
    // DLManagedTensor *dltensor = new DLManagedTensor;
    // dltensor->dl_tensor.data = pImage;
  };
};

// extern "C" {
// VideoCapture *video(std::string input_file, int gpu_id, int h = 0, int w = 0, bool gpu_output = false) {
//   return new VideoCapture(input_file, gpu_id, std::vector<int>{h, w}, gpu_output);
// }

// DLManagedTensor *read(VideoCapture *video) { return video->read(); }
// }