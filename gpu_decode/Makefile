################################################################################
#
# Copyright 2017-2018 NVIDIA Corporation.  All rights reserved.
#
# Please refer to the NVIDIA end user license agreement (EULA) associated
# with this source code for terms and conditions that govern your use of
# this software. Any use, reproduction, disclosure, or distribution of
# this software and related documentation outside the terms of the EULA
# is strictly prohibited.
#
################################################################################

include common.mk

NVCCFLAGS := $(CCFLAGS)

LDFLAGS += -pthread -fopenmp
LDFLAGS += -L$(CUDA_PATH)/lib64 -lcudart -lnvcuvid
LDFLAGS += $(shell pkg-config --libs libavcodec libavutil libavformat opencv4)

INCLUDES += $(shell pkg-config --cflags libavcodec libavutil libavformat opencv4)

# Target rules
all: build

build: video

NvDecoder.o: NvCodec/NvDecoder/NvDecoder.cpp NvCodec/NvDecoder/NvDecoder.h
	$(GCC) -O3 $(CCFLAGS) $(INCLUDES) -o $@ -c $<

ColorSpace.o: Utils/ColorSpace.cu
	$(NVCC)  -O3 $(NVCCFLAGS) $(INCLUDES) -o $@ -c $<

video.o: VideoCapture.cpp Utils/FFmpegDemuxer.h \
                       NvCodec/NvDecoder/NvDecoder.h Utils/NvCodecUtils.h \
                       Utils/Logger.h
	$(GCC) -O3 $(CCFLAGS) $(INCLUDES) -o $@ -c $<

video: video.o ColorSpace.o NvDecoder.o
	$(GCC) -O3 $(CCFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -rf video video.o ColorSpace.o NvDecoder.o
