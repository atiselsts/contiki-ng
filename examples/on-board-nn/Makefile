CONTIKI_PROJECT = node
all: $(CONTIKI_PROJECT)

OPTIMIZATIONS=-Os

PROJECT_SOURCEFILES += nn.cpp

TENSORFLOW=/home/atis/source/tensorflow
CFLAGS += -I$(TENSORFLOW)

CXXFLAGS += -DTF_LITE_STATIC_MEMORY -DTF_LITE_MCU_DEBUG_LOG
CXXFLAGS += -I/home/atis/source/tensorflow/tensorflow/lite/micro/tools/make/downloads/flatbuffers/include

LDFLAGS += -L.
TARGET_LIBFILES += -ltensorflow-microlite -lm

CONTIKI = ../..
include $(CONTIKI)/Makefile.include

