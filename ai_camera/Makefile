# ============================================================
#  RV1126B Smart Camera — Makefile
# ============================================================

# 交叉编译器
CC  = aarch64-none-linux-gnu-gcc
CXX = aarch64-none-linux-gnu-g++

# 生成的可执行文件
TARGET = rtsp_test

# 源文件 (纯 C)
SRCS = main.c capture.c rtsp_service.c cv_branch.c gst_pipeline.c
OBJS = $(SRCS:.c=.o)

# ============================================================
# GStreamer 1.0 配置
# ============================================================
# 方式1: pkg-config (推荐, 需在 sysroot 安装 gstreamer-1.0 gstreamer-app-1.0)
GST_CFLAGS  = $(shell aarch64-none-linux-gnu-pkg-config --cflags gstreamer-1.0 gstreamer-app-1.0 2>/dev/null)
GST_LIBS    = $(shell aarch64-none-linux-gnu-pkg-config --libs   gstreamer-1.0 gstreamer-app-1.0 2>/dev/null)

# 方式2: 手动路径 (如果 pkg-config 不可用, 取消注释并修改)
# GST_ROOT    = /path/to/rv1126/sysroot/usr
# GST_CFLAGS  = -I$(GST_ROOT)/include/gstreamer-1.0 -I$(GST_ROOT)/include/glib-2.0 -I$(GST_ROOT)/lib/glib-2.0/include
# GST_LIBS    = -L$(GST_ROOT)/lib -lgstreamer-1.0 -lgstapp-1.0 -lgobject-2.0 -lglib-2.0

# ============================================================
# OpenCV 4.x 配置 (可选, 定义 CV_BRANCH_HAS_OPENCV 启用)
# ============================================================
# OPENCV_CFLAGS = $(shell aarch64-none-linux-gnu-pkg-config --cflags opencv4 2>/dev/null)
# OPENCV_LIBS   = $(shell aarch64-none-linux-gnu-pkg-config --libs   opencv4 2>/dev/null)
# OPENCV_DEFS   = -DCV_BRANCH_HAS_OPENCV

OPENCV_CFLAGS =
OPENCV_LIBS   =
OPENCV_DEFS   =

# ============================================================
# 编译选项
# ============================================================
CFLAGS  = -Wall -O2 -std=gnu11 -pthread $(GST_CFLAGS) $(OPENCV_CFLAGS) $(OPENCV_DEFS)
LDFLAGS = -pthread $(GST_LIBS) $(OPENCV_LIBS)

# 当启用 OpenCV 时，cv_branch.c 需要 C++ 编译
# 简单做法：整个项目用 CXX 链接
ifeq ($(strip $(OPENCV_LIBS)),)
  # 纯 C 链接
  LINKER = $(CC)
else
  # C++ 链接 (OpenCV 需要)
  LINKER = $(CXX)
endif

# ============================================================
# 规则
# ============================================================
all: $(TARGET)

$(TARGET): $(OBJS)
	$(LINKER) $(CFLAGS) $(OBJS) -o $(TARGET) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
