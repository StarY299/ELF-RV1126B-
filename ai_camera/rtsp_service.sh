#!/bin/sh

echo "===== RTSP Pipeline Starting ====="

# 检查视频设备
if [ ! -e /dev/video52 ]; then
    echo "ERROR: /dev/video52 not found"
    exit 1
fi

echo "[Pipeline] Starting GStreamer..."

# 低延迟管线：禁用 queue 时间/字节缓冲，leaky 防止上游阻塞
# 硬件(mppjpegdec → mpph264enc) + 软件(videoconvert)
exec gst-launch-1.0 -e v4l2src device=/dev/video52 ! \
    image/jpeg,width=1920,height=1080,framerate=30/1 ! \
    queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream ! \
    mppjpegdec ! \
    queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream ! \
    videoconvert ! \
    queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream ! \
    mpph264enc rc-mode=cbr bps=4000000 ! \
    queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream ! \
    h264parse ! \
    rtph264pay pt=96 config-interval=1 ! \
    udpsink host=127.0.0.1 port=5000 sync=false \
    > /userdata/gst.log 2>&1
    
