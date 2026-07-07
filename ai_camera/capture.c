/**
 * capture.c — V4L2 MJPEG 采集实现
 *
 * 流程:
 *   open → query caps → set fmt → req bufs → mmap → stream on
 *   loop: dqbuf → return to caller → qbuf
 *   cleanup: stream off → munmap → close
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <linux/videodev2.h>
#include "capture.h"

/* ============================================================
 *  自动查找 USB 摄像头
 * ============================================================ */
int capture_find_device(char *out_device, size_t out_size)
{
    /* 优先 by-id 稳定路径 (直接检查存在即可, 由 capture_init 打开) */
    const char *by_id = "/dev/v4l/by-id/usb-HSK-250911-J_UNIQUESKY_CAR_CAMERA-video-index0";
    if (access(by_id, F_OK) == 0) {
        snprintf(out_device, out_size, "%s", by_id);
        return 0;
    }

    /* 扫描 /dev/video* (仅检查设备名, 不打开) */
    DIR *dir = opendir("/sys/class/video4linux");
    if (!dir) {
        /* 回退: opendir /dev */
        dir = opendir("/dev");
        if (!dir) return -1;

        struct dirent *ent;
        while ((ent = readdir(dir)) != NULL) {
            if (strncmp(ent->d_name, "video", 5) != 0) continue;
            char path[128];
            snprintf(path, sizeof(path), "/dev/%s", ent->d_name);

            int fd = open(path, O_RDWR | O_NONBLOCK);
            if (fd < 0) continue;

            struct v4l2_capability cap;
            memset(&cap, 0, sizeof(cap));
            if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0 &&
                strstr((char *)cap.card, "UNIQUESKY")) {
                close(fd);
                closedir(dir);
                snprintf(out_device, out_size, "%s", path);
                return 0;
            }
            close(fd);
        }
        closedir(dir);
        return -1;
    }

    /* 通过 sysfs 扫描 (不打开设备) */
    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL) {
        if (ent->d_name[0] == '.') continue;
        char name_path[256];
        snprintf(name_path, sizeof(name_path),
                 "/sys/class/video4linux/%s/name", ent->d_name);
        FILE *fp = fopen(name_path, "r");
        if (!fp) continue;
        char card[128] = {0};
        fgets(card, sizeof(card), fp);
        fclose(fp);
        if (strstr(card, "UNIQUESKY")) {
            char dev_path[128];
            snprintf(dev_path, sizeof(dev_path), "/dev/%s", ent->d_name);
            closedir(dir);
            snprintf(out_device, out_size, "%s", dev_path);
            return 0;
        }
    }
    closedir(dir);
    return -1;
}

/* ============================================================
 *  内部状态
 * ============================================================ */
#define NB_BUFS 4

typedef struct {
    void   *start;
    size_t  length;
} buffer_t;

static struct {
    int         fd;
    int         width;
    int         height;
    buffer_t    bufs[NB_BUFS];
    int         n_bufs;
    int64_t     frame_id;
    int         pending_idx;    // 上次 dq 但未 q 的 buf index, -1 表示无
} g_cap;

/* ============================================================
 *  API
 * ============================================================ */

int capture_init(const char *device, int width, int height, int fps)
{
    memset(&g_cap, 0, sizeof(g_cap));
    g_cap.fd = -1;
    g_cap.pending_idx = -1;

    /* ---- 1. 打开设备 ---- */
    g_cap.fd = open(device, O_RDWR);  // 阻塞模式: DQBUF 等待帧
    if (g_cap.fd < 0) {
        perror("[CAP] open");
        return -1;
    }

    /* ---- 2. 查询能力 ---- */
    struct v4l2_capability cap;
    if (ioctl(g_cap.fd, VIDIOC_QUERYCAP, &cap) < 0) {
        perror("[CAP] VIDIOC_QUERYCAP");
        goto fail;
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        fprintf(stderr, "[CAP] %s is not a video capture device\n", device);
        goto fail;
    }
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        fprintf(stderr, "[CAP] %s does not support streaming I/O\n", device);
        goto fail;
    }
    printf("[CAP] device: %s (driver: %s, card: %s)\n",
           device, cap.driver, cap.card);

    /* ---- 3. 设置格式 (MJPEG) ---- */
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = width;
    fmt.fmt.pix.height      = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field       = V4L2_FIELD_ANY;

    if (ioctl(g_cap.fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("[CAP] VIDIOC_S_FMT MJPEG");
        // 尝试 YUYV 作为回退
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        if (ioctl(g_cap.fd, VIDIOC_S_FMT, &fmt) < 0) {
            perror("[CAP] VIDIOC_S_FMT YUYV");
            goto fail;
        }
        printf("[CAP] fallback to YUYV format\n");
    }
    g_cap.width  = fmt.fmt.pix.width;
    g_cap.height = fmt.fmt.pix.height;
    printf("[CAP] format: %c%c%c%c %dx%d, stride=%d, size=%d\n",
           (char)(fmt.fmt.pix.pixelformat & 0xFF),
           (char)((fmt.fmt.pix.pixelformat >> 8) & 0xFF),
           (char)((fmt.fmt.pix.pixelformat >> 16) & 0xFF),
           (char)((fmt.fmt.pix.pixelformat >> 24) & 0xFF),
           g_cap.width, g_cap.height,
           fmt.fmt.pix.bytesperline,
           fmt.fmt.pix.sizeimage);

    /* ---- 4. 设置帧率 ---- */
    struct v4l2_streamparm parm;
    memset(&parm, 0, sizeof(parm));
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator   = 1;
    parm.parm.capture.timeperframe.denominator = fps;
    if (ioctl(g_cap.fd, VIDIOC_S_PARM, &parm) < 0) {
        perror("[CAP] VIDIOC_S_PARM (non-fatal)");
    }

    /* ---- 5. 请求缓冲区 ---- */
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count  = NB_BUFS;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(g_cap.fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("[CAP] VIDIOC_REQBUFS");
        goto fail;
    }
    g_cap.n_bufs = req.count;
    printf("[CAP] allocated %d buffers\n", g_cap.n_bufs);

    /* ---- 6. mmap 并全部入队 ---- */
    for (int i = 0; i < g_cap.n_bufs; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;

        if (ioctl(g_cap.fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("[CAP] VIDIOC_QUERYBUF");
            goto fail;
        }

        g_cap.bufs[i].length = buf.length;
        g_cap.bufs[i].start = mmap(NULL, buf.length,
                                   PROT_READ | PROT_WRITE,
                                   MAP_SHARED,
                                   g_cap.fd, buf.m.offset);
        if (g_cap.bufs[i].start == MAP_FAILED) {
            perror("[CAP] mmap");
            goto fail;
        }

        if (ioctl(g_cap.fd, VIDIOC_QBUF, &buf) < 0) {
            perror("[CAP] VIDIOC_QBUF");
            goto fail;
        }
    }

    /* ---- 7. 启动流 ---- */
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(g_cap.fd, VIDIOC_STREAMON, &type) < 0) {
        perror("[CAP] VIDIOC_STREAMON");
        goto fail;
    }

    printf("[CAP] Init OK (%dx%d)\n", g_cap.width, g_cap.height);
    return 0;

fail:
    capture_deinit();
    return -1;
}

int capture_get_frame(capture_frame_t *frame)
{
    if (g_cap.fd < 0) return -1;

    /* ---- 0. 先归还上一帧的缓冲区 ---- */
    if (g_cap.pending_idx >= 0) {
        struct v4l2_buffer qbuf;
        memset(&qbuf, 0, sizeof(qbuf));
        qbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        qbuf.memory = V4L2_MEMORY_MMAP;
        qbuf.index  = g_cap.pending_idx;
        if (ioctl(g_cap.fd, VIDIOC_QBUF, &qbuf) < 0) {
            perror("[CAP] VIDIOC_QBUF (re-enqueue)");
        }
        g_cap.pending_idx = -1;
    }

    /* ---- 1. 取新帧 ---- */
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    // 阻塞等待帧
    if (ioctl(g_cap.fd, VIDIOC_DQBUF, &buf) < 0) {
        if (errno == EAGAIN) return -2;
        perror("[CAP] VIDIOC_DQBUF");
        return -1;
    }

    // 标记该缓冲区为"使用中", 等下次 get_frame 再归还
    g_cap.pending_idx = buf.index;

    frame->data         = (uint8_t *)g_cap.bufs[buf.index].start;
    frame->size         = buf.bytesused;
    frame->timestamp_us = (int64_t)buf.timestamp.tv_sec * 1000000LL
                        + (int64_t)buf.timestamp.tv_usec;
    frame->frame_id     = g_cap.frame_id++;

    return 0;
}

void capture_deinit(void)
{
    if (g_cap.fd < 0) return;

    printf("[CAP] Deinitializing...\n");

    // 归还可能挂起的缓冲区
    if (g_cap.pending_idx >= 0) {
        struct v4l2_buffer qbuf;
        memset(&qbuf, 0, sizeof(qbuf));
        qbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        qbuf.memory = V4L2_MEMORY_MMAP;
        qbuf.index  = g_cap.pending_idx;
        ioctl(g_cap.fd, VIDIOC_QBUF, &qbuf);
        g_cap.pending_idx = -1;
    }

    // 停止流
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(g_cap.fd, VIDIOC_STREAMOFF, &type);

    // 释放 mmap
    for (int i = 0; i < g_cap.n_bufs; i++) {
        if (g_cap.bufs[i].start && g_cap.bufs[i].start != MAP_FAILED) {
            munmap(g_cap.bufs[i].start, g_cap.bufs[i].length);
        }
    }

    close(g_cap.fd);
    g_cap.fd = -1;

    printf("[CAP] Deinit done. frames captured=%lld\n",
           (long long)g_cap.frame_id);
}
