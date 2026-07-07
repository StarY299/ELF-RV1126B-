/**
 * gst_pipeline.c — GStreamer appsrc 推流管线实现
 *
 * 等价于原 shell 脚本:
 *   v4l2src → mppjpegdec → videoconvert → mpph264enc
 *           → h264parse → rtph264pay → udpsink
 *
 * 但 v4l2src 替换为 appsrc, 由外部统一喂帧。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include "rtsp_stream.h"

typedef struct {
    GstElement *pipeline, *appsrc, *jpegdec, *tee;
    GstElement *q_rtsp, *videoconvert, *h264enc;
    GstElement *parser, *payloader, *sink;
    GstBus     *bus;
    int         initialized, width, height, fps;
} PipeCtx;

static PipeCtx g_pipe;

/* ============================================================
 *  内部: 看门狗消息处理
 * ============================================================ */
static gboolean bus_watch(GstBus *bus, GstMessage *msg, gpointer data)
{
    (void)bus;
    (void)data;

    switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
        GError *err = NULL;
        gchar *dbg  = NULL;
        gst_message_parse_error(msg, &err, &dbg);
        fprintf(stderr, "[GST] ERROR: %s\n", err->message);
        if (dbg) fprintf(stderr, "[GST]  debug: %s\n", dbg);
        g_error_free(err);
        g_free(dbg);
        break;
    }
    case GST_MESSAGE_WARNING: {
        GError *err = NULL;
        gchar *dbg  = NULL;
        gst_message_parse_warning(msg, &err, &dbg);
        fprintf(stderr, "[GST] WARNING: %s\n", err->message);
        g_error_free(err);
        g_free(dbg);
        break;
    }
    case GST_MESSAGE_EOS:
        fprintf(stderr, "[GST] EOS received\n");
        break;
    case GST_MESSAGE_STATE_CHANGED: {
        GstState old, now, pending_state;
        gst_message_parse_state_changed(msg, &old, &now, &pending_state);
        if (GST_MESSAGE_SRC(msg) == GST_OBJECT(g_pipe.pipeline)) {
            printf("[GST] state: %s → %s\n",
                   gst_element_state_get_name(old),
                   gst_element_state_get_name(now));
        }
        break;
    }
    default:
        break;
    }
    return TRUE;
}

/* ============================================================
 *  API
 * ============================================================ */

int gst_pipeline_init(int width, int height, int fps, int bitrate)
{
    if (g_pipe.initialized) {
        fprintf(stderr, "[GST] Already initialized\n");
        return -1;
    }
    memset(&g_pipe, 0, sizeof(g_pipe));
    g_pipe.width  = width;
    g_pipe.height = height;
    g_pipe.fps    = fps;

    /* ---- 初始化 GStreamer ---- */
    GError *err = NULL;
    if (!gst_init_check(NULL, NULL, &err)) {
        fprintf(stderr, "[GST] gst_init_check: %s\n",
                err ? err->message : "unknown");
        if (err) g_error_free(err);
        return -1;
    }

    /* ---- 创建元素 ---- */
    g_pipe.appsrc      = gst_element_factory_make("appsrc",       "src");
    g_pipe.jpegdec     = gst_element_factory_make("mppjpegdec",   "jpegdec");
    g_pipe.tee         = gst_element_factory_make("tee",          "tee");
    g_pipe.q_rtsp      = gst_element_factory_make("queue",        "q_rtsp");
    g_pipe.videoconvert= gst_element_factory_make("videoconvert", "conv");
    g_pipe.h264enc     = gst_element_factory_make("mpph264enc",   "h264enc");
    g_pipe.parser      = gst_element_factory_make("h264parse",    "parser");
    g_pipe.payloader   = gst_element_factory_make("rtph264pay",   "pay");
    g_pipe.sink        = gst_element_factory_make("udpsink",      "sink");

    GstElement *elems[] = {
        g_pipe.appsrc, g_pipe.jpegdec, g_pipe.tee, g_pipe.q_rtsp,
        g_pipe.videoconvert, g_pipe.h264enc, g_pipe.parser,
        g_pipe.payloader, g_pipe.sink
    };
    const char *names[] = {
        "appsrc","mppjpegdec","tee","q_rtsp","videoconvert",
        "mpph264enc","h264parse","rtph264pay","udpsink"
    };
    for (int i = 0; i < 9; i++) {
        if (!elems[i]) {
            fprintf(stderr, "[GST] Failed to create element: %s\n", names[i]);
            gst_pipeline_deinit();
            return -1;
        }
    }

    /* ---- 配置 appsrc ---- */
    GstCaps *caps = gst_caps_new_simple("image/jpeg",
        "width",       G_TYPE_INT,    width,
        "height",      G_TYPE_INT,    height,
        "framerate",   GST_TYPE_FRACTION, fps, 1,
        "parsed",      G_TYPE_BOOLEAN, TRUE,
        NULL);
    g_object_set(G_OBJECT(g_pipe.appsrc),
        "caps",             caps,
        "format",           GST_FORMAT_TIME,
        "is-live",          TRUE,
        "do-timestamp",     TRUE,
        "max-bytes",        (guint64)0,       // 不限制
        "block",            FALSE,            // 非阻塞 push, 满则丢弃
        NULL);
    gst_caps_unref(caps);

    /* ---- 配置 h264enc ---- */
    if (bitrate > 0) {
        g_object_set(G_OBJECT(g_pipe.h264enc),
            "rc-mode", 1,     // CBR
            "bps",     bitrate,
            NULL);
    } else {
        g_object_set(G_OBJECT(g_pipe.h264enc),
            "rc-mode", 1,
            "bps",     4000000,
            NULL);
    }

    /* ---- 配置 rtph264pay ---- */
    g_object_set(G_OBJECT(g_pipe.payloader),
        "pt",             96,
        "config-interval", 1,
        NULL);

    /* ---- 配置 udpsink ---- */
    g_object_set(G_OBJECT(g_pipe.sink),
        "host",  "127.0.0.1",
        "port",  5000,
        "sync",  FALSE,
        NULL);

    /* ---- 构建管线 (appsrc → jpegdec → videoconvert → h264enc → udpsink) ---- */
    g_pipe.pipeline = gst_pipeline_new("rtsp-pipeline");
    gst_bin_add_many(GST_BIN(g_pipe.pipeline),
        g_pipe.appsrc,g_pipe.jpegdec,g_pipe.tee,
        g_pipe.q_rtsp,g_pipe.videoconvert,
        g_pipe.h264enc,g_pipe.parser,g_pipe.payloader,
        g_pipe.sink,NULL);

    gst_element_link_many(g_pipe.appsrc,g_pipe.jpegdec,g_pipe.tee,NULL);
    gst_element_link_many(g_pipe.q_rtsp,g_pipe.videoconvert,
        g_pipe.h264enc,g_pipe.parser,g_pipe.payloader,g_pipe.sink,NULL);

    GstPad *s1=gst_element_request_pad_simple(g_pipe.tee,"src_%u");
    GstPad *d1=gst_element_get_static_pad(g_pipe.q_rtsp,"sink");
    gst_pad_link(s1,d1);
    gst_object_unref(s1);gst_object_unref(d1);

    /* ---- 设置总线监听 ---- */
    g_pipe.bus = gst_pipeline_get_bus(GST_PIPELINE(g_pipe.pipeline));
    gst_bus_add_watch(g_pipe.bus, bus_watch, NULL);
    gst_object_unref(g_pipe.bus);

    /* ---- 启动管线 ---- */
    GstStateChangeReturn ret = gst_element_set_state(g_pipe.pipeline,
                                                      GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        fprintf(stderr, "[GST] Failed to start pipeline\n");
        gst_pipeline_deinit();
        return -1;
    }
    printf("[GST] Pipeline PLAYING (%dx%d, %dfps, %dbps)\n",
           width, height, fps, bitrate > 0 ? bitrate : 4000000);

    g_pipe.initialized = 1;
    return 0;
}

int gst_pipeline_push_frame(const uint8_t *data, size_t size, int64_t frame_id)
{
    if (!g_pipe.initialized || !g_pipe.appsrc) return -1;

    // 分配 buffer 并拷贝数据
    GstBuffer *buf = gst_buffer_new_allocate(NULL, size, NULL);
    if (!buf) return -1;

    GstMapInfo map;
    if (gst_buffer_map(buf, &map, GST_MAP_WRITE)) {
        memcpy(map.data, data, size);
        gst_buffer_unmap(buf, &map);
    } else {
        gst_buffer_unref(buf);
        return -1;
    }

    // 设置时间戳
    GstClockTime pts = gst_util_uint64_scale(
        (guint64)frame_id,
        GST_SECOND,
        (guint64)g_pipe.fps);
    GST_BUFFER_PTS(buf) = pts;
    GST_BUFFER_DURATION(buf) = gst_util_uint64_scale_int(
        1, GST_SECOND, g_pipe.fps);

    // 推送 (非阻塞)
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(g_pipe.appsrc), buf);

    if (ret != GST_FLOW_OK && ret != GST_FLOW_FLUSHING) {
        // 仅在非预期状态时打印
        static int err_count = 0;
        if (err_count++ < 5) {
            fprintf(stderr, "[GST] push error: %s\n",
                    gst_flow_get_name(ret));
        }
        return -1;
    }
    return 0;
}

void gst_pipeline_deinit(void)
{
    if (!g_pipe.initialized) return;

    printf("[GST] Deinitializing...\n");

    if (g_pipe.pipeline) {
        // 发送 EOS 并等待管线排空
        gst_app_src_end_of_stream(GST_APP_SRC(g_pipe.appsrc));
        gst_element_set_state(g_pipe.pipeline, GST_STATE_NULL);
        gst_object_unref(g_pipe.pipeline);
        g_pipe.pipeline = NULL;
    }

    // 注意: elements 由 pipeline 管理生命周期, 不需要单独 unref
    g_pipe.appsrc       = NULL;
    g_pipe.jpegdec      = NULL;
    g_pipe.videoconvert = NULL;
    g_pipe.h264enc      = NULL;
    g_pipe.parser       = NULL;
    g_pipe.payloader    = NULL;
    g_pipe.sink         = NULL;
    g_pipe.initialized  = 0;

    printf("[GST] Deinit done\n");
}
