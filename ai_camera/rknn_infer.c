/**
 * rknn_infer.c — RKNN NPU YOLOv8 推理实现
 *
 * 管线:
 *   BGR图像 → OpenCV resize+cvtColor (NEON SIMD) → NPU推理
 *   → 解码输出 → NMS → 缩放回原图坐标 → 返回检测结果
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rga/rga.h>
#include <rga/im2d.h>
#include "rknn_api.h"
#include "rknn_infer.h"

#ifdef __cplusplus
using namespace cv;
#endif

/* ============================================================
 *  配置
 * ============================================================ */
#define MODEL_INPUT_W      640
#define MODEL_INPUT_H      640
#define CONF_THRESHOLD     0.60f   /* 类别置信度阈值 */
#define SCORE_SUM_THRESH   0.10f   /* score_sum预滤 (INT8量化值偏小) */
#define NMS_THRESHOLD      0.45f
#define MAX_DETECTIONS     20
// NUM_CLASSES 从模型输出形状自动检测, 不再硬编码

/* ============================================================
 *  全局状态
 * ============================================================ */
static struct {
    rknn_context  ctx;
    int           initialized;

    // 输入输出属性
    rknn_tensor_attr  input_attr;
    rknn_tensor_attr  output_attrs[12]; // 最多12输出 (RKOPT 3×3 + reserve)
    int               n_outputs;

    // 输入 buffer 和格式
    uint8_t      *input_buf;
    size_t        input_size;
    int           input_is_fp16;    // 1=FP16模型, 需LUT转换; 0=INT8/UINT8模型, 直接memcpy
    uint16_t      fp16_lut[256];    // uint8→FP16 预计算查表 (÷255归一化)

    // letterbox 参数 (等比缩放+灰边, 用于还原坐标)
    float         letterbox_scale;   // min(MODEL_W/img_w, MODEL_H/img_h)
    int            pad_left;
    int            pad_top;

    // 耗时统计
    int64_t       last_elapsed;
} g_rknn;

/* ============================================================
 *  辅助: 时间
 * ============================================================ */
static int64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}

/* ============================================================
 *  辅助: NMS (交并比)
 * ============================================================ */
typedef struct {
    float cx, cy, w, h;
    float conf;
    int   class_id;
} raw_box_t;

static float iou(const raw_box_t *a, const raw_box_t *b)
{
    float ax1 = a->cx - a->w / 2, ay1 = a->cy - a->h / 2;
    float ax2 = a->cx + a->w / 2, ay2 = a->cy + a->h / 2;
    float bx1 = b->cx - b->w / 2, by1 = b->cy - b->h / 2;
    float bx2 = b->cx + b->w / 2, by2 = b->cy + b->h / 2;

    float inter_x1 = fmaxf(ax1, bx1);
    float inter_y1 = fmaxf(ay1, by1);
    float inter_x2 = fminf(ax2, bx2);
    float inter_y2 = fminf(ay2, by2);
    if (inter_x2 <= inter_x1 || inter_y2 <= inter_y1) return 0.0f;

    float inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1);
    float area_a = (ax2 - ax1) * (ay2 - ay1);
    float area_b = (bx2 - bx1) * (by2 - by1);
    return inter_area / (area_a + area_b - inter_area + 1e-6f);
}

static int cmp_conf_desc(const void *a, const void *b) {
    float fa = ((const raw_box_t *)a)->conf;
    float fb = ((const raw_box_t *)b)->conf;
    return (fa < fb) ? 1 : ((fa > fb) ? -1 : 0);
}

static int nms(raw_box_t *boxes, int n, float threshold)
{
    qsort(boxes, n, sizeof(raw_box_t), cmp_conf_desc);

    /* 预选 top-200 防止NMS处理过多候选 */
    if (n > 200) n = 200;

    int keep_count = 0;
    int keep[200] = {0};

    for (int i = 0; i < n; i++) {
        int suppressed = 0;
        for (int j = 0; j < keep_count; j++) {
            if (boxes[i].class_id != boxes[keep[j]].class_id) continue; /* 同类才抑制 */
            if (iou(&boxes[i], &boxes[keep[j]]) > threshold) {
                suppressed = 1;
                break;
            }
        }
        if (!suppressed) {
            keep[keep_count++] = i;
            if (keep_count >= MAX_DETECTIONS) break;
        }
    }

    // 原地重排: 保留的移到前面
    for (int i = 0; i < keep_count; i++) {
        if (keep[i] != i) {
            raw_box_t tmp = boxes[i];
            boxes[i] = boxes[keep[i]];
            boxes[keep[i]] = tmp;
            // 更新后续索引
            for (int j = i + 1; j < keep_count; j++) {
                if (keep[j] == i) keep[j] = keep[i];
            }
        }
    }
    return keep_count;
}

/* ============================================================
 *  辅助: IEEE 754 float32 → float16
 * ============================================================ */
static inline uint16_t float_to_fp16(float f)
{
    uint32_t x = *(uint32_t *)&f;
    uint32_t sign = (x >> 16) & 0x8000;
    int32_t  exp  = ((int32_t)(x >> 23) & 0xff) - 127 + 15;
    uint32_t mant = (x >> 13) & 0x3ff;
    if (exp <= 0)  return (uint16_t)sign;           // 下溢→0
    if (exp >= 31) return (uint16_t)(sign | 0x7c00); // 上溢→Inf
    return (uint16_t)(sign | ((uint32_t)exp << 10) | mant);
}

/* ============================================================
 *  预处理: 直接缩放到 640x640 (与 PC OpenCV DNN 一致)
 * ============================================================ */
static void preprocess(const uint8_t *bgr, int img_w, int img_h)
{
    /* 始终输出 uint8 RGB 到 input_buf.
     * 归一化 (÷255) 和格式转换 (uint8→FP16/INT8) 由驱动 pass_through=0 完成,
     * 与官方 rknn_yolov5_demo 一致. */
#ifdef __cplusplus
    Mat src(img_h, img_w, CV_8UC3, (void*)bgr);

    // 静态分配, 避免每帧 malloc 2.4MB
    static Mat resized(MODEL_INPUT_H, MODEL_INPUT_W, CV_8UC3);
    static Mat rgb(MODEL_INPUT_H, MODEL_INPUT_W, CV_8UC3);

	    float scale = std::min((float)MODEL_INPUT_W / img_w, (float)MODEL_INPUT_H / img_h);
	    int new_w = (int)(img_w * scale), new_h = (int)(img_h * scale);
	    int pad_x = (MODEL_INPUT_W - new_w) / 2, pad_y = (MODEL_INPUT_H - new_h) / 2;
	    g_rknn.letterbox_scale = scale; g_rknn.pad_left = pad_x; g_rknn.pad_top = pad_y;

	    /* RGA 硬件 resize (替代 cv::resize, 节省 ~4ms) */
	    Mat resized_tmp(new_h, new_w, CV_8UC3);
	    rga_buffer_t rga_src = wrapbuffer_virtualaddr((void*)src.data, img_w, img_h, RK_FORMAT_BGR_888);
	    rga_buffer_t rga_dst = wrapbuffer_virtualaddr((void*)resized_tmp.data, new_w, new_h, RK_FORMAT_BGR_888);
	    imresize(rga_src, rga_dst);

	    copyMakeBorder(resized_tmp, resized, pad_y, pad_y, pad_x, pad_x,
	                   BORDER_CONSTANT, Scalar(114, 114, 114));
    cvtColor(resized, rgb, COLOR_BGR2RGB);

    if (g_rknn.input_is_fp16) {
        // FP16 模型: LUT 查表 uint8→FP16 (含 ÷255 归一化), pass_through=1 跳过驱动转换
        uint16_t *dst = (uint16_t *)g_rknn.input_buf;
        const uint8_t *s = rgb.data;
        int n = MODEL_INPUT_W * MODEL_INPUT_H * 3;
        for (int i = 0; i < n; i++) dst[i] = g_rknn.fp16_lut[s[i]];
    } else {
        // INT8/UINT8 模型: 直接传 uint8, pass_through=0 让驱动处理
        memcpy(g_rknn.input_buf, rgb.data, g_rknn.input_size);
    }
#else
    // C 回退 (编译兼容, 实际不用)
    for (int y = 0; y < MODEL_INPUT_H; y++) {
        int src_y = y * img_h / MODEL_INPUT_H;
        for (int x = 0; x < MODEL_INPUT_W; x++) {
            int src_x = x * img_w / MODEL_INPUT_W;
            int si = (src_y * img_w + src_x) * 3;
            int di = (y * MODEL_INPUT_W + x) * 3;
            if (g_rknn.input_is_fp16) {
                uint16_t *dst = (uint16_t *)g_rknn.input_buf;
                dst[di+0] = g_rknn.fp16_lut[bgr[si+2]];  // B→R
                dst[di+1] = g_rknn.fp16_lut[bgr[si+1]];  // G→G
                dst[di+2] = g_rknn.fp16_lut[bgr[si+0]];  // R→B
            } else {
                g_rknn.input_buf[di+0] = bgr[si+2];
                g_rknn.input_buf[di+1] = bgr[si+1];
                g_rknn.input_buf[di+2] = bgr[si+0];
            }
        }
    }
#endif
}

/* ============================================================
 *  YOLOv8 后处理: 解码 + NMS + 坐标还原
 *
 *  YOLOv8 输出格式 (以 80 类为例):
 *    单输出: [1, 84, 8400]  其中 84 = 4(bbox) + 80(cls)
 *    多输出: 3 个尺度 (同 YOLOv5)
 *
 *  解码: boxes 已是归一化的 cx,cy,w,h (相对于 640x640), 直接可用
 * ============================================================ */
/* ============================================================
 *  坐标还原: 640x640 stretch → 原始图像坐标
 * ============================================================ */
static void rescale_boxes(raw_box_t *boxes, int n, int img_w, int img_h)
{
    float scale = g_rknn.letterbox_scale;
    float pad_x = (float)g_rknn.pad_left;
    float pad_y = (float)g_rknn.pad_top;

    for (int i = 0; i < n; i++) {
        // letterbox 逆映射: (model_coord - pad) / scale → 原图坐标
        float cx = (boxes[i].cx - pad_x) / scale;
        float cy = (boxes[i].cy - pad_y) / scale;
        float w  = boxes[i].w / scale;
        float h  = boxes[i].h / scale;

        boxes[i].cx = cx - w / 2;
        boxes[i].cy = cy - h / 2;
        boxes[i].w  = w;
        boxes[i].h  = h;

        if (boxes[i].cx < 0) { boxes[i].w += boxes[i].cx; boxes[i].cx = 0; }
        if (boxes[i].cy < 0) { boxes[i].h += boxes[i].cy; boxes[i].cy = 0; }
        if (boxes[i].cx + boxes[i].w > img_w) boxes[i].w = img_w - boxes[i].cx;
        if (boxes[i].cy + boxes[i].h > img_h) boxes[i].h = img_h - boxes[i].cy;
    }
}

/* ============================================================
 *  API
 * ============================================================ */

int rknn_infer_init(const char *model_path)
{
    memset(&g_rknn, 0, sizeof(g_rknn));

    /* ---- 加载模型 ---- */
    FILE *fp = fopen(model_path, "rb");
    if (!fp) {
        fprintf(stderr, "[RKNN] Failed to open %s\n", model_path);
        return -1;
    }
    fseek(fp, 0, SEEK_END);
    int model_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    unsigned char *model_data = (unsigned char *)malloc(model_size);
    if (!model_data) {
        fprintf(stderr, "[RKNN] malloc model failed\n");
        fclose(fp);
        return -1;
    }
    fread(model_data, 1, model_size, fp);
    fclose(fp);

    fprintf(stderr, "[RKNN] calling rknn_init (size=%d)...\n", model_size);
    int ret = rknn_init(&g_rknn.ctx, model_data, model_size, 0, NULL);
    free(model_data);
    fprintf(stderr, "[RKNN] rknn_init returned %d\n", ret);
    if (ret < 0) {
        fprintf(stderr, "[RKNN] rknn_init failed: %d\n", ret);
        return -1;
    }

    /* ---- 查询 SDK 版本 ---- */
    rknn_sdk_version ver;
    ret = rknn_query(g_rknn.ctx, RKNN_QUERY_SDK_VERSION, &ver, sizeof(ver));
    if (ret >= 0) {
        printf("[RKNN] SDK: api=%s driver=%s\n", ver.api_version, ver.drv_version);
    }

    /* ---- 查询输入输出数量 ---- */
    rknn_input_output_num io_num;
    ret = rknn_query(g_rknn.ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret < 0) {
        fprintf(stderr, "[RKNN] query in/out num failed\n");
        rknn_destroy(g_rknn.ctx);
        return -1;
    }
    printf("[RKNN] inputs=%d outputs=%d\n", io_num.n_input, io_num.n_output);

    /* ---- 查询输入属性 ---- */
    g_rknn.input_attr.index = 0;
    ret = rknn_query(g_rknn.ctx, RKNN_QUERY_INPUT_ATTR,
                     &g_rknn.input_attr, sizeof(g_rknn.input_attr));
    if (ret < 0) {
        fprintf(stderr, "[RKNN] query input attr failed\n");
        rknn_destroy(g_rknn.ctx);
        return -1;
    }
    printf("[RKNN] input: dims=[%d", g_rknn.input_attr.dims[0]);
    for (int i = 1; i < g_rknn.input_attr.n_dims; i++)
        printf(", %d", g_rknn.input_attr.dims[i]);
    printf("] size=%d fmt=%d type=%s (model native format)\n",
           g_rknn.input_attr.size, g_rknn.input_attr.fmt,
           get_type_string((rknn_tensor_type)g_rknn.input_attr.type));

    // 检测输入格式: FP16 用LUT+pass_through=1, INT8/UINT8 用原始uint8+pass_through=0
    g_rknn.input_is_fp16 = (g_rknn.input_attr.type == RKNN_TENSOR_FLOAT16) ? 1 : 0;

    if (g_rknn.input_is_fp16) {
        printf("[RKNN] input: FP16 model → LUT uint8→FP16, pass_through=1\n");
        // 预计算 FP16 查表: lut[i] = fp16(i / 255.0)
        for (int i = 0; i < 256; i++)
            g_rknn.fp16_lut[i] = float_to_fp16((float)i / 255.0f);
    } else {
        printf("[RKNN] input: INT8/UINT8 model → raw uint8, pass_through=0\n");
    }

    /* ---- 查询输出属性 ---- */
    g_rknn.n_outputs = io_num.n_output;
    for (int i = 0; i < g_rknn.n_outputs; i++) {
        g_rknn.output_attrs[i].index = i;
        rknn_query(g_rknn.ctx, RKNN_QUERY_OUTPUT_ATTR,
                   &g_rknn.output_attrs[i], sizeof(rknn_tensor_attr));
        printf("[RKNN] output %d: dims=[%d", i, g_rknn.output_attrs[i].dims[0]);
        for (int d = 1; d < g_rknn.output_attrs[i].n_dims; d++)
            printf(", %d", g_rknn.output_attrs[i].dims[d]);
        printf("] size=%d fmt=%d type=%d qnt=%d zp=%d scale=%.6f\n",
               g_rknn.output_attrs[i].size,
               g_rknn.output_attrs[i].fmt,
               g_rknn.output_attrs[i].type,
               g_rknn.output_attrs[i].qnt_type,
               g_rknn.output_attrs[i].zp,
               g_rknn.output_attrs[i].scale);
    }

    /* ---- 分配输入缓冲区 ---- */
    if (g_rknn.input_is_fp16)
        g_rknn.input_size = MODEL_INPUT_W * MODEL_INPUT_H * 3 * sizeof(uint16_t); // FP16: 2.46MB
    else
        g_rknn.input_size = MODEL_INPUT_W * MODEL_INPUT_H * 3;                     // uint8: 1.2MB
    g_rknn.input_buf = (uint8_t *)malloc(g_rknn.input_size);
    if (!g_rknn.input_buf) {
        fprintf(stderr, "[RKNN] malloc input buf failed\n");
        rknn_destroy(g_rknn.ctx);
        return -1;
    }
    printf("[RKNN] input buf: %zu bytes (%s)\n", g_rknn.input_size,
           g_rknn.input_is_fp16 ? "fp16" : "uint8");

    g_rknn.initialized = 1;
    printf("[RKNN] Init OK (model=%s)\n", model_path);
    return 0;
}

int rknn_infer_run(const uint8_t *bgr_data, int img_w, int img_h,
                    rknn_result_t *result)
{
    if (!g_rknn.initialized || !bgr_data || !result) return -1;

    memset(result, 0, sizeof(*result));
    int64_t t0 = now_us(), t1, t2, t3, t4;

    /* ---- 1. 预处理 ---- */
    preprocess(bgr_data, img_w, img_h);
    t1 = now_us();

    /* ---- 2. 设置输入 ---- */
    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index      = 0;
    inputs[0].fmt        = RKNN_TENSOR_NHWC;
    inputs[0].buf        = g_rknn.input_buf;
    inputs[0].size       = g_rknn.input_size;

    if (g_rknn.input_is_fp16) {
        // FP16: LUT已归一化(÷255), pass_through=1 跳过驱动转换
        inputs[0].type         = RKNN_TENSOR_FLOAT16;
        inputs[0].pass_through = 1;
    } else {
        // INT8/UINT8: 原始uint8, pass_through=0 驱动处理归一化+量化
        inputs[0].type         = RKNN_TENSOR_UINT8;
        inputs[0].pass_through = 0;
    }

    int ret = rknn_inputs_set(g_rknn.ctx, 1, inputs);
    if (ret < 0) {
        fprintf(stderr, "[RKNN] inputs_set failed: %d\n", ret);
        return -1;
    }
    int64_t t_set = now_us();

    /* ---- 3. 推理 ---- */
    ret = rknn_run(g_rknn.ctx, NULL);
    if (ret < 0) {
        fprintf(stderr, "[RKNN] run failed: %d\n", ret);
        return -1;
    }
    t2 = now_us();

    /* ---- 4. 获取输出 ---- */
    rknn_output outputs[g_rknn.n_outputs];
    memset(outputs, 0, sizeof(outputs));
    for (int i = 0; i < g_rknn.n_outputs; i++) {
        outputs[i].want_float = 1;
        outputs[i].index      = i;
        outputs[i].is_prealloc = 0;
    }
    ret = rknn_outputs_get(g_rknn.ctx, g_rknn.n_outputs, outputs, NULL);
    if (ret < 0) {
        fprintf(stderr, "[RKNN] outputs_get failed: %d\n", ret);
        return -1;
    }

    /* ---- 5. 后处理 (RKOPT: 9路输出, DFL box解码) ---- */
    t3 = now_us();

    /* 检测标准 vs RKOPT 格式 */
    int is_rkopt = (g_rknn.n_outputs == 9 || g_rknn.n_outputs == 3);

    raw_box_t *candidates = (raw_box_t *)malloc(MAX_DETECTIONS * 100 * sizeof(raw_box_t));
    int n_cand = 0;
    int num_classes = 4;  /* no_smoking, smoking, fire, smoke */

    if (is_rkopt) {
        /* RKOPT YOLOv8: box [1,64,H,W] + cls [1,4,H,W] + score [1,1,H,W] × 3 scales */
        struct { int h, w, stride; } scales[] = {{80,80,8}, {40,40,16}, {20,20,32}};
        int nc3 = num_classes;  /* number of class channels in cls tensor */
        static int rkopt_printed = 0;
        if (!rkopt_printed) {
            printf("[RKNN] RKOPT mode: %d scales, %d classes\n",
                   (int)(sizeof(scales)/sizeof(scales[0])), num_classes);
            rkopt_printed = 1;
        }

        for (int s = 0; s < 3 && s * 3 + 2 < g_rknn.n_outputs; s++) {
            int base = s * 3;
            float *box_data   = (float *)outputs[base + 0].buf;
            float *cls_data   = (float *)outputs[base + 1].buf;
            float *score_data = (float *)outputs[base + 2].buf;

            int grid_h = scales[s].h, grid_w = scales[s].w;
            int stride = scales[s].stride;
            int grid_len = grid_h * grid_w;
            const int DFL_LEN = 16;  /* 4 coords × 16 bins = 64 box channels */

            for (int y = 0; y < grid_h; y++) {
                for (int x = 0; x < grid_w; x++) {
                    int off = y * grid_w + x;

                    /* score_sum 预筛选 (INT8模型各尺度scale差异大, 放宽) */
                    if (score_data[off] < 0.0f) continue;

                    /* 找最大 class */
                    float max_score = 0.0f;
                    int best_cls = 0;
                    for (int c = 0; c < num_classes; c++) {
                        float sc = cls_data[c * grid_len + off];
                        /* RKOPT 输出需要 sigmoid */
                        if (sc < 0 || sc > 1) sc = 1.0f / (1.0f + expf(-sc));
                        if (sc > max_score) { max_score = sc; best_cls = c; }
                    }
                    if (max_score < CONF_THRESHOLD) continue;

                    /* DFL box解码: 4 coord × 16 bins */
                    float ltrb[4];
                    for (int side = 0; side < 4; side++) {
                        float acc = 0.0f, sum_exp = 0.0f;
                        float max_v = box_data[(side * DFL_LEN) * grid_len + off];
                        for (int k = 1; k < DFL_LEN; k++) {
                            float v = box_data[(side * DFL_LEN + k) * grid_len + off];
                            if (v > max_v) max_v = v;
                        }
                        /* softmax + weighted sum */
                        for (int k = 0; k < DFL_LEN; k++) {
                            float v = box_data[(side * DFL_LEN + k) * grid_len + off];
                            float ev = expf(v - max_v);
                            sum_exp += ev;
                            acc += ev * k;
                        }
                        ltrb[side] = (sum_exp > 0) ? acc / sum_exp : 0.0f;
                    }

                    /* 坐标->像素 (DFL值为grid单位, 需×stride) */
                    float cx = (x + 0.5f) * stride;
                    float cy = (y + 0.5f) * stride;
                    float l = ltrb[0] * stride, t = ltrb[1] * stride;
                    float r = ltrb[2] * stride, b = ltrb[3] * stride;

                    float x1 = cx - l, y1 = cy - t;
                    float x2 = cx + r, y2 = cy + b;
                    float w = x2 - x1, h = y2 - y1;

                    if (w <= 15 || h <= 15) continue;  /* 过滤过小框 */

                    /* letterbox 逆映射 */
                    float scale = g_rknn.letterbox_scale;
                    float pad_x = (float)g_rknn.pad_left;
                    float pad_y = (float)g_rknn.pad_top;
                    x1 = (x1 - pad_x) / scale;
                    y1 = (y1 - pad_y) / scale;
                    x2 = (x2 - pad_x) / scale;
                    y2 = (y2 - pad_y) / scale;

                    /* 钳位 */
                    if (x1 < 0) x1 = 0; if (y1 < 0) y1 = 0;
                    if (x2 > img_w) x2 = img_w;
                    if (y2 > img_h) y2 = img_h;
                    if (x2 <= x1 || y2 <= y1) continue;

                    candidates[n_cand].cx = x1;
                    candidates[n_cand].cy = y1;
                    candidates[n_cand].w  = x2 - x1;
                    candidates[n_cand].h  = y2 - y1;
                    candidates[n_cand].conf = max_score;
                    candidates[n_cand].class_id = best_cls;
                    n_cand++;
                    if (n_cand >= MAX_DETECTIONS * 100) goto out_loop;
                }
            }
        }
out_loop: ;
    } else {
        /* 标准 YOLOv8: [1, N, A] 单输出 */
        for (int o = 0; o < g_rknn.n_outputs; o++) {
            rknn_tensor_attr *attr = &g_rknn.output_attrs[o];
            uint32_t *dims = attr->dims;
            if (attr->n_dims != 3) continue;
            int N = dims[1], A = dims[2];
            if (dims[2] >= 5 && dims[2] <= 200) { N = dims[2]; A = dims[1]; }
            float *out_data = (float *)outputs[o].buf;
            int nc = N - 4;
            for (int a = 0; a < A && n_cand < MAX_DETECTIONS * 100; a++) {
                float ms = 0; int bc = 0;
                for (int c = 0; c < nc; c++) {
                    float sc = out_data[(c + 4) * A + a];
                    if (sc > ms) { ms = sc; bc = c; }
                }
                if (ms < CONF_THRESHOLD) continue;
                candidates[n_cand].cx = out_data[a];
                candidates[n_cand].cy = out_data[A + a];
                candidates[n_cand].w  = out_data[2*A + a];
                candidates[n_cand].h  = out_data[3*A + a];
                candidates[n_cand].conf = ms;
                candidates[n_cand].class_id = bc;
                n_cand++;
            }
        }
        rescale_boxes(candidates, n_cand, img_w, img_h);
    }

    /* ---- 6. NMS ---- */
    int n_keep = nms(candidates, n_cand, NMS_THRESHOLD);

    /* ---- 7. 填充结果 ---- */
    result->count = n_keep;
    result->detections = (rknn_detection_t *)calloc(n_keep, sizeof(rknn_detection_t));
    for (int i = 0; i < n_keep; i++) {
        result->detections[i].class_id   = candidates[i].class_id;
        result->detections[i].confidence = candidates[i].conf;
        result->detections[i].x = (int)candidates[i].cx;
        result->detections[i].y = (int)candidates[i].cy;
        result->detections[i].w = (int)candidates[i].w;
        result->detections[i].h = (int)candidates[i].h;
        static const char *names[] = {"no_smoking", "smoking", "fire", "smoke"};
        int cid = candidates[i].class_id;
        snprintf(result->detections[i].label, sizeof(result->detections[i].label),
                 "%s", (cid >= 0 && cid < 4) ? names[cid] : "?");
    }

    t4 = now_us();
    free(candidates);

    /* ---- 8. 释放输出 ---- */
    rknn_outputs_release(g_rknn.ctx, g_rknn.n_outputs, outputs);

    g_rknn.last_elapsed = now_us() - t0;
    result->elapsed_us  = g_rknn.last_elapsed;

    /* 每100帧打印 rknn 内部分布 */
    static int rknn_cnt = 0;
    if (++rknn_cnt % 100 == 0) {
        printf("[RKNN-PERF] pre=%lldms set=%lldms run=%lldms out=%lldms post=%lldms total=%lldms\n",
               (long long)(t1-t0)/1000, (long long)(t_set-t1)/1000,
               (long long)(t2-t_set)/1000, (long long)(t3-t2)/1000,
               (long long)(t4-t3)/1000, (long long)(t4-t0)/1000);
    }

    return 0;
}

void rknn_infer_deinit(void)
{
    if (!g_rknn.initialized) return;

    printf("[RKNN] Deinitializing...\n");
    if (g_rknn.input_buf) { free(g_rknn.input_buf); g_rknn.input_buf = NULL; }
    rknn_destroy(g_rknn.ctx);
    g_rknn.initialized = 0;
    printf("[RKNN] Deinit done\n");
}
