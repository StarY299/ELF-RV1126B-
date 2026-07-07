#ifndef RKNN_INFER_H
#define RKNN_INFER_H

#include <stdint.h>

/* ============================================================
 *  rknn_infer — RKNN NPU YOLOv8 推理模块
 *
 *  流程:
 *    加载 .rknn 模型 → BGR预处理(letterbox+归一化) → NPU推理 → 后处理(NMS) → 输出检测框
 *
 *  用法:
 *    rknn_infer_init("/userdata/yolov8n.rknn");
 *    rknn_infer_run(bgr_data, width, height, &results);
 *    rknn_infer_deinit();
 * ============================================================ */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 检测框 ---- */
typedef struct {
    int     class_id;
    float   confidence;
    int     x, y, w, h;
    char    label[64];
} rknn_detection_t;

typedef struct {
    rknn_detection_t *detections;
    int               count;
    int64_t           elapsed_us;
} rknn_result_t;

/* ---- API ---- */
int  rknn_infer_init(const char *model_path);
int  rknn_infer_run(const uint8_t *bgr_data, int img_w, int img_h,
                    rknn_result_t *result);
void rknn_infer_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // RKNN_INFER_H
