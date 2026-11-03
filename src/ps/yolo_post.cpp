#include "yolo_post.h"
#include <math.h>

static const float YP_ANCHORS[6][2] = {
    { 10.0f,  14.0f},
    { 23.0f,  27.0f},
    { 37.0f,  58.0f},
    { 81.0f,  82.0f},
    {135.0f, 169.0f},
    {344.0f, 319.0f}
};

static const int YP_MASK[2][3] = { {3, 4, 5}, {1, 2, 3} };

static inline float yp_sigmoid(float x)
{
    return 1.0f / (1.0f + expf(-x));
}

static float yp_iou(const float* a, const float* b)
{
    float ix1 = a[0] > b[0] ? a[0] : b[0];
    float iy1 = a[1] > b[1] ? a[1] : b[1];
    float ix2 = a[2] < b[2] ? a[2] : b[2];
    float iy2 = a[3] < b[3] ? a[3] : b[3];

    float iw = ix2 - ix1;
    float ih = iy2 - iy1;
    if (iw <= 0.0f || ih <= 0.0f)
        return 0.0f;

    float inter = iw * ih;
    float area_a = (a[2] - a[0]) * (a[3] - a[1]);
    float area_b = (b[2] - b[0]) * (b[3] - b[1]);
    float uni = area_a + area_b - inter;

    return uni > 0.0f ? inter / uni : 0.0f;
}

struct YpCand {
    float box[4];
    float obj;
    float cls;
    float score;
    int   class_id;
};

template <typename T>
static int yp_decode_scale(const T* src, float qscale, int grid, int scale_idx,
                           float t_gate, float conf_thres,
                           YpCand* cand, int n_cand, int max_cand)
{
    const int HW = grid * grid;
    const int stride_attr = HW;
    const int stride_anchor = YP_BBOX_ATTRS * HW;

    for (int a = 0; a < 3; a++) {
        const T* pa = src + a * stride_anchor;
        const float anchor_w = YP_ANCHORS[YP_MASK[scale_idx][a]][0];
        const float anchor_h = YP_ANCHORS[YP_MASK[scale_idx][a]][1];

        for (int r = 0; r < grid; r++) {
            for (int c = 0; c < grid; c++) {
                const int p = r * grid + c;

                const float t_obj = (float)pa[4 * stride_attr + p] * qscale;
                if (t_obj < t_gate)
                    continue;

                int   best_k = 0;
                float best_t = pa[5 * stride_attr + p] * qscale;
                for (int k = 1; k < YP_NUM_CLASSES; k++) {
                    const float t = (float)pa[(5 + k) * stride_attr + p] * qscale;
                    if (t > best_t) {
                        best_t = t;
                        best_k = k;
                    }
                }
                if (best_t < t_gate)
                    continue;

                const float obj = yp_sigmoid(t_obj);
                const float cls = yp_sigmoid(best_t);
                const float score = obj * cls;
                if (score < conf_thres)
                    continue;

                if (n_cand >= max_cand)
                    return n_cand;

                const float t_x = (float)pa[0 * stride_attr + p] * qscale;
                const float t_y = (float)pa[1 * stride_attr + p] * qscale;
                const float t_w = (float)pa[2 * stride_attr + p] * qscale;
                const float t_h = (float)pa[3 * stride_attr + p] * qscale;

                const float cx = (yp_sigmoid(t_x) + (float)c) / (float)grid;
                const float cy = (yp_sigmoid(t_y) + (float)r) / (float)grid;

                const float bw = expf(t_w) * anchor_w / (float)YP_INPUT_SIZE;
                const float bh = expf(t_h) * anchor_h / (float)YP_INPUT_SIZE;

                YpCand* d = &cand[n_cand++];
                d->box[0] = cx - bw * 0.5f;
                d->box[1] = cy - bh * 0.5f;
                d->box[2] = cx + bw * 0.5f;
                d->box[3] = cy + bh * 0.5f;
                d->obj = obj;
                d->cls = cls;
                d->score = score;
                d->class_id = best_k;
            }
        }
    }
    return n_cand;
}

template <typename T>
static int yp_postprocess(const T* out0, const T* out1, float qscale,
                          int img_w, int img_h,
                          float conf_thres, float nms_thres,
                          YoloBox* dets, int max_det)
{
    static YpCand cand[YP_MAX_CAND];
    static int    order[YP_MAX_CAND];
    static char   removed[YP_MAX_CAND];

    if (dets == 0 || max_det <= 0)
        return 0;

    float t_gate;
    if (conf_thres <= 0.0f)
        t_gate = -1e30f;
    else if (conf_thres >= 1.0f)
        t_gate = 1e30f;
    else
        t_gate = logf(conf_thres / (1.0f - conf_thres));

    int n = 0;
    n = yp_decode_scale<T>(out0, qscale, YP_GRID0, 0, t_gate, conf_thres,
                           cand, n, YP_MAX_CAND);
    n = yp_decode_scale<T>(out1, qscale, YP_GRID1, 1, t_gate, conf_thres,
                           cand, n, YP_MAX_CAND);
    if (n == 0)
        return 0;

    for (int i = 0; i < n; i++) {
        order[i] = i;
        removed[i] = 0;
    }
    for (int i = 0; i < n - 1; i++) {
        int best = i;
        for (int j = i + 1; j < n; j++) {
            if (cand[order[j]].score > cand[order[best]].score)
                best = j;
        }
        if (best != i) {
            int t = order[i];
            order[i] = order[best];
            order[best] = t;
        }
    }

    int n_det = 0;
    for (int i = 0; i < n && n_det < max_det; i++) {
        const int ii = order[i];
        if (removed[ii])
            continue;

        const YpCand* keep = &cand[ii];

        YoloBox* o = &dets[n_det++];

        o->top    = keep->box[1] * (float)img_h;
        o->left   = keep->box[0] * (float)img_w;
        o->bottom = keep->box[3] * (float)img_h;
        o->right  = keep->box[2] * (float)img_w;

        if (o->top < 0.0f)    o->top = 0.0f;
        if (o->left < 0.0f)   o->left = 0.0f;
        if (o->bottom > (float)img_h) o->bottom = (float)img_h;
        if (o->right  > (float)img_w) o->right  = (float)img_w;
        o->obj_conf = keep->obj;
        o->class_conf = keep->cls;
        o->score = keep->score;
        o->class_id = keep->class_id;

        for (int j = i + 1; j < n; j++) {
            const int jj = order[j];
            if (removed[jj])
                continue;
            if (cand[jj].class_id != keep->class_id)
                continue;
            if (yp_iou(keep->box, cand[jj].box) > nms_thres)
                removed[jj] = 1;
        }
    }
    return n_det;
}

int yolo_postprocess_f(const float* out0, const float* out1,
                       int img_w, int img_h,
                       float conf_thres, float nms_thres,
                       YoloBox* dets, int max_det)
{
    return yp_postprocess<float>(out0, out1, 1.0f, img_w, img_h,
                                 conf_thres, nms_thres, dets, max_det);
}

int yolo_postprocess_q(const short* out0, const short* out1,
                       int img_w, int img_h,
                       float conf_thres, float nms_thres,
                       YoloBox* dets, int max_det)
{
    return yp_postprocess<short>(out0, out1, 1.0f / YP_SCALE, img_w, img_h,
                                 conf_thres, nms_thres, dets, max_det);
}

#if YP_MODEL_WEED
const char* const yp_class_names[YP_NUM_CLASSES] = {
    "weed",
    "crop"
};
#else

const char* const yp_class_names[YP_NUM_CLASSES] = {
    "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush"
};
#endif
