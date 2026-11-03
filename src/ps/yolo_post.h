#ifndef YOLO_POST_H_INCLUDED
#define YOLO_POST_H_INCLUDED

#define YP_MODEL_WEED   1

#if YP_MODEL_WEED
#define YP_NUM_CLASSES  2
#else
#define YP_NUM_CLASSES  80
#endif

#define YP_BBOX_ATTRS   (5 + YP_NUM_CLASSES)
#define YP_OUT_CH       (YP_BBOX_ATTRS * 3)

#define YP_CLS_WEED     0
#define YP_CLS_CROP     1
#define YP_INPUT_SIZE   416
#define YP_GRID0        13
#define YP_GRID1        26

#define YP_MAX_CAND     512
#define YP_MAX_DET      64

#define YP_SCALE        512.0f

struct YoloBox {
    float top, left, bottom, right;
    float obj_conf;
    float class_conf;
    float score;
    int   class_id;
};

int yolo_postprocess_f(const float* out0, const float* out1,
                       int img_w, int img_h,
                       float conf_thres, float nms_thres,
                       YoloBox* dets, int max_det);

int yolo_postprocess_q(const short* out0, const short* out1,
                       int img_w, int img_h,
                       float conf_thres, float nms_thres,
                       YoloBox* dets, int max_det);

extern const char* const yp_class_names[YP_NUM_CLASSES];

#ifdef OUT_CH
#if (YP_OUT_CH != OUT_CH)
#error "YP_NUM_CLASSES 与 basic_op.h 的 OUT_CH 不匹配: 应满足 OUT_CH == (YP_NUM_CLASSES+5)*3"
#endif
#endif

#endif
