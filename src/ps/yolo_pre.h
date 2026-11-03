#ifndef YOLO_PRE_H_INCLUDED
#define YOLO_PRE_H_INCLUDED

#define YPRE_NET_SIZE   416

#ifndef YPRE_SRC_BGR
#define YPRE_SRC_BGR    1
#endif

#define YPRE_NEAREST    0
#define YPRE_BOX2X2     1
#define YPRE_BILINEAR   2

int yolo_preprocess(const unsigned char* src,
                    int src_w, int src_h, int src_stride,
                    int roi_x, int roi_y, int roi_w, int roi_h,
                    int mode,
                    short* dst);

void yolo_map_box_to_frame(float* top, float* left, float* bottom, float* right,
                           int roi_x, int roi_y, int roi_w, int roi_h);

#endif
