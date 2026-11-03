#ifndef GIMBAL_H_INCLUDED
#define GIMBAL_H_INCLUDED

#define GB_VIRT_W   320
#define GB_VIRT_H   240

struct Gimbal {
    int   virt_w, virt_h;
    float smooth;
    float sx, sy;
    int   has_state;

    int   last_nx, last_ny, last_laser;
    int   last_is_pos;
    int   has_last;

    unsigned long sent;
    unsigned long repeated;
    unsigned long skipped;
};

void gb_init(Gimbal* g, float smooth);

void gb_lost(Gimbal* g);

int gb_send(Gimbal* g, float cx, float cy, int frame_w, int frame_h,
            int laser, int* out_nx, int* out_ny);

int gb_repeat(Gimbal* g);

int gp_send(Gimbal* g, long gx, long gy, int laser,
            int* out_nx, int* out_ny);

#endif
