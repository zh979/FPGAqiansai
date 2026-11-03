#ifndef LASER_H_INCLUDED
#define LASER_H_INCLUDED

enum LaserState {
    LS_IDLE = 0,
    LS_AIMING,
    LS_FIRING,
    LS_COOLDOWN
};

struct LaserZone {
    float x0, y0, x1, y1;
};

#define LZ_MAX_DONE   16

enum LsBlock {
    LB_NONE = 0,
    LB_NOTGT,
    LB_PRED,
    LB_CLASS,
    LB_SCORE,
    LB_ZONE,
    LB_DONE,
    LB_JUMP,
    LB_AIM
};

const char* ls_block_name(int why);

struct Laser {
    LaserState state;
    int   on;

    LaserZone zone;
    float aim_tol_px;
    int   aim_hold_ticks;
    int   fire_ticks;
    int   cool_ticks;
    float min_score;
    float done_radius_px;
    int   fire_coast_max;
    float jump_tol_px;

    unsigned long done_max_age_ms;

    float vel_deadband;

    int   aim_cnt;
    int   fire_cnt;
    int   cool_cnt;
    int   fire_coast;
    float tgt_x, tgt_y;

    float done_x[LZ_MAX_DONE];
    float done_y[LZ_MAX_DONE];

    float done_vx[LZ_MAX_DONE];
    float done_vy[LZ_MAX_DONE];
    unsigned long done_t[LZ_MAX_DONE];

    int   done_uid[LZ_MAX_DONE];
    int   done_n;
    int   done_head;

    int   fire_done_idx;

    unsigned long now_ms;

    int   just_done;
    int   blk_why;

    unsigned long n_pulse;
    unsigned long n_fired;
    unsigned long n_blocked;
};

void ls_init(Laser* l, const LaserZone* zone);

void ls_clear_done(Laser* l);

int ls_is_done(const Laser* l, float x, float y);

int ls_tick(Laser* l, int has_target, int measured, int class_id,
            float score, float tx, float ty, float aim_err_px, int uid,
            float vx, float vy, unsigned long now_ms);

void ls_abort(Laser* l);

const char* ls_state_name(LaserState s);

#endif
