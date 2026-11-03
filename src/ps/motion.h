#ifndef MOTION_H_INCLUDED
#define MOTION_H_INCLUDED

#define MO_GRID_W   50
#define MO_GRID_H   30
#define MO_CELLS    (MO_GRID_W * MO_GRID_H)

struct MotionResult {
    int   moving;
    int   changed_cells;
    int   energy;

    int   bx0, by0, bx1, by1;
};

int motion_detect(const unsigned char* fb, int w, int h, int stride,
                  int cell_thres, int cell_count, MotionResult* res);

void motion_reset(void);

void motion_set_ignore_bottom(int rows);

#endif
