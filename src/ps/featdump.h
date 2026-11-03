#ifndef FEATDUMP_H_INCLUDED
#define FEATDUMP_H_INCLUDED

#include "sd_io.h"

extern int g_feat_dump;

extern const char* g_feat_dir;

void fd_request(const char* dir);

void fd_layer(const char* tag, data_t* x, int c, int h, int w);

#define FD_MAX_CH   16

#endif
