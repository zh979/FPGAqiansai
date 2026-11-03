#include "featdump.h"
#include <cstdio>
#include <cstring>

int g_feat_dump = 0;
const char* g_feat_dir = "feat";

static int s_done = 0;

#define FD_LAYERS   5

void fd_request(const char* dir)
{
    if (dir)
        g_feat_dir = dir;
    s_done = 0;
    g_feat_dump = 1;

    printf("\r\n[featdump] requested: next inference dumps %d layers to %s/\r\n",
           FD_LAYERS, g_feat_dir);
    printf("[featdump] writing to SD takes a few seconds, display will freeze\r\n");
}

void fd_layer(const char* tag, data_t* x, int c, int h, int w)
{
    if (!g_feat_dump)
        return;

    int nc = (c < FD_MAX_CH) ? c : FD_MAX_CH;
    int plane = h * w;

    char path[96];
    snprintf(path, sizeof(path), "%s\\%s_%dx%dx%d.bin",
             g_feat_dir, tag, c, h, w);

    printf("[featdump] %s: %d/%d channels %dx%d ...\r\n", tag, nc, c, h, w);

    write_data(string(path), x, nc * plane);

    if (++s_done >= FD_LAYERS) {
        g_feat_dump = 0;
        s_done = 0;
        printf("[featdump] all layers written.\r\n");
        printf("[featdump] *** WAIT 3 SECONDS before removing the card ***\r\n");
    }
}
