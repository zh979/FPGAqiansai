#ifndef BASIC_OP_H_INCLUDED
#define BASIC_OP_H_INCLUDED
#include"sd_io.h"
#include<cmath>
#include<string>
#include<iostream>
using namespace std;
#include"xconv.h"
#include"xil_cache.h"

#define OUT_CH 21

static XConv conv_inst;

typedef void (*conv_idle_hook_t)(void);
extern conv_idle_hook_t g_conv_idle_hook;

void conv_init();

void conv_leakyrelu(int ch_in,int ch_out,int pad,int stride,int k,int h,int w,data_t* in,data_t *weight,data_t *bias,data_t *out,int act);

void sampling(data_t* in,data_t* out,int ch,int fsize,int mode);

#endif
