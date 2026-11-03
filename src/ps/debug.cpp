#include"debug.h"
#include"basic_block.h"
#include<iostream>
#include "xtime_l.h"
#include "yolo_post.h"
using namespace std;

#define TEST_IMG_W  724
#define TEST_IMG_H  1024

void yolo_test(string weight_dir,string ref_dir){
    data_t* in=new data_t[3*416*416];
    data_t* out0=new data_t[OUT_CH*13*13];
    data_t* out1=new data_t[OUT_CH*26*26];
    data_t* out0_ref=new data_t[OUT_CH*13*13];
    data_t* out1_ref=new data_t[OUT_CH*26*26];
    XTime tEnd, tCur;
    u32 tUsed;

    read_params(ref_dir+"\\img.bin",in,3*416*416);

    conv_init();
    Yolo4_Tiny* yolo4_tiny=new Yolo4_Tiny();
    yolo4_tiny->load_weight(weight_dir);
    Xil_DCacheFlush();
    XTime_GetTime(&tCur);
    yolo4_tiny->forward(in,out0,out1);
    Xil_DCacheInvalidateRange((u32)((unsigned int)out0&0xffffffe0), 32*((OUT_CH*13*13*sizeof(data_t))/32+2));
    Xil_DCacheInvalidateRange((u32)((unsigned int)out1&0xffffffe0), 32*((OUT_CH*26*26*sizeof(data_t))/32+2));
    XTime_GetTime(&tEnd);
	tUsed = ((tEnd-tCur)*1000000)/(COUNTS_PER_SECOND);
	cout<<"time elapsed is "<<tUsed<<" us"<<endl;

    {
        XTime tp0, tp1;
        XTime_GetTime(&tp0);

        static YoloBox dets[YP_MAX_DET];
        int nd = yolo_postprocess_q(out0, out1,
                                    TEST_IMG_W, TEST_IMG_H,
                                    0.5f, 0.3f,
                                    dets, YP_MAX_DET);

        XTime_GetTime(&tp1);
        u32 tPost = ((tp1-tp0)*1000000)/(COUNTS_PER_SECOND);
        printf("post-process: %d dets, %lu us\r\n", nd, (unsigned long)tPost);

        for(int i=0;i<nd;i++){
            int top    = (int)dets[i].top;
            int left   = (int)dets[i].left;
            int bottom = (int)dets[i].bottom;
            int right  = (int)dets[i].right;
            if(top<0) top=0;
            if(left<0) left=0;
            if(bottom>TEST_IMG_H) bottom=TEST_IMG_H;
            if(right >TEST_IMG_W) right =TEST_IMG_W;

            int s100 = (int)(dets[i].score * 100.0f + 0.5f);

            printf("b'%s %d.%02d' %d %d %d %d\r\n",
                   yp_class_names[dets[i].class_id], s100/100, s100%100,
                   top, left, bottom, right);
        }
    }

	write_data(ref_dir+"\\out0.bin",out0,OUT_CH*13*13);
	write_data(ref_dir+"\\out1.bin",out1,OUT_CH*26*26);

    delete [] in;
    delete [] out0;
    delete [] out0_ref;
    delete [] out1;
    delete [] out1_ref;
    return;
}
