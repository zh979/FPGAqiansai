#ifndef YOLO4_TINY_H_INCLUDED
#define YOLO4_TINY_H_INCLUDED
#include"basic_block.h"

class Yolo4_Tiny{
public:
    CSPDarkNet* backbone;

    BasicConv* conv_forP5_conv;

    BasicConv* yolo_headP4_basic_conv1;
    data_t* yolo_headP4_w2;
    data_t* yolo_headP4_b2;

    BasicConv* yolo_headP5_basic_conv1;
    data_t* yolo_headP5_w2;
    data_t* yolo_headP5_b2;

    BasicConv* upsample_conv;
    Yolo4_Tiny(){
        this->backbone=new CSPDarkNet();

        this->conv_forP5_conv=new BasicConv(13,13,1,1,0,512,256);

        this->yolo_headP4_basic_conv1=new BasicConv(26,26,3,1,1,384,256);
        this->yolo_headP4_w2=new data_t[OUT_CH*256*1*1];
        this->yolo_headP4_b2=new data_t[OUT_CH];

        this->yolo_headP5_basic_conv1=new BasicConv(13,13,3,1,1,256,512);
        this->yolo_headP5_w2=new data_t[OUT_CH*512*1*1];
        this->yolo_headP5_b2=new data_t[OUT_CH];

        this->upsample_conv=new BasicConv(13,13,1,1,0,256,128);
    }
    void load_weight(string dir){
        this->backbone->load_weight(dir);

        this->conv_forP5_conv->load_weight(dir+"\\conv_forP5");

        read_params(dir+"\\yolo_headP4\\w1.bin",this->yolo_headP4_basic_conv1->weight,256*384*3*3);
        read_params(dir+"\\yolo_headP4\\b1.bin",this->yolo_headP4_basic_conv1->bias,256);
        read_params(dir+"\\yolo_headP4\\w2.bin",this->yolo_headP4_w2,OUT_CH*256*1*1);
        read_params(dir+"\\yolo_headP4\\b2.bin",this->yolo_headP4_b2,OUT_CH);

        read_params(dir+"\\yolo_headP5\\w1.bin",this->yolo_headP5_basic_conv1->weight,512*256*9);
        read_params(dir+"\\yolo_headP5\\b1.bin",this->yolo_headP5_basic_conv1->bias,512);
        read_params(dir+"\\yolo_headP5\\w2.bin",this->yolo_headP5_w2,OUT_CH*512*1*1);
        read_params(dir+"\\yolo_headP5\\b2.bin",this->yolo_headP5_b2,OUT_CH);

        this->upsample_conv->load_weight(dir+"\\upsample");
    }

    void forward(data_t* in,data_t* out0,data_t* out1){
    	static data_t feat1[256*26*26];
		static data_t feat2[512*13*13];
		static data_t P5[256*13*13];
		static data_t out0_tmp[512*13*13];
		static data_t P5_Upsample[128*13*13];
		static data_t P4[384*26*26];
		static data_t out1_tmp[256*26*26];

        this->backbone->forward(in,P4+128*26*26,feat2);

        this->conv_forP5_conv->forward(feat2,P5);

        this->yolo_headP5_basic_conv1->forward(P5,out0_tmp);
        conv_leakyrelu(512,OUT_CH,0,1,1,13,13,out0_tmp,this->yolo_headP5_w2,this->yolo_headP5_b2,out0,0);

        this->upsample_conv->forward(P5,P5_Upsample);

        sampling(P5_Upsample,P4,128,13,0);

        this->yolo_headP4_basic_conv1->forward(P4,out1_tmp);
        conv_leakyrelu(256,OUT_CH,0,1,1,26,26,out1_tmp,this->yolo_headP4_w2,this->yolo_headP4_b2,out1,0);
    }
};

#endif
