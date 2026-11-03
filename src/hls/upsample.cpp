#include"upsample.h"

void upsample(data_t* in,data_t* out){
	data_t tmp;
	for(unsigned short n=0;n<128;n++)
		for(unsigned short i=0;i<13;i++)
			for(unsigned short j=0;j<13;j++){
#pragma HLS PIPELINE
				int wr_base_addr=n*676+i*52+2*j;

				int rd_base_addr=n*169+i*13;

				tmp=*(in+rd_base_addr+j);
				*(out+wr_base_addr)=tmp;
				*(out+wr_base_addr+1)=tmp;
				*(out+wr_base_addr+26)=tmp;
				*(out+wr_base_addr+27)=tmp;
			}
}
