#include <iostream>
#include <stdio.h>
#include "xil_types.h"
#include "xil_cache.h"
#include "xparameters.h"
#include "xgpio.h"
#include "xaxivdma.h"
#include "xaxivdma_i.h"

#include "display_ctrl/display_ctrl.h"
#include "vdma_api/vdma_api.h"
#include "emio_sccb_cfg/emio_sccb_cfg.h"
#include "ov5640/ov5640_init.h"
#include "cam_lock.h"

#include "debug.h"
#include "calib.h"
#include "laser.h"
#include "capture.h"
#include "laser_test.h"
#include "calib_run.h"
#include "motion.h"

#pragma GCC optimize(3,"Ofast","inline")
using namespace std;

#define DYNCLK_BASEADDR    XPAR_AXI_DYNCLK_0_BASEADDR
#define DISP_VTC_ID        XPAR_VTC_0_DEVICE_ID
#define AXI_GPIO_0_ID      XPAR_AXI_GPIO_0_DEVICE_ID
#define AXI_GPIO_0_CHANEL  1

#define VDMA_CAP_ID        XPAR_AXIVDMA_1_DEVICE_ID
#define VDMA_DISP_ID       XPAR_AXIVDMA_0_DEVICE_ID

#define FRAME_BUFFER_ADDR   0x18000000u
#define FRAME_BUFFER_RESERVE 0x04000000u

unsigned int const frame_buffer_addr = FRAME_BUFFER_ADDR;
unsigned int const frame_buffer_disp = FRAME_BUFFER_ADDR + 0x01000000u;

extern "C" {
    extern char _heap_start[];
    extern char _end[];
}

XAxiVdma     vdma_cap;
XAxiVdma     vdma_disp;
DisplayCtrl  dispCtrl;
XGpio        axi_gpio_inst;
VideoMode    vd_mode;
unsigned int lcd_id;

void pre_selftest(unsigned int fb_addr, int fb_w, int fb_h, const char* out_dir);

void detect_loop(Yolo4_Tiny* yolo, unsigned int fb_cap, unsigned int fb_disp,
                 int w, int h, int n_frames);
void detect_loop_smooth(Yolo4_Tiny* yolo, unsigned int fb_cap,
                        int w, int h, int n_frames);
void detect_loop_event(Yolo4_Tiny* yolo, unsigned int fb_cap,
                       int w, int h, int n_frames,
                       int cell_thres, int cell_count);
void detect_loop_track(Yolo4_Tiny* yolo, unsigned int fb_cap,
                       int w, int h, int n_frames,
                       int cell_thres, int cell_count,
                       int want_class, float lead_sec);
void detect_loop_weed(Yolo4_Tiny* yolo, unsigned int fb_cap,
                      int w, int h, int n_frames,
                      int cell_thres, int cell_count,
                      float lead_sec,
                      const Calib* cal, const LaserZone* zone,
                      float aim_off_x, float aim_off_y);
void detect_loop_demo(Yolo4_Tiny* yolo, unsigned int fb_cap,
                      int w, int h, int n_frames,
                      int cell_thres, int cell_count,
                      float lead_sec,
                      const Calib* cal, const LaserZone* zone,
                      float aim_off_x, float aim_off_y);

#define LOOP_MODE   4

#define TRACK_CLASS   0

#define DATASET_DIR   "dataset"

#define LZ_X0   120.0f
#define LZ_Y0    72.0f
#define LZ_X1   680.0f
#define LZ_Y1   408.0f

#define AIM_OFF_X    0.0f
#define AIM_OFF_Y    0.0f

#define TRACK_LEAD    0.54f
#define USE_SMOOTH  (LOOP_MODE >= 1)

#define MO_THRES    15
#define MO_COUNT    (MO_CELLS / 100)

int main()
{
    u32 status;
    u16 cmos_h_pixel, cmos_v_pixel, total_h_pixel, total_v_pixel;

    cout << "=== Camera + Accelerator ===" << endl;

    {
        unsigned int prog_lo = (unsigned int)(unsigned long)_heap_start;
        unsigned int prog_hi = (unsigned int)(unsigned long)_end;
        unsigned int fb_lo = FRAME_BUFFER_ADDR;
        unsigned int fb_hi = FRAME_BUFFER_ADDR + FRAME_BUFFER_RESERVE;

        xil_printf("mem: heap_start=0x%08X  _end=0x%08X  fb=0x%08X..0x%08X\r\n",
                   prog_lo, prog_hi, fb_lo, fb_hi);

        if (fb_lo < prog_hi && fb_hi > prog_lo) {
            xil_printf("!! FATAL: frame buffer overlaps program memory\r\n");
            xil_printf("!! move FRAME_BUFFER_ADDR above 0x%08X\r\n", prog_hi);
            return -1;
        }
        if (fb_hi > XPAR_PS7_DDR_0_S_AXI_HIGHADDR) {
            xil_printf("!! FATAL: frame buffer beyond DDR end 0x%08X\r\n",
                       XPAR_PS7_DDR_0_S_AXI_HIGHADDR);
            return -1;
        }
    }

    XGpio_Initialize(&axi_gpio_inst, AXI_GPIO_0_ID);
    XGpio_SetDataDirection(&axi_gpio_inst, AXI_GPIO_0_CHANEL, 0x07);
    lcd_id = LTDC_PanelID_Read(&axi_gpio_inst, AXI_GPIO_0_CHANEL);
    XGpio_SetDataDirection(&axi_gpio_inst, AXI_GPIO_0_CHANEL, 0x00);
    xil_printf("LCD ID: %x\r\n", lcd_id);

    switch (lcd_id) {
        case 0x4342: cmos_h_pixel = 480;  cmos_v_pixel = 272;
                     total_h_pixel = 1800; total_v_pixel = 1000; break;
        case 0x4384: cmos_h_pixel = 800;  cmos_v_pixel = 480;
                     total_h_pixel = 1800; total_v_pixel = 1000; break;
        case 0x7084: cmos_h_pixel = 800;  cmos_v_pixel = 480;
                     total_h_pixel = 1800; total_v_pixel = 1000; break;
        case 0x7016: cmos_h_pixel = 1024; cmos_v_pixel = 600;
                     total_h_pixel = 2200; total_v_pixel = 1000; break;
        case 0x1018: cmos_h_pixel = 1280; cmos_v_pixel = 800;
                     total_h_pixel = 2570; total_v_pixel = 980;  break;
        default:     cmos_h_pixel = 480;  cmos_v_pixel = 272;
                     total_h_pixel = 1800; total_v_pixel = 1000; break;
    }

    emio_init();
    status = ov5640_init(cmos_h_pixel, cmos_v_pixel, total_h_pixel, total_v_pixel);
    if (status == 0)
        xil_printf("OV5640 detected successful!\r\n");
    else
        xil_printf("OV5640 detected failed!\r\n");

    switch (lcd_id) {
        case 0x4342: vd_mode = VMODE_480x272;  break;
        case 0x4384: vd_mode = VMODE_800x480;  break;
        case 0x7084: vd_mode = VMODE_800x480;  break;
        case 0x7016: vd_mode = VMODE_1024x600; break;
        case 0x1018: vd_mode = VMODE_1280x800; break;
        default:     vd_mode = VMODE_800x480;  break;
    }

    run_vdma_frame_buffer(&vdma_cap, VDMA_CAP_ID, vd_mode.width, vd_mode.height,
                          frame_buffer_addr, 0, 0, ONLY_WRITE);
#if USE_SMOOTH

    run_vdma_frame_buffer(&vdma_disp, VDMA_DISP_ID, vd_mode.width, vd_mode.height,
                          frame_buffer_addr, 0, 0, ONLY_READ);
#else
    run_vdma_frame_buffer(&vdma_disp, VDMA_DISP_ID, vd_mode.width, vd_mode.height,
                          frame_buffer_disp, 0, 0, ONLY_READ);
#endif

    DisplayInitialize(&dispCtrl, DISP_VTC_ID, DYNCLK_BASEADDR);
    DisplaySetMode(&dispCtrl, &vd_mode);
    DisplayStart(&dispCtrl);

    cam_lock_awb_aec(2000);

    xil_printf("display %dx%d, frame buffer 0x%08X\r\n",
               vd_mode.width, vd_mode.height, frame_buffer_addr);

    for (volatile int i = 0; i < 20000000; i++)
        ;

    SD_Init();

#if LOOP_MODE == 7

    calib_run(frame_buffer_addr, vd_mode.width, vd_mode.height);
#elif LOOP_MODE == 6

    laser_test_mode();
#elif LOOP_MODE == 5

    capture_mode(frame_buffer_addr, vd_mode.width, vd_mode.height,
                 DATASET_DIR);
#else
    pre_selftest(frame_buffer_addr, vd_mode.width, vd_mode.height, "yolo_test");

    conv_init();
    Yolo4_Tiny* yolo = new Yolo4_Tiny();
    yolo->load_weight("folded_weights");

    Xil_DCacheFlush();
    cout << "weights loaded" << endl;

#if (LOOP_MODE == 4) || (LOOP_MODE == 8)
    {
        static const Calib CAL = {
            { -0.605556f, 0.000000f, 270.222229f },
            {  0.008333f, 0.759259f, -194.000000f },
            1, 9.583f
        };
        static const LaserZone ZONE = { LZ_X0, LZ_Y0, LZ_X1, LZ_Y1 };
#if LOOP_MODE == 8

        detect_loop_demo(yolo, frame_buffer_addr,
                         vd_mode.width, vd_mode.height, 0,
                         MO_THRES, MO_COUNT, TRACK_LEAD, &CAL, &ZONE,
                         AIM_OFF_X, AIM_OFF_Y);
#else
        detect_loop_weed(yolo, frame_buffer_addr,
                         vd_mode.width, vd_mode.height, 0,
                         MO_THRES, MO_COUNT, TRACK_LEAD, &CAL, &ZONE,
                         AIM_OFF_X, AIM_OFF_Y);
#endif
    }
#elif LOOP_MODE == 3
    detect_loop_track(yolo, frame_buffer_addr,
                      vd_mode.width, vd_mode.height, 0,
                      MO_THRES, MO_COUNT, TRACK_CLASS, TRACK_LEAD);
#elif LOOP_MODE == 2
    detect_loop_event(yolo, frame_buffer_addr,
                      vd_mode.width, vd_mode.height, 0,
                      MO_THRES, MO_COUNT);
#elif LOOP_MODE == 1
    detect_loop_smooth(yolo, frame_buffer_addr,
                       vd_mode.width, vd_mode.height, 0);
#else
    detect_loop(yolo, frame_buffer_addr, frame_buffer_disp,
                vd_mode.width, vd_mode.height, 30);
#endif
#endif

    return 0;
}
