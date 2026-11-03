#include "laser_test.h"
#include "xparameters.h"
#include "xuartps_hw.h"
#include "sleep.h"
#include <stdio.h>

#define UART_BASE   XPAR_PS7_UART_0_BASEADDR

static void emit(int on)
{
    printf("X160Y120L%d\r\n", on ? 1 : 0);
}

static void hold(int on, int ms)
{
    int n = ms / 50;
    for (int i = 0; i < n; i++) {
        emit(on);
        usleep(50000);
    }
}

static void wait_key(const char* prompt)
{
    printf("%s", prompt);
    while (XUartPs_IsReceiveData(UART_BASE) == 0)
        ;
    (void)XUartPs_RecvByte(UART_BASE);
    printf("\r\n");
}

void laser_test_mode(void)
{
    printf("\r\n=== Laser interlock self-test ===\r\n");
    printf("All frames are X160Y120 -> zero error -> gimbal stays still.\r\n");
    printf("\r\n");
    printf("KEEP THE LASER ON THE BENCH, POINTING AT A WALL.\r\n");
    printf("Do NOT mount it on the gimbal until steps 4 and 6 pass.\r\n");
    wait_key("\r\npress any key to start... ");

    printf("[1/6] sending nothing for 3 s\r\n");
    printf("      expect: OFF, not even a faint glow\r\n");
    usleep(3000000);
    wait_key("      fully off? press any key... ");

    printf("[2/6] L1 for 2.5 s      expect: ON\r\n");
    hold(1, 2500);
    wait_key("      did it light up? press any key... ");

    printf("[3/6] L0 for 2.5 s      expect: OFF\r\n");
    hold(0, 2500);
    wait_key("      did it go off? press any key... ");

    printf("[4/6] holding L1 for 8 s (STM32 hard limit is 3 s)\r\n");
    printf("      expect: ON for ~3 s, then OFF BY ITSELF\r\n");
    for (int s = 1; s <= 8; s++) {
        hold(1, 1000);
        printf("      t=%d s%s\r\n", s,
               (s == 3) ? "   <- should go dark about now" : "");
    }
    wait_key("      did it cut off by itself? press any key... ");

    printf("[5/6] one L0 to release the latch, then L1 for 2.5 s\r\n");
    printf("      expect: ON again\r\n");
    hold(0, 300);
    hold(1, 2500);
    wait_key("      did it come back? press any key... ");

    printf("[6/6] L1 for 1 s, then output stops completely\r\n");
    printf("      (simulates this board crashing or the cable coming loose)\r\n");
    printf("      expect: OFF within ~200 ms after the line goes quiet\r\n");
    hold(1, 1000);
    printf("      --- output stopped, watch the laser ---\r\n");
    usleep(3000000);
    wait_key("      did it go off on its own? press any key... ");

    printf("\r\n=== done ===\r\n");
    printf("Steps 4 and 6 are the ones that matter: enforced by the\r\n");
    printf("STM32 alone, so they still hold if this board dies.\r\n");
    printf("If either failed, do NOT mount the laser on the gimbal.\r\n");

    hold(0, 500);
    while (1)
        ;
}
