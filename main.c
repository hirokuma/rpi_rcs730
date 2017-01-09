#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>


static void interruptIrq(void)
{
    printf("%s\n", __func__);
}


int main(int argc, char *argv[])
{
    int ret;

    ret = wiringPiSetup();
    if (ret == -1) {
        printf("fail: wiringPi init\n");
        exit(-1);
    }
    printf("OK: wiringPiSetup()\n");

    pinMode(7, INPUT);
    ret = wiringPiISR (7, INT_EDGE_FALLING, interruptIrq);
    if (ret == -1) {
        printf("fail: wiringPiISR()\n");
        exit(-1);
    }
    printf("OK: wiringPiISR()\n");

    while (1) {
        delay(10000);
    }

    return 0;
}

