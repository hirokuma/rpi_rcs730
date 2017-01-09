#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>

#include "rcs730.h"


/////////////////////////////////////////////////////////////////
static RCS730_callbacktable_t m_rcs730_cbtbl;


/////////////////////////////////////////////////////////////////
static void interruptIrq(void)
{
    RCS730_isrIrq();
}


static bool rcs730cb_read(void *pUser, uint8_t *pData, uint8_t Len)
{
    printf("%s\n", __func__);

    uint8_t nob = pData[13] << 4;       //16byte * NoB
    pData[0] = (uint8_t)(13 + nob);
    pData[10] = 0;  //ST1
    pData[11] = 0;  //ST2
    for (int i = 0; i < nob; i++) {
        pData[13 + i] = (uint8_t)(0x30 + i);
    }

    return true;
}


static bool rcs730cb_write(void *pUser, uint8_t *pData, uint8_t Len)
{
    printf("%s\n", __func__);

    pData[0] = 12;

    pData[16 + 16] = '\0';
    pData[10] = 0;  //ST1
    pData[11] = 0;  //ST2

    return true;
}


/////////////////////////////////////////////////////////////////
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

    ret = RCS730_open();
    if (ret != RCS730_SUCCESS) {
        printf("fail: RCS730_open()\n");
        exit(-1);
    }
    printf("OK: RCS730_open()\n");

    m_rcs730_cbtbl.pUserData = NULL;
    m_rcs730_cbtbl.pCbRxHTRDone = rcs730cb_read;
    m_rcs730_cbtbl.pCbRxHTWDone = rcs730cb_write;
    RCS730_setCallbackTable(&m_rcs730_cbtbl);
    ret = RCS730_initFTMode(RCS730_OPMODE_PLUG);
    if (ret != RCS730_SUCCESS) {
        printf("fail: RCS730_initFTMode()\n");
        exit(-1);
    }
    printf("OK: RCS730_initFTMode()\n");

    while (1) {
        delay(10000);
    }

    RCS730_close();

    return 0;
}
