/** FeliCa Link(RC-S730) Library
 *
 * @file    rcs730.c
 * @author  hiro99ma
 * @version 1.00
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "rcs730.h"


#define I2C_DEV         "/dev/i2c-1"
#define I2C_SLV_ADDR    (0x40)          //Default Slave Address(7bit)
#define RETRY_NUM       (10)            //max I2C Retry count


#if 1
#define DBG_PRINTF(...)
#define DBG_PERROR(...)
#else
#define DBG_PRINTF      printf
#define DBG_PERROR      perror
#endif


//static uint8_t                  _slvAddr;
static RCS730_callbacktable_t   _cbTable;
static int                      _fd = -1;


static int set_tag_rf_send_enable(void)
{
    uint32_t val = 0x00000001;
    return RCS730_pageWrite(RCS730_REG_TAG_TX_CTRL, (const uint8_t*)&val, sizeof(val));
}


static int read_rf_buf(uint8_t *pData)
{
    const int LEN_FIRST = 16;
    int len = 0;
    int ret;

    //read from LEN
    ret = RCS730_sequentialRead(RCS730_BUF_RF_COMM, pData, LEN_FIRST);
    if (ret == RCS730_SUCCESS) {
        len = pData[0];
    }
    if ((ret == RCS730_SUCCESS) && (pData[0] > LEN_FIRST)) {
        ret = RCS730_sequentialRead(RCS730_BUF_RF_COMM + LEN_FIRST, pData + LEN_FIRST, pData[0] - LEN_FIRST);
        if (ret != RCS730_SUCCESS) {
            len = 0;
        }
    }

    return len;
}


int RCS730_open(void)
{
    if (_fd != -1) {
        return RCS730_ERROR;
    }

    _cbTable.pUserData = 0;
    _cbTable.pCbRxHTRDone = 0;
    _cbTable.pCbRxHTWDone = 0;
    _fd = open(I2C_DEV, O_RDWR);
    if (_fd == -1) {
        DBG_PERROR("open");
        return RCS730_ERROR;
    }
    int ret = ioctl(_fd, I2C_SLAVE, I2C_SLV_ADDR);
    if (ret < 0) {
        DBG_PERROR("ioctl");
        return RCS730_ERROR;
    }
//    for (uint16_t reg = RCS730_REG_OPMODE; reg <= RCS730_REG_PLUG_CONF3; reg += 4) {
//        uint32_t val;
//        RCS730_readRegister(reg, &val);
//    }

    return RCS730_SUCCESS;
}


void RCS730_close(void)
{
    if (_fd != -1) {
        close(_fd);
        _fd = -1;
    }
}


void RCS730_setCallbackTable(const RCS730_callbacktable_t *pInitTable)
{
    _cbTable = *pInitTable;
}


#if 0
int RCS730_byteWrite(uint16_t MemAddr, uint8_t Data)
{
    int ret;
    int retry = RETRY_NUM;
    uint8_t buf[3];

    buf[0] = (char)(MemAddr >> 8);
    buf[1] = (char)(MemAddr & 0xff);
    buf[2] = (char)Data;

    do {
        ret = twi_master_transfer(I2C_SLV_ADDR, buf, (uint8_t)sizeof(buf), TWI_ISSUE_STOP);
    } while ((ret != 0) && (retry--));

    return ret;
}
#endif


int RCS730_pageWrite(uint16_t MemAddr, const uint8_t *pData, uint8_t Length)
{
    int ret;
    int retry = RETRY_NUM;
    uint8_t buf[256];
    uint8_t *p;

    if (Length > 254) {
        return RCS730_INVALID_PARAM;
    }

    buf[0] = (char)(MemAddr >> 8);
    buf[1] = (char)(MemAddr & 0xff);
    memcpy(&buf[2], pData, Length);

    int len = 2 + Length;
    p = buf;
    do {
        ret = write(_fd, p, len);
        //DBG_PRINTF("[%d]W ret=%d\n", __LINE__, ret);
        if (ret > 0) {
            p += ret;
            len -= ret;
        }
    } while ((len > 0) && (retry--));

    return (int)((len == 0) ? RCS730_SUCCESS : RCS730_ERROR);
}


#if 0
int RCS730_randomRead(uint16_t MemAddr, uint8_t *pData)
{
    return RCS730_sequentialRead(MemAddr, pData, 1);
}
#endif


int RCS730_sequentialRead(uint16_t MemAddr, uint8_t *pData, uint8_t Length)
{
    int ret;
    int retry = RETRY_NUM;
    uint8_t buf[256];
    uint8_t *p;

    buf[0] = (char)(MemAddr >> 8);
    buf[1] = (char)(MemAddr & 0xff);

    int len = 2;
    p = buf;
    do {
        ret = write(_fd, p, len);
        //DBG_PRINTF("[%d]RW ret=%d\n", __LINE__, ret);
        if (ret > 0) {
            p += ret;
            len -= ret;
        }
    } while ((len > 0) && (retry--));
    if (len == 0) {
        retry = RETRY_NUM;
        do {
            ret = read(_fd, pData, Length);
            //DBG_PRINTF("[%d]RR ret=%d\n", __LINE__, ret);
            pData += ret;
            Length -= ret;
        } while ((Length > 0) && (retry--));
    } else {
        DBG_PRINTF("RW fail\n");
    }

    return (int)((Length == 0) ? RCS730_SUCCESS : RCS730_ERROR);
}


#if 0
int RCS730_currentAddrRead(uint8_t *pData)
{
    int ret;
    int retry = RETRY_NUM;

    do {
        ret = twi_master_transfer((uint8_t)(I2C_SLV_ADDR | TWI_READ_BIT), pData, 1, TWI_ISSUE_STOP);
    } while ((ret != 0) && (retry--));

    return ret;
}
#endif


int RCS730_readRegister(uint16_t Reg, uint32_t* pData)
{
    int ret = RCS730_sequentialRead(Reg, (uint8_t*)pData, sizeof(uint32_t));
    DBG_PRINTF("[%s]Reg=%04x, Data=%08x\n", __func__, Reg, *pData);
    return ret;
}


int RCS730_writeRegisterForce(uint16_t Reg, uint32_t Data)
{
    DBG_PRINTF("[%s]Reg=%04x, Data=%08x\n", __func__, Reg, Data);
    return RCS730_pageWrite(Reg, (const uint8_t*)&Data, sizeof(Data));
}


int RCS730_writeRegister(uint16_t Reg, uint32_t Data, uint32_t Mask)
{
    int ret;
    uint32_t cur;   //current register value

    ret = RCS730_readRegister(Reg, &cur);
    if (ret == RCS730_SUCCESS) {
        if ((cur & Mask) != Data) {
            DBG_PRINTF("[%s]Reg=%04x, Data=%08x\n", __func__, Reg, Data);

            // change value
            Data |= cur & ~Mask;
            ret = RCS730_writeRegisterForce(Reg, Data);
        }
    }

    return ret;
}


int RCS730_setRegOpMode(RCS730_OpMode Mode)
{
    return RCS730_writeRegister(RCS730_REG_OPMODE, (uint32_t)Mode, RCS730_REG_MASK_VAL);
}


#if 0
int RCS730_setRegSlaveAddr(int SAddr)
{
    int ret;

    ret = RCS730_writeRegister(RCS730_REG_I2C_SLAVE_ADDR, (uint32_t)SAddr, RCS730_REG_MASK_VAL);
    if (ret == RCS730_SUCCESS) {
        _slvAddr = SAddr << 1;
    }

    return ret;
}
#endif


int RCS730_setRegInterruptMask(uint32_t Mask, uint32_t Value)
{
    return RCS730_writeRegister(RCS730_REG_INT_MASK, Value, Mask);
}


int RCS730_setRegPlugSysCode(RCS730_PlugSysCode SysCode)
{
    return RCS730_writeRegister(RCS730_REG_PLUG_CONF1, (uint32_t)SysCode, 0x00000002);
}


int RCS730_goToInitializeStatus(void)
{
    return RCS730_writeRegisterForce(RCS730_REG_INIT_CTRL, 0x0000004a);
}


int RCS730_initFTMode(RCS730_OpMode Mode)
{
    int ret;

    if (RCS730_OPMODE_PLUG < Mode) {
        return RCS730_INVALID_PARAM;
    }

    ret = RCS730_setRegOpMode(Mode);
    if (ret == RCS730_SUCCESS) {
        ret = RCS730_setRegInterruptMask(
                RCS730_MSK_INT_TAG_RW_RX_DONE2,
//                RCS730_MSK_INT_TAG_RX_DONE |
//                RCS730_MSK_INT_TAG_PL_RX_DONE |
//                RCS730_MSK_INT_TAG_RW_RX_DONE1 |
//                RCS730_MSK_INT_TAG_RW_RX_DONE2 |
//                RCS730_MSK_INT_TAG_RW_RX_DONE3 |
//                RCS730_MSK_INT_TAG_TX_DONE,
                0);
    }
//    if (ret == RCS730_SUCCESS) {
//        ret = RCS730_setRegPlugSysCode(RCS730_PLUG_SYS_CODE_NDEF);
//    }
    if (ret == RCS730_SUCCESS) {
        ret = RCS730_writeRegister(RCS730_REG_PLUG_CONF3, 0x67452301, RCS730_REG_MASK_VAL);
    }

    return ret;
}


#if 0
int RCS730_initNfcDepMode(void)
{
    int ret;

    ret = RCS730_setRegOpMode(RCS730_OPMODE_NFCDEP);
    if (ret == RCS730_SUCCESS) {
        ret = RCS730_setRegInterruptMask(RCS730_MSK_INT_TAG_NFC_DEP_RX_DONE, 0);
    }

    return ret;
}
#endif


void RCS730_isrIrq(void)
{
    int ret;
    bool b_send = false;
    uint32_t intstat;
    uint8_t rf_buf[256];

    (void)RCS730_readRegister(RCS730_REG_INT_RAW_STATUS, &intstat);

    ret = RCS730_readRegister(RCS730_REG_INT_STATUS, &intstat);
    if (ret == RCS730_SUCCESS) {
        if (intstat & RCS730_MSK_INT_TAG_RW_RX_DONE2) {
            //Read or Write w/o Enc Rx done for HT block
            int len = read_rf_buf(rf_buf);
            if (len > 0) {
                switch (rf_buf[1]) {
                    case 0x06:  //Read w/o Enc
                        DBG_PRINTF("[%d]read\n", __LINE__);
                        if (_cbTable.pCbRxHTRDone) {
                            b_send = (*_cbTable.pCbRxHTRDone)(_cbTable.pUserData, rf_buf, len);
                        }
                        break;
                    case 0x08:  //Write w/o Enc;
                        if (_cbTable.pCbRxHTWDone) {
                            b_send = (*_cbTable.pCbRxHTWDone)(_cbTable.pUserData, rf_buf, len);
                        }
                        break;
                    default:
                        break;
                }
            }
        } else {
            DBG_PRINTF("[%d]intstat=%d\n", __LINE__, intstat);
        }
#if 0
        if (_cbTable.pCbRxDepDone && (intstat & RCS730_MSK_INT_TAG_NFC_DEP_RX_DONE)) {
            //DEP command Rx done
            int len = read_rf_buf(this, rf_buf);
            (*_cbTable.pCbRxDepDone)(_cbTable.pUserData, rf_buf, len);
        }
        if (_cbTable.pCbTxDone && (intstat & MSK_INT_TAG_TX_DONE)) {
            //Tx Done
            int len = read_rf_buf(this, rf_buf);
            (*_cbTable.pCbTxDone)(_cbTable.pUserData, rf_buf, len);
        }

        uint32_t intother = intstat & ~(RCS730_MSK_INT_TAG_TX_DONE | RCS730_MSK_INT_TAG_NFC_DEP_RX_DONE | RCS730_MSK_INT_TAG_RW_RX_DONE2);
        if (_cbTable.pCbOther && intother) {
            (*_cbTable.mCbOther)(_cbTable.pUserData, 0, intother);
        }
#endif

        //response
        if (b_send) {
            ret = RCS730_pageWrite(RCS730_BUF_RF_COMM, rf_buf, rf_buf[0]);
            if (ret == RCS730_SUCCESS) {
                set_tag_rf_send_enable();
            }
        }

        if (intstat) {
            RCS730_writeRegisterForce(RCS730_REG_INT_CLEAR, intstat);
        }
    }
}
