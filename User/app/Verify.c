
#include "Verify.h"

extern CanTxMsg TxMsg;	
extern CanRxMsg RxMsg; 
extern unsigned int CardId;
extern unsigned int AxisId;

int CheckVerifyCode(void){
    uint16_t cmd = 0;
    uint16_t verifycode = 0;
    uint16_t i = 0;
    uint16_t temp = 0;
    uint16_t card_temp = 0;
    uint16_t axis_temp = 0;
    temp = (RxMsg.ExtId >> 16) & 0xff;
    card_temp = (temp >> 4) & 0xf;
    axis_temp = temp & 0xf;
    cmd = (RxMsg.ExtId >> 8) & 0xff;
    if(CardId != card_temp){
        return -1;
    }
    if(AxisId != axis_temp){
           return -2;
    }
    for(i = 0;i < 8;i ++){
        verifycode ^= RxMsg.Data[i];
    }
    verifycode ^= temp;
    verifycode ^= cmd;
    if(verifycode != (RxMsg.ExtId & 0xff)){
        return -3;
    }
    return 0;
}
unsigned int GetVerifyCode(unsigned int CardId,unsigned int axisid,unsigned int cmd,uint8_t data[]){
    uint16_t verifycode = 0;
    uint16_t i = 0;
    for(i = 0;i < 8;i ++){
        verifycode ^= data[i];
    }
    verifycode ^= (CardId * 16 + axisid);
    verifycode ^= cmd;
    return verifycode;
}
