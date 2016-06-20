/*
 * cmd.h
 *
 *  Created on: 2016Äê1ÔÂ19ÈÕ
 *      Author: victory
 */

#ifndef USR_INC_CMD_H_
#define USR_INC_CMD_H_

/*command*/
#define CMD_PF_MOVE                     		0x00
#define CMD_PF_STOP                     		0x01
		
#define CMD_GET_POS                     		0x02
#define CMD_SET_POS                     		0x03
#define CMD_SET_ENCPOS                  		0x04
#define CMD_GET_ENCPOS                  		0x05
#define CMD_ENABLE                      		0x06
		
		
#define CMD_INITIALIZE_CARD		          		0x07
#define CMD_GET_MOTOR_STATUS            		0x08
		
#define CMD_PF_SET_STEP1                		0x09
#define CMD_PF_SET_STEP2                		0x0A
		
#define CMD_CNT_MOVE                    		0x0B
#define CMD_CNT_STOP                    		0x0C
#define CMD_DISENABLE                   		0x0D
#define CMD_CHANGE_PF_MOVE									0x0E
		
#define CMD_SET_TRIGGER_POS									0x12
#define CMD_SET_TRIGGER_PERIOD							0x13
#define CMD_CLEAR_TRIGGER										0x14
#define CMD_SET_LIGHT_PERIOD								0x15
		
#define CMD_SET_PLIMIT											0x16
#define CMD_SET_NLIMIT											0x17
#define CMD_CLEAR_LIMIT											0x18
#define CMD_SEARCH_SENSOR										0x19
#define CMD_ENABLE_ENCODER									0x1A
#define CMD_DISABLE_ENCODER									0x1B

void Deal_Cmd(void);

#endif /* USR_INC_CMD_H_ */
