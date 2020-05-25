/*
 * cmd_gen.h
 *
 *  Created on: 2014/04/21
 *      Author: seigo13
 */

#ifndef CMD_GEN_H_
#define CMD_GEN_H_

#include <jendefs.h>

bool_t bSerCmd_SetModuleSetting(uint8 *au8cmd, uint8 u8len);
void vSerResp_GetModuleSetting(uint32 u32setting);

void vSerResp_Ack(uint8 u8Status);
void vSerResp_TxEx(uint8 u8RspId, uint8 u8Status);

void vSerResp_GetModuleAddress();

#endif /* CMD_GEN_H_ */
