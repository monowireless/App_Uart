/* Copyright (C) 2021 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-*J,*E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT).                                                   */

/** @file
 *
 * @defgroup SEROUT UART出力のための処理
 *
 * UARTよりインタラクティブに文字列を入力する。
 *
 */

#ifndef SER_OUT_H_
#define SER_OUT_H_

#include <jendefs.h>
#include <string.h>
#include <AppHardwareApi.h>

#include <utils.h>


// Serial options
#include <serial.h>
#include <fprintf.h>
#include <sprintf.h>

#include "App_Uart.h"
#include "sercmd_plus3.h"
#include "sercmd_gen.h"

#include "Interactive.h"
#include "cmd_gen.h"

#include "ser_out.h"

void vSerInitMessage();
void vSerChatPrompt();
void vSerOutputFmt(uint8 *pOutputData, uint16 u16len, tsSerSeq *pSerSeq, tsRxDataApp *pRx);
uint16 u16SerTimeUpdate();

#endif /* SER_OUT_H_ */
