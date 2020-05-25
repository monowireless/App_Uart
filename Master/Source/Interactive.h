/*
 * Interactive.h
 *
 *  Created on: 2014/04/21
 *      Author: seigo13
 */

#ifndef INTERACTIVE_H_
#define INTERACTIVE_H_

#include "flash.h"

void vConfig_SetDefaults(tsFlashApp *p);
void vConfig_UnSetAll(tsFlashApp *p);
void vConfig_SaveAndReset();

void vProcessInputByte(uint8 u8Byte);
void vProcessInputString(tsInpStr_Context *pContext);

void vSerUpdateScreen();

/**
 * スリープ用のIOピンのプルアップ停止
 */
#define E_APPCONF_OPT_DISABLE_PULL_UP_SLEEP_PIN 0x10UL //!< スリープピンのプルアップを停止する @ingroup FLASH
#define IS_APPCONF_OPT_DISABLE_PULL_UP_SLEEP_PIN() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_DISABLE_PULL_UP_SLEEP_PIN) //!< E_APPCONF_OPT_DISABLE_PULL_UP_SLEEP_PIN 判定 @ingroup FLASH

/**
 * 透過モード選択ピンのプルアップの停止
 */
#define E_APPCONF_OPT_DISABLE_PULL_UP_DI_PIN 0x1UL //!< スリープピンのプルアップを停止する @ingroup FLASH
#define IS_APPCONF_OPT_DISABLE_PULL_UP_DI_PIN() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_DISABLE_PULL_UP_DI_PIN) //!< E_APPCONF_OPT_DISABLE_PULL_UP_DI_PIN 判定 @ingroup FLASH

/**
 * UART 設定の強制
 */
#define E_APPCONF_OPT_UART_FORCE_SETTINGS 0x10000UL //!< スリープピンのプルアップを停止する @ingroup FLASH
#define IS_APPCONF_OPT_UART_FORCE_SETTINGS() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_UART_FORCE_SETTINGS) //!< E_APPCONF_OPT_UART_FORCE_SETTINGS 判定 @ingroup FLASH

/**
 * 透過モードで、改行コード (0x0D) で送信を行うオプション(それまでは何もしない)
 */
#define E_APPCONF_OPT_TX_BY_CR_ON_TRANSPARENT 0x100UL //!< スリープピンのプルアップを停止する @ingroup FLASH
#define IS_APPCONF_OPT_TX_BY_CR_ON_TRANSPARENT() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_TX_BY_CR_ON_TRANSPARENT) //!< E_APPCONF_OPT_TX_BY_CR_ON_TRANSPARENT 判定 @ingroup FLASH

#endif /* INTERACTIVE_H_ */
