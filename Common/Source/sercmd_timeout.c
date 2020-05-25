/****************************************************************************
 * (C) Tokyo Cosmos Electric, Inc. (TOCOS) - 2013 all rights reserved.
 *
 * Condition to use: (refer to detailed conditions in Japanese)
 *   - The full or part of source code is limited to use for TWE (TOCOS
 *     Wireless Engine) as compiled and flash programmed.
 *   - The full or part of source code is prohibited to distribute without
 *     permission from TOCOS.
 *
 * 利用条件:
 *   - 本ソースコードは、別途ソースコードライセンス記述が無い限り東京コスモス電機が著作権を
 *     保有しています。
 *   - 本ソースコードは、無保証・無サポートです。本ソースコードや生成物を用いたいかなる損害
 *     についても東京コスモス電機は保証致しません。不具合等の報告は歓迎いたします。
 *   - 本ソースコードは、東京コスモス電機が販売する TWE シリーズ上で実行する前提で公開
 *     しています。他のマイコン等への移植・流用は一部であっても出来ません。
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <string.h>
#include <jendefs.h>
#include "sercmd_gen.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#define IS_TRANSMIT_CHAR(c) (c=='\t' || ((c)>=0x20 && (c)!=0x7F)) // 送信対象文字列か判定

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
/** @ingroup SERCMD
 * シリアルコマンドの状態定義
 */
typedef enum {
	E_SERCMD_TIMEOUT_CMD_EMPTY = 0,      //!< 入力されていない
	E_SERCMD_TIMEOUT_CMD_READPAYLOAD,    //!< E_SERCMD_TIMEOUT_CMD_READPAYLOAD
	E_SERCMD_TIMEOUT_CMD_COMPLETE = 0x80,//!< 入力が完結した(LCRチェックを含め)
	E_SERCMD_TIMEOUT_CMD_ERROR = 0x81,          //!< 入力エラー
} teSercmdTimeoutState;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************///
extern uint32 u32TickCount_ms; //!< ToCoNet での TickTimer

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/** @ingroup SERCMD
 * 入力系列の解釈を行う。
 * ※ 完了条件はタイムアウトで、bComplete() 関数の呼び出しによりチェックされる。
 *
 * @param pCmd 管理構造体
 * @param u8byte 入力文字
 * @return 状態コード (teModbusCmdState 参照)
 */
static uint8 SerCmdTimeout_u8Parse(tsSerCmd_Context *pCmd, uint8 u8byte) {
	// check for complete or error status
	if (pCmd->u8state >= 0x80) {
		pCmd->u8state = E_SERCMD_TIMEOUT_CMD_EMPTY;
	}

	// run state machine
	switch (pCmd->u8state) {
	case E_SERCMD_TIMEOUT_CMD_EMPTY:
		{
			pCmd->u8state = E_SERCMD_TIMEOUT_CMD_READPAYLOAD;
			pCmd->au8data[0] = u8byte;

			pCmd->u32timestamp = u32TickCount_ms; // store received time for timeout
			pCmd->u16pos = 1;
			pCmd->u16len = 1;
			pCmd->u16cksum = 0;
			// memset(pCmd->au8data, 0, pCmd->u16maxlen);
		}
		break;

	case E_SERCMD_TIMEOUT_CMD_READPAYLOAD:
		{
			pCmd->au8data[pCmd->u16pos] = u8byte;

			pCmd->u16pos++;
			pCmd->u16len++;
			pCmd->u32timestamp = u32TickCount_ms; // 入力ごとにタイムアウトを設定していく

			// 最大長のエラー
			if (pCmd->u16pos == pCmd->u16maxlen - 1) {
				pCmd->u8state = E_SERCMD_TIMEOUT_CMD_ERROR;
			}
		}
		break;

	default:
		break;
	}

	return pCmd->u8state;
}

/** @ingroup SERCMD
 * 出力 (vOutputメソッド用)
 * - そのまま系列を出力する。
 *
 * @param pc
 * @param ps
 */
static void SerCmdTimeout_Output(tsSerCmd_Context *pc, tsFILE *ps) {
	int i;

	for (i = 0; i < pc->u16len; i++) {
		vPutChar(ps, pc->au8data[i]);
	}
}


/** @ingroup SERCMD
 * 完了チェック
 *
 */
static bool_t SerCmdTimeout_bComplete(tsSerCmd_Context *pc) {
	bool_t bRet = FALSE;

	if (pc->u8state == E_SERCMD_TIMEOUT_CMD_READPAYLOAD) {
		if (u32TickCount_ms - pc->u32timestamp > pc->u16timeout) {
			// 100ms 経過したら確定
			pc->u8state = E_SERCMD_TIMEOUT_CMD_COMPLETE;
		}
	}

	if (pc->u8state == E_SERCMD_TIMEOUT_CMD_COMPLETE) {
		bRet = TRUE;
	}

	return bRet;
}

/** @ingroup SERCMD
 * 解析構造体を初期化する。
 *
 * @param pc
 * @param pbuff
 * @param u16maxlen
 */
void SerCmdTimeout_vInit(tsSerCmd_Context *pc, uint8 *pbuff, uint16 u16maxlen) {
	memset(pc, 0, sizeof(tsSerCmd_Context));

	pc->au8data = pbuff;
	pc->u16maxlen = u16maxlen;

	pc->u8Parse = SerCmdTimeout_u8Parse;
	pc->vOutput = SerCmdTimeout_Output;
	pc->bComplete = SerCmdTimeout_bComplete;
}

