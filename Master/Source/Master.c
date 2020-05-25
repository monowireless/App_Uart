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
#include <AppHardwareApi.h>

#include "Master.h"

#include "config.h"
#include "ccitt8.h"
#include "Interrupt.h"

#include "Version.h"

#include "utils.h"
#include "input_string.h"

#include "flash.h"

#include "common.h"
#include "config.h"

// IO Read Options
#include "btnMgr.h"

// 重複チェッカ
#include "duplicate_checker.h"

// Serial options
#include <serial.h>
#include <fprintf.h>
#include <sprintf.h>

#include "sercmd_plus3.h"
#include "sercmd_gen.h"

/****************************************************************************/
/***        ToCoNet Definitions                                           ***/
/****************************************************************************/
// Select Modules (define befor include "ToCoNet.h")
#define ToCoNet_USE_MOD_RXQUEUE_BIG
#define ToCoNet_USE_MOD_CHANNEL_MGR

#ifdef NWK_LAYER
#define ToCoNet_USE_MOD_NWK_LAYERTREE
#define ToCoNet_USE_MOD_NBSCAN
#define ToCoNet_USE_MOD_NBSCAN_SLAVE
#endif

// includes
#include "ToCoNet.h"
#include "ToCoNet_mod_prototype.h"

#include "app_event.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);

static void vInitHardware(int f_warm_start);

static void vSerialInit(uint32, tsUartOpt *);
static void vProcessSerialCmd(tsSerCmd_Context *pSer);
static void vProcessSerialCmd_TransmitEx(tsSerCmd_Context *pSer);
static void vProcessInputByte(uint8 u8Byte);
static void vProcessInputByte_Transparent(uint8 u8Byte);
static void vProcessInputByte_FormatCmd(uint8 u8Byte);
static void vProcessInputString(tsInpStr_Context *pContext);
static void vHandleSerialInput();
static void vSerUpdateScreen();
static void vSerChatPrompt();

static void vSerResp_Ack(uint8 u8Status);
static void vSerResp_TxEx(uint8 u8RspId, uint8 u8Status);
static void vSerResp_GetModuleAddress();

static void vReceiveSerMsg(tsRxDataApp *pRx);
//static void vReceiveSerMsgAck(tsRxDataApp *pRx);

static int16 i16TransmitSerMsg(uint8 *p, uint16 u16len, tsTxDataApp *pTxTemplate,
		uint32 u32AddrSrc, uint8 u8AddrSrc, uint32 u32AddrDst, uint8 u8AddrDst,
		bool_t bRelay, uint8 u8Req, uint8 u8RspId);

static void vConfig_SetDefaults(tsFlashApp *p);
static void vConfig_UnSetAll(tsFlashApp *p);
static void vConfig_SaveAndReset();

#ifdef USE_DIO_SLEEP
static void vSleep(uint32 u32SleepDur_ms, bool_t bPeriodic, bool_t bDeep);
#endif


/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
static tsAppData sAppData; //!< アプリケーションデータ  @ingroup MASTER

tsFILE sSerStream; //!< シリアル出力ストリーム  @ingroup MASTER
tsSerialPortSetup sSerPort; //!< シリアルポートの設定  @ingroup MASTER

tsSerCmdPlus3_Context sSerCmd_P3; //!< シリアル入力系列のパーサー (+ + +)  @ingroup MASTER
tsSerCmd_Context sSerCmd; //!< シリアル入力系列のパーサー   @ingroup MASTER

tsSerSeq sSerSeqTx; //!< 分割パケット管理構造体（送信用）  @ingroup MASTER
uint8 au8SerBuffTx[SERCMD_MAXPAYLOAD+32]; //!< sSerSeqTx 用に確保  @ingroup MASTER

tsSerSeq sSerSeqRx; //!< 分割パケット管理構造体（受信用）  @ingroup MASTER
uint8 _au8SerBuffRx[SERCMD_MAXPAYLOAD+32]; //!< sSerSeqRx 用に確保  @ingroup MASTER
uint8 * au8SerBuffRx = _au8SerBuffRx + 16; //!< 後ろに書き込めるようにバッファの先頭をずらす  @ingroup MASTER
#define SIZEOF_au8SerBuffRx (sizeof(_au8SerBuffRx) - 16) //!< sizeof が使えないので、マクロ定義

tsSerCmd_Context sSerCmdOut; //!< シリアル出力用   @ingroup MASTER

tsInpStr_Context sSerInpStr; //!< 文字列入力  @ingroup MASTER
static uint16 u16HoldUpdateScreen = 0; //!< スクリーンアップデートを行う遅延カウンタ  @ingroup MASTER

tsTimerContext sTimerApp; //!< タイマー管理構造体  @ingroup MASTER

uint8 au8SerOutBuff[128]; //!< シリアルの出力書式のための暫定バッファ  @ingroup MASTER
tsSerCmd_Context sSerCmdTemp; //!< シリアル出力用ー   @ingroup MASTER

tsDupChk_Context sDupChk_IoData; //!< 重複チェック(IO関連のデータ転送)  @ingroup MASTER
tsDupChk_Context sDupChk_SerMsg; //!< 重複チェック(シリアル関連のデータ転送)  @ingroup MASTER

uint8 au8TxCbId_to_RespID[256]; //!< 送信パケットのコールバックIDから RespID を紐づける表 @ingroup MASTER

/** @ingroup MASTER
 * UARTモードによる送信種別仕訳
 * (Transparent, chat, binary/ascii が混在しないように)
 */
const uint8 au8UartModeToTxCmdId[] = {
	0,    // TRANSPARENT
	1, 1, // ASCII, BINARY
	2, 2  // チャットモード
};

/****************************************************************************/
/***        FUNCTIONS                                                     ***/
/****************************************************************************/

/** @ingroup MASTER
 * 始動時メッセージの表示
 */
static void vSerInitMessage() {
	// 始動メッセージ
	if (sAppData.sFlash.sData.u8uart_mode == UART_MODE_CHAT || sAppData.sFlash.sData.u8uart_mode == UART_MODE_CHAT_NO_PROMPT) {
		vfPrintf(&sSerStream, LB"!INF TWE UART APP V%d-%02d-%d, SID=0x%08X, LID=0x%02x",
				VERSION_MAIN, VERSION_SUB, VERSION_VAR, ToCoNet_u32GetSerial(), sAppData.u8AppLogicalId);
		vSerChatPrompt();
	} else
	if (sAppData.sFlash.sData.u8uart_mode == UART_MODE_ASCII || sAppData.sFlash.sData.u8uart_mode == UART_MODE_BINARY) {
		// ASCII, BINARY モードでは自分のアドレスを表示する
		vSerResp_GetModuleAddress();
	}
}

/**  @ingroup MASTER
 * Chatモードのプロンプト表示
 */
static void vSerChatPrompt() {
	if (sAppData.sFlash.sData.u8uart_mode == UART_MODE_CHAT) {
		if (sAppData.sFlash.sData.au8ChatHandleName[0] == 0x00) {
			vfPrintf(&sSerStream, LB"%08X:%d> ", ToCoNet_u32GetSerial(), sAppData.u8UartReqNum);
		} else {
			vfPrintf(&sSerStream, LB"%s:%d> ", sAppData.sFlash.sData.au8ChatHandleName, sAppData.u8UartReqNum);
		}

		// 入力中なら表示を回復する
		if (sSerCmd.u8state < 0x80 && sSerCmd.u8state != E_SERCMD_EMPTY) {
			sSerCmd.vOutput(&sSerCmd, &sSerStream);
		}
	}
}

/** @ingroup MASTER
 * アプリケーションの基本制御状態マシン。
 * - 特別な処理は無いが、ネットワーク層を利用したコードに改造する場合には、ここでネットワークの初期化など
 *   を実行する。
 *
 * @param pEv
 * @param eEvent
 * @param u32evarg
 */
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
	switch (pEv->eState) {
	case E_STATE_IDLE:
		if (eEvent == E_EVENT_START_UP) {
			// 暗号化キーを登録する
#ifdef USE_AES
			if (IS_CRYPT_MODE()) {
				ToCoNet_bRegisterAesKey((void*)(sAppData.sFlash.sData.au8AesKey), NULL);
			}
#endif

			if (IS_APPCONF_ROLE_SILENT_MODE()) {
				vfPrintf(&sSerStream, LB"!Note: launch silent mode."LB);
				ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
			} else if (sAppData.eNwkMode == E_NWKMODE_LAYERTREE) {
				// layer tree の実装
#ifdef NWK_LAYER
				switch(sAppData.u8Mode) {
				case E_IO_MODE_PARNET:
					sAppData.sNwkLayerTreeConfig.u8Role = TOCONET_NWK_ROLE_PARENT;
					break;
				case E_IO_MODE_CHILD:
				case E_IO_MODE_REPEATER:
					sAppData.sNwkLayerTreeConfig.u8Role = TOCONET_NWK_ROLE_ROUTER;
					sAppData.sNwkLayerTreeConfig.u8Layer = sAppData.sFlash.sData.u8layer;
					break;
				default: break;
				}

				//sAppData.sNwkLayerTreeConfig.u16TxMaxDelayDn_ms = 30;
					// 親機から子機へのパケットは Ack 無しブロードキャスト４回送信を行うが、
					// 中継機もこのパケットを同様に４回送信する。このため、複数の中継機が
					// 同一電波範囲内にある事を想定してデフォルトは大きな値になっている。
					// 本アプリは中継機無しを想定しているので、値を小さくし応答性を重視する。

				// 設定を行う
				sAppData.pContextNwk = ToCoNet_NwkLyTr_psConfig(&sAppData.sNwkLayerTreeConfig);

				if (sAppData.pContextNwk) {
					// ネットワークの開始
					ToCoNet_Nwk_bInit(sAppData.pContextNwk);
					ToCoNet_Nwk_bStart(sAppData.pContextNwk);

					sAppData.eNwkMode = E_NWKMODE_LAYERTREE;
					ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
				}
#endif
			} else if (sAppData.eNwkMode == E_NWKMODE_MAC_DIRECT) {
				// LayerNetwork で無ければ、特別な動作は不要。
				// run as default...
				if (!(u32evarg & EVARG_START_UP_WAKEUP_MASK)) {
					vSerInitMessage(); // SKIP!
				}

				ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
			}
		}
		break;

	case E_STATE_RUNNING:

		break;
	default:
		break;
	}
}

/** @ingroup MASTER
 * 電源投入時・リセット時に最初に実行される処理。本関数は２回呼び出される。初回は u32AHI_Init()前、
 * ２回目は AHI 初期化後である。
 *
 * - 各種初期化
 * - ToCoNet ネットワーク設定
 * - 設定IO読み取り
 * - 設定値の計算
 * - ハードウェア初期化
 * - イベントマシンの登録
 * - 本関数終了後は登録したイベントマシン、および cbToCoNet_vMain() など各種コールバック関数が
 *   呼び出される。
 *
 * @param bStart TRUE:u32AHI_Init() 前の呼び出し FALSE: 後
 */
void cbAppColdStart(bool_t bStart) {
	if (!bStart) {
		// before AHI initialization (very first of code)

		// Module Registration
		ToCoNet_REG_MOD_ALL();
	} else {
		// clear application context
		memset(&sAppData, 0x00, sizeof(sAppData));

		// configure network
		sToCoNet_AppContext.u8TxMacRetry = 3; // MAC再送回数（JN516x では変更できない）
		//sToCoNet_AppContext.bRxOnIdle = TRUE; // TRUE:受信回路をオープンする
		sToCoNet_AppContext.u32AppId = APP_ID; // アプリケーションID
		sToCoNet_AppContext.u32ChMask = CHMASK; // 利用するチャネル群（最大３つまで）
		sToCoNet_AppContext.u8Channel = CHANNEL;

		// configuration
		vConfig_UnSetAll(&sAppData.sConfig_UnSaved);

		// load flash value
		sAppData.bFlashLoaded = bFlash_Read(&sAppData.sFlash, FLASH_SECTOR_NUMBER - 1, 0);
		if (sAppData.bFlashLoaded &&
			(   sAppData.sFlash.sData.u32appkey != APP_ID
			 || (sAppData.sFlash.sData.u32ver !=
					((VERSION_MAIN<<16) | (VERSION_SUB<<8) | (VERSION_VAR))))
			) {
			sAppData.bFlashLoaded = FALSE;
		}

		if (sAppData.bFlashLoaded) {
			sToCoNet_AppContext.u32AppId = sAppData.sFlash.sData.u32appid;
			// sToCoNet_AppContext.u8Channel = sAppData.sFlash.sData.u8ch; // チャネルマネージャで決定するので設定不要
			sToCoNet_AppContext.u32ChMask = sAppData.sFlash.sData.u32chmask;
			sToCoNet_AppContext.u8TxPower = sAppData.sFlash.sData.u8power;

			if (   sAppData.sFlash.sData.u8role == E_APPCONF_ROLE_MAC_NODE
				|| sAppData.sFlash.sData.u8role == E_APPCONF_ROLE_MAC_NODE_REPEATER) {
				sAppData.eNwkMode = E_NWKMODE_MAC_DIRECT;
			} else
			if (sAppData.sFlash.sData.u8role & E_APPCONF_ROLE_NWK_MASK) {
				sAppData.eNwkMode = E_NWKMODE_LAYERTREE;
			} else
			if (sAppData.sFlash.sData.u8role == E_APPCONF_ROLE_SILENT) {
				sAppData.eNwkMode = E_NWKMODE_MAC_DIRECT;
			} else {
				sAppData.bFlashLoaded = FALSE;
			}
		}

		if (sAppData.bFlashLoaded != TRUE) {
			// デフォルト値を格納する
			vConfig_SetDefaults(&(sAppData.sFlash.sData));
		}

		// ヘッダの１バイト識別子を AppID から計算
		sAppData.u8AppIdentifier = u8CCITT8((uint8*)&sToCoNet_AppContext.u32AppId, 4) & 0xFE;
			// APP ID の CRC8 (下１ビットは中継ビット)

		// IOより状態を読み取る (ID など)
		//sAppData.u32DIO_startup = ~(u32PortReadBitmap()); // この時点では全部入力ポート

		// version info
		sAppData.u32ToCoNetVersion = ToCoNet_u32GetVersion();

		// ToCoNet の制御 Tick [ms]
		sAppData.u16ToCoNetTickDelta_ms = 1000 / sToCoNet_AppContext.u16TickHz;

		// other hardware
		vInitHardware(FALSE);

		// 論理IDをフラッシュ設定した場合
		if (sAppData.bFlashLoaded) {
			// 子機
			if (IS_LOGICAL_ID_CHILD(sAppData.sFlash.sData.u8id)) {
				sAppData.u8Mode = E_IO_MODE_CHILD;
			}
			// 親機
			if (sAppData.sFlash.sData.u8id == 121) {
				sAppData.u8Mode = E_IO_MODE_PARNET; // 親機のモード番号
			}
			// 中継機
			if (sAppData.sFlash.sData.u8id == LOGICAL_ID_REPEATER) {
				sAppData.u8Mode = E_IO_MODE_REPEATER; // 親機のモード番号
			}
		}

#ifdef NWK_LAYER_FORCE
		// 強制的に Layer Tree で起動する
		sAppData.eNwkMode = E_NWKMODE_LAYERTREE;
#endif

		// モードごとの独自設定
		sAppData.u8AppLogicalId = 0; // 最初に０にしておく
		switch(sAppData.u8Mode) {
		case E_IO_MODE_PARNET:
			sAppData.u8AppLogicalId = LOGICAL_ID_PARENT;
#ifdef NWK_LAYER_FORCE
			sAppData.sFlash.sData.u8role = E_APPCONF_ROLE_PARENT;
#endif
			break;

		case E_IO_MODE_REPEATER:
			sAppData.u8AppLogicalId = LOGICAL_ID_REPEATER;
			sAppData.sFlash.sData.u8role = E_APPCONF_ROLE_MAC_NODE_REPEATER;
			break;

		case E_IO_MODE_CHILD:
		case E_IO_MODE_REPEAT_CHILD:
			// 子機IDはフラッシュ値が設定されていれば、これを採用
			if (sAppData.bFlashLoaded) {
				sAppData.u8AppLogicalId = sAppData.sFlash.sData.u8id;
			}
			// 値が子機のID範囲外ならデフォルト値を設定する (120=0x78)
			if (!IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId)) {
				sAppData.u8AppLogicalId = au8IoModeTbl_To_LogicalID[E_IO_MODE_CHILD];
			}

#ifndef NWK_LAYER
			if (sAppData.u8Mode == E_IO_MODE_REPEAT_CHILD) {
				// フラッシュの設定ではなく IO により中継の枠割を割り当てる
				sAppData.sFlash.sData.u8role = E_APPCONF_ROLE_MAC_NODE_REPEATER;
			}
#endif

#ifdef NWK_LAYER_FORCE
			sAppData.sFlash.sData.u8role = E_APPCONF_ROLE_ROUTER;
#endif
			break;

		default: // 未定義機能なので、SILENT モードにする。
			sAppData.u8AppLogicalId = 255;
			sAppData.sFlash.sData.u8role = E_APPCONF_ROLE_SILENT;
			break;
		}

		// ショートアドレスの設定(決めうち)
		sToCoNet_AppContext.u16ShortAddress = SERCMD_ADDR_CONV_TO_SHORT_ADDR(sAppData.u8AppLogicalId);

		// UART の初期化
		ToCoNet_vDebugInit(&sSerStream);
		ToCoNet_vDebugLevel(0);

		// その他の初期化
		DUPCHK_vInit(&sDupChk_IoData);
		DUPCHK_vInit(&sDupChk_SerMsg);

		if (!(IS_APPCONF_ROLE_SILENT_MODE())) {
			// メインアプリケーション処理部
			switch(sAppData.u8Mode) {
			case E_IO_MODE_PARNET:
			case E_IO_MODE_REPEATER:
			case E_IO_MODE_CHILD:
				sToCoNet_AppContext.bRxOnIdle = TRUE;
				break;

			default: // 未定義機能なので、SILENT モードにする。
				sToCoNet_AppContext.bRxOnIdle = FALSE;
				break;
			}

			// MAC start
			ToCoNet_vMacStart();

			// event machine
			ToCoNet_Event_Register_State_Machine(vProcessEvCore); // main state machine
		}
	}
}

/** @ingroup MASTER
 * スリープ復帰後に呼び出される関数。本関数も cbAppColdStart() と同様に２回呼び出され、u32AHI_Init() 前の
 * 初回呼び出しに於いて、スリープ復帰要因を判定している。u32AHI_Init() 関数はこれらのレジスタを初期化してしまう。
 *
 * - 変数の初期化（必要なもののみ）
 * - ハードウェアの初期化（スリープ後は基本的に再初期化が必要）
 * - イベントマシンは登録済み。
 *
 * @param bStart TRUE:u32AHI_Init() 前の呼び出し FALSE: 後
 */
void cbAppWarmStart(bool_t bStart) {
	if (!bStart) {
		// before AHI init, very first of code.
		//  to check interrupt source, etc.

		sAppData.bWakeupByButton = FALSE;
		if(u8AHI_WakeTimerFiredStatus()) {
			;
		} else
		if(u32AHI_DioWakeStatus() & ((1UL << PORT_INPUT1) | (1UL << PORT_INPUT2) | (1UL << PORT_INPUT3) | (1UL << PORT_INPUT4)) ) {
			// woke up from DIO events
			sAppData.bWakeupByButton = TRUE;
		}

	} else {
		// 変数の初期化（必要なものだけ）
		sAppData.u16CtTimer0 = 0; // このカウンタは、起動時からのカウントとする

		// other hardware
		vInitHardware(TRUE);

		// UART の初期化
		ToCoNet_vDebugInit(&sSerStream);
		ToCoNet_vDebugLevel(0);

		// その他の初期化
		DUPCHK_vInit(&sDupChk_IoData);
		DUPCHK_vInit(&sDupChk_SerMsg);

		// MAC start
		ToCoNet_vMacStart();
	}
}

/** @ingroup MASTER
 * 本関数は ToCoNet のメインループ内で必ず１回は呼び出される。
 * ToCoNet のメインループでは、CPU DOZE 命令を発行しているため、割り込みなどが発生した時に呼び出され、
 * かつ TICK TIMER の割り込みは定期的に発生しているため、定期処理としても使用可能である。
 *
 * - シリアルの入力チェック
 */
void cbToCoNet_vMain(void) {
	/* handle serial input */
	vHandleSerialInput();
}

/** @ingroup MASTER
 * パケットの受信処理。パケットの種別によって具体的な処理関数にディスパッチしている。
 * @param psRx 受信パケット
 */
void cbToCoNet_vRxEvent(tsRxDataApp *psRx) {
	//uint8 *p = pRx->auData;
	DBGOUT(3, "Rx packet (cm:%02x, fr:%08x, to:%08x)"LB, psRx->u8Cmd, psRx->u32SrcAddr, psRx->u32DstAddr);

	if (IS_APPCONF_ROLE_SILENT_MODE()) {
		// SILENT, 1秒スリープ, 10秒スリープでは受信処理はしない。
		return;
	}

	// UART モードに合致するパケット以外は受け付けない
	if (psRx->u8Cmd == au8UartModeToTxCmdId[sAppData.sFlash.sData.u8uart_mode]) {
		DBGOUT(1, "Rx Data (%d[%d],%08x,%08x)"LB,
			psRx->u8Cmd,
			au8UartModeToTxCmdId[sAppData.sFlash.sData.u8uart_mode],
			psRx->u32SrcAddr,
			psRx->u32DstAddr);
		vReceiveSerMsg(psRx);
	}
}


/** @ingroup MASTER
 * 送信完了時に発生する。
 *
 * - IO 送信完了イベントはイベントマシンにイベントを伝達する。
 * - シリアルメッセージの一連のパケット群の送信完了も検出している。
 *
 * @param u8CbId 送信時に設定したコールバックID
 * @param bStatus 送信ステータス
 */
void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus) {
	//uint8 *q = au8SerOutBuff;
	if (IS_APPCONF_ROLE_SILENT_MODE()) {
		return;
	}

	// UART 送信の完了チェック
	bool_t bHandleMsg = FALSE;
	if (sSerSeqTx.bWaitComplete) {
		uint8 idx = (u8CbId & CBID_MASK_BASE) - sSerSeqTx.u8Seq;

		if (idx < sSerSeqTx.u8PktNum) {
			bHandleMsg = TRUE;

			if (bStatus) {
				sSerSeqTx.bPktStatus[idx] = 1;
			} else {
				if (sSerSeqTx.bPktStatus[idx] == 0) {
					sSerSeqTx.bPktStatus[idx] = -1;
				}
			}

			int i, isum = 0;
			for (i = 0; i < sSerSeqTx.u8PktNum; i++) {
				if (sSerSeqTx.bPktStatus[i] == 0) break;
				isum += sSerSeqTx.bPktStatus[i];
			}

			if (i == sSerSeqTx.u8PktNum) {
				/* 送信完了 (MAC レベルで成功した) */
				sSerSeqTx.bWaitComplete = FALSE;

				// VERBOSE MESSAGE
				DBGOUT(3, "* >>> MacTxFin%s(tick=%d,req=#%d) <<<" LB,
						(isum == sSerSeqTx.u8PktNum) ? "" : "Fail",
						u32TickCount_ms & 65535,
						sSerSeqTx.u8ReqNum
						);

				// 応答を返す
				if ((sAppData.sFlash.sData.u8uart_mode == UART_MODE_ASCII || sAppData.sFlash.sData.u8uart_mode == UART_MODE_BINARY) && !(u8CbId & CBID_MASK_SILENT)) {
					vSerResp_TxEx(sSerSeqTx.u8RespID, isum == sSerSeqTx.u8PktNum);
				}
			}
		}
	}

	// 単独パケットで併行送信する場合(bWaitCompleteフラグは立たない)の応答
	if (!bHandleMsg && (sAppData.sFlash.sData.u8uart_mode == UART_MODE_ASCII || sAppData.sFlash.sData.u8uart_mode == UART_MODE_BINARY)) {
		// 応答を返す
		if (!(u8CbId & CBID_MASK_SPLIT_PKTS) && !(u8CbId & CBID_MASK_SILENT)) { // 分割パケットおよびリピートパケットは処理しない
			vSerResp_TxEx(au8TxCbId_to_RespID[u8CbId], bStatus);
		}
	}

	return;
}

/** @ingroup MASTER
 * ネットワーク層などのイベントが通達される。
 * 本アプリケーションでは特別な処理は行っていない。
 *
 * @param ev
 * @param u32evarg
 */
void cbToCoNet_vNwkEvent(teEvent ev, uint32 u32evarg) {
	if (IS_APPCONF_ROLE_SILENT_MODE()) {
		return;
	}

	switch(ev) {
	case E_EVENT_TOCONET_NWK_START:
		break;

	case E_EVENT_TOCONET_NWK_DISCONNECT:
		break;

	default:
		break;
	}
}

/** @ingroup MASTER
 * ハードウェア割り込み時に呼び出される。本処理は割り込みハンドラではなく、割り込みハンドラに登録された遅延実行部による処理で、長い処理が記述可能である。
 * 本アプリケーションに於いては、ADC/DIの入力状態のチェック、64fps のタイマーイベントの処理などを行っている。
 *
 * - E_AHI_DEVICE_TICK_TIMER
 *   - ADC の完了確認
 *   - DI の変化のチェック
 *   - イベントマシンに TIMER0 イベントを発行
 *   - インタラクティブモード時の画面再描画
 *
 * @param u32DeviceId
 * @param u32ItemBitmap
 */
void cbToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap) {
	switch (u32DeviceId) {
	case E_AHI_DEVICE_ANALOGUE: //ADC完了時にこのイベントが発生する。
		break;

	case E_AHI_DEVICE_TICK_TIMER: //頻繁なので、TICKTIMER は処理しない。
		break;

	case E_AHI_DEVICE_TIMER0:
		// タイマーカウンタをインクリメントする (64fps なので 64カウントごとに１秒)
		sAppData.u32CtTimer0++;
		sAppData.u16CtTimer0++;

		// 重複チェックのタイムアウト処理
		if ((sAppData.u32CtTimer0 & 0xF) == 0) {
			DUPCHK_bFind(&sDupChk_IoData, 0, NULL);
			DUPCHK_bFind(&sDupChk_SerMsg, 0, NULL);
		}

		// 送信処理のタイムアウト処理
		if (sSerSeqTx.bWaitComplete) {
			if (u32TickCount_ms - sSerSeqTx.u32Tick > 1000) {
				// タイムアウトとして、処理を続行
				memset(&sSerSeqTx, 0, sizeof(sSerSeqTx));
			}
		}

		// ボタンの変化
		if (u32TickCount_ms - sAppData.u32BTM_Tick_LastChange > 100) // 前回変化から 100ms 以上経過している事
		{
			uint32 bmPorts, bmChanged;
			if (bBTM_GetState(&bmPorts, &bmChanged)) {
				// PORT_INPUT* の変化検出
				if (   (sAppData.sFlash.sData.u8uart_mode == UART_MODE_TRANSPARENT) // 送信モードが透過モード
					&& (bmChanged & PORT_INPUT_MASK)) // I1-I4の入力が有った
				{
					sAppData.u8PortNow =
								(bmPorts & (1UL << PORT_INPUT1)) ? 1 : 0
							|	(bmPorts & (1UL << PORT_INPUT2)) ? 2 : 0
							|	(bmPorts & (1UL << PORT_INPUT3)) ? 4 : 0
							|	(bmPorts & (1UL << PORT_INPUT4)) ? 8 : 0;

					if (IS_LOGICAL_ID_PARENT(sAppData.u8AppLogicalId)) {
						// ショートアドレスの再設定
						sToCoNet_AppContext.u16ShortAddress =
								SERCMD_ADDR_CONV_TO_SHORT_ADDR_PARENT_IN_PAIR(sAppData.u8PortNow);
						ToCoNet_vRfConfig();

						// 相手方のアドレスを設定
						sAppData.u8AppLogicalId_Pair = sAppData.u8PortNow + 101;
						sAppData.u16ShAddr_Pair =
								SERCMD_ADDR_CONV_TO_SHORT_ADDR_CHILD_IN_PAIR(sAppData.u8PortNow);
					} else if (IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId)) {
						// ショートアドレスの再設定
						sToCoNet_AppContext.u16ShortAddress =
								SERCMD_ADDR_CONV_TO_SHORT_ADDR_CHILD_IN_PAIR(sAppData.u8PortNow);
						ToCoNet_vRfConfig();

						// 自分のアプリケーションアドレスを設定
						sAppData.u8AppLogicalId = sAppData.u8PortNow + 101;

						// 相手方のアドレスを設定
						sAppData.u8AppLogicalId_Pair = 0;
						sAppData.u16ShAddr_Pair =
								SERCMD_ADDR_CONV_TO_SHORT_ADDR_PARENT_IN_PAIR(sAppData.u8PortNow);
					} else {
						// do nothing. should not be here!
					}

				}

#ifdef USE_DIO_SLEEP
				// PORT_SLEEPの変化検出 (チャタリング期間が過ぎてから）
				if (bmChanged & (1UL << PORT_SLEEP)) {
					if (bmPorts & (1UL << PORT_SLEEP)) {
						// ポートPORT_SLEEPが Lo になったときスリープ遷移する
						vPortDisablePullup(PORT_SLEEP);
						vSleep(0, FALSE, FALSE);
					}
				}
#endif

				sAppData.u32BTM_Tick_LastChange = u32TickCount_ms;
			}
		}

		// 透過モードでの定期送信を行う
		if (	sAppData.sFlash.sData.u8uart_mode == UART_MODE_TRANSPARENT // 送信モードが透過モード
			&&	sAppData.u16ShAddr_Pair != 0 // 送信先が確定している事
			&&	sSerCmd.u16len // データが有る事
			&&	!sSerSeqTx.bWaitComplete // 送信中でない事
		) {
			// 送信
			if (i16TransmitSerMsg(
					sSerCmd.au8data,
					sSerCmd.u16len,
					NULL,
					ToCoNet_u32GetSerial(),
					sAppData.u8AppLogicalId,
					sAppData.u16ShAddr_Pair,
					sAppData.u8AppLogicalId_Pair,
					FALSE,
					sAppData.u8UartReqNum,
					sAppData.u8UartReqNum) != -1) {

				// 送信出来たのでバッファ長さを巻き戻す
				sSerCmd.u16len = 0;
			}
			sAppData.u8UartReqNum++;
		}

		// シリアル画面制御のためのカウンタ
		if (sSerCmd_P3.bverbose && u16HoldUpdateScreen) {
			if (!(--u16HoldUpdateScreen)) {
				vSerUpdateScreen();
			}
		}
		break;

	default:
		break;
	}
}

/** @ingroup MASTER
 * 割り込みハンドラ。ここでは長い処理は記述してはいけない。
 *
 * - TICK_TIMER 起点で ADC の稼働、ボタンの連照処理を実行する
 */
PUBLIC uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap) {
	uint8 u8handled = FALSE;

	switch (u32DeviceId) {
	case E_AHI_DEVICE_TIMER0:
		break;

	case E_AHI_DEVICE_TICK_TIMER:
		// ボタンハンドラの駆動
		if (sAppData.pr_BTM_handler) {
			// ハンドラを稼働させる
			(*sAppData.pr_BTM_handler)(sAppData.u16ToCoNetTickDelta_ms);
		}

		break;

	default:
		break;
	}

	return u8handled;
}

/** @ingroup MASTER
 * * ハードウェアの初期化
 * @param f_warm_start TRUE:スリープ復帰時
 */
PRIVATE void vInitHardware(int f_warm_start) {
	int i;

	// メモリのクリア
	memset(&sTimerApp, 0, sizeof(tsTimerContext));

	// 入力の設定
	for (i = 0; i < 4; i++) {
		vPortAsInput(au8PortTbl_DIn[i]);
	}
#ifdef USE_MODE_PIN
	// モード設定
	vPortAsInput(PORT_CONF1);
	vPortAsInput(PORT_CONF2);
	vPortAsInput(PORT_CONF3);
	sAppData.u8Mode = (bPortRead(PORT_CONF1) | (bPortRead(PORT_CONF2) << 1)); // M1,M2 を設定用として読みだす。
#else
	sAppData.u8Mode = 0; // 0 決め打ち
#endif

	// UART 設定
	{
#ifdef USE_BPS_PIN
		vPortAsInput(PORT_BAUD);
		uint32 u32baud = bPortRead(PORT_BAUD) ? UART_BAUD_SAFE : UART_BAUD;
#else
		uint32 u32baud = UART_BAUD;
#endif
		tsUartOpt sUartOpt;

		memset(&sUartOpt, 0, sizeof(tsUartOpt));

		// BAUD ピンが GND になっている場合、かつフラッシュの設定が有効な場合は、設定値を採用する (v1.0.3)
#ifdef USE_BPS_PIN
		if (sAppData.bFlashLoaded && bPortRead(PORT_BAUD)) {
			u32baud = sAppData.sFlash.sData.u32baud_safe;
			sUartOpt.bHwFlowEnabled = FALSE;
			sUartOpt.bParityEnabled = UART_PARITY_ENABLE;
			sUartOpt.u8ParityType = UART_PARITY_TYPE;
			sUartOpt.u8StopBit = UART_STOPBITS;

			// 設定されている場合は、設定値を採用する (v1.0.3)
			switch(sAppData.sFlash.sData.u8parity) {
			case 0:
				sUartOpt.bParityEnabled = FALSE;
				break;
			case 1:
				sUartOpt.bParityEnabled = TRUE;
				sUartOpt.u8ParityType = E_AHI_UART_ODD_PARITY;
				break;
			case 2:
				sUartOpt.bParityEnabled = TRUE;
				sUartOpt.u8ParityType = E_AHI_UART_EVEN_PARITY;
				break;
			}

			vSerialInit(u32baud, &sUartOpt);
		} else
#endif
		{
			vSerialInit(u32baud, NULL);
		}
	}

	// タイマの未使用ポートの解放（汎用ＩＯとして使用するため）
#ifdef JN516x
	vAHI_TimerFineGrainDIOControl(0x7F); // タイマー関連のピンを使わない
#else
	vAHI_TimerFineGrainDIOControl(0x7F); // タイマー関連のピンを使わない
#endif

	// メイン(64fps)タイマ管理構造体の初期化
	memset(&sTimerApp, 0, sizeof(sTimerApp));

	// activate tick timers
	sTimerApp.u8Device = E_AHI_DEVICE_TIMER0;
	sTimerApp.u16Hz = 64;
	sTimerApp.u8PreScale = 4; // 15625ct@2^4
	vTimerConfig(&sTimerApp);
	vTimerStart(&sTimerApp);

	// button Manager (for Input)
#ifdef USE_DIO_SLEEP
	sAppData.sBTM_Config.bmPortMask =
			  (1UL << PORT_INPUT1) | (1UL << PORT_INPUT2)
			| (1UL << PORT_INPUT3) | (1UL << PORT_INPUT4)
			| (1UL << PORT_SLEEP);
#else
	sAppData.sBTM_Config.bmPortMask =
			  (1UL << PORT_INPUT1) | (1UL << PORT_INPUT2)
			| (1UL << PORT_INPUT3) | (1UL << PORT_INPUT4);
#endif
	sAppData.sBTM_Config.u16Tick_ms = 8;
	sAppData.sBTM_Config.u8MaxHistory = 5;
	sAppData.sBTM_Config.u8DeviceTimer = 0xFF; // TickTimer を流用する。
	sAppData.pr_BTM_handler = prBTM_InitExternal(&sAppData.sBTM_Config);
	vBTM_Enable();
}

/** @ingroup MASTER
 * UART を初期化する
 * @param u32Baud ボーレート
 */
void vSerialInit(uint32 u32Baud, tsUartOpt *pUartOpt) {
	/* Create the debug port transmit and receive queues */
	static uint8 au8SerialTxBuffer[1536];
	static uint8 au8SerialRxBuffer[1536];

	/* Initialise the serial port to be used for debug output */
	sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
	sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
	sSerPort.u32BaudRate = u32Baud;
	sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
	sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
	sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
	sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
	sSerPort.u8SerialPort = UART_PORT_MASTER;
	sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
	SERIAL_vInitEx(&sSerPort, pUartOpt);

	/* prepare stream for vfPrintf */
	sSerStream.bPutChar = SERIAL_bTxChar;
	sSerStream.u8Device = UART_PORT_MASTER;

	/* other initialization */
	INPSTR_vInit(&sSerInpStr, &sSerStream);
	memset(&sSerCmd_P3, 0x00, sizeof(sSerCmd));
	memset(&sSerCmd, 0x00, sizeof(sSerCmd));
	memset(&sSerSeqTx, 0x00, sizeof(sSerSeqTx));
	memset(&sSerSeqRx, 0x00, sizeof(sSerSeqRx));
	memset(&sSerCmdOut, 0x00, sizeof(sSerCmd));

	memset(&sSerCmdTemp, 0x00, sizeof(sSerCmd));

	memset(au8TxCbId_to_RespID, 0x00, sizeof(au8TxCbId_to_RespID));

	// シリアルコマンド解釈部分の初期化
	switch(sAppData.sFlash.sData.u8uart_mode) {
	case UART_MODE_CHAT: case UART_MODE_CHAT_NO_PROMPT:
		SerCmdChat_vInit(&sSerCmd, au8SerBuffTx, sizeof(au8SerBuffTx));
		SerCmdChat_vInit(&sSerCmdOut, au8SerBuffRx, SIZEOF_au8SerBuffRx);
		SerCmdChat_vInit(&sSerCmdTemp, au8SerOutBuff, sizeof(au8SerOutBuff));
		break;
	case UART_MODE_BINARY:
		SerCmdBinary_vInit(&sSerCmd, au8SerBuffTx, sizeof(au8SerBuffTx));
		SerCmdBinary_vInit(&sSerCmdOut, au8SerBuffRx, SIZEOF_au8SerBuffRx);
		SerCmdBinary_vInit(&sSerCmdTemp, au8SerOutBuff, sizeof(au8SerOutBuff));
		break;
	case UART_MODE_ASCII: case UART_MODE_TRANSPARENT:
		SerCmdAscii_vInit(&sSerCmd, au8SerBuffTx, sizeof(au8SerBuffTx));
		SerCmdAscii_vInit(&sSerCmdOut, au8SerBuffRx, SIZEOF_au8SerBuffRx);
		SerCmdAscii_vInit(&sSerCmdTemp, au8SerOutBuff, sizeof(au8SerOutBuff));
		break;
	}
	sSerCmd.u16timeout = 1000; // デフォルトのタイムアウト
}

/** @ingroup MASTER
 * インタラクティブモードの画面を再描画する。
 */
static void vSerUpdateScreen() {
	V_PRINT("%c[2J%c[H", 27, 27); // CLEAR SCREEN
	V_PRINT("--- CONFIG/TWE UART APP V%d-%02d-%d/SID=0x%08x/LID=0x%02x ---"LB,
			VERSION_MAIN, VERSION_SUB, VERSION_VAR, ToCoNet_u32GetSerial(), sAppData.u8AppLogicalId);

	// Application ID
	V_PRINT(" a: set Application ID (0x%08x)%c" LB,
			FL_IS_MODIFIED_u32(appid) ? FL_UNSAVE_u32(appid) : FL_MASTER_u32(appid),
			FL_IS_MODIFIED_u32(appid) ? '*' : ' ');

	// Device ID
	{
		uint8 u8DevID = FL_IS_MODIFIED_u8(id) ? FL_UNSAVE_u8(id) : FL_MASTER_u8(id);

		if (u8DevID == 0x00) { // unset
			V_PRINT(" i: set Device ID (--)%c"LB,
					FL_IS_MODIFIED_u8(id) ? '*' : ' '
					);
		} else {
			V_PRINT(" i: set Device ID (%d=0x%02x)%c"LB,
					u8DevID, u8DevID,
					FL_IS_MODIFIED_u8(id) ? '*' : ' '
					);
		}
	}

	V_PRINT(" c: set Channels (");
	{
		// find channels in ch_mask
		uint8 au8ch[MAX_CHANNELS], u8ch_idx = 0;
		int i;
		memset(au8ch,0,MAX_CHANNELS);
		uint32 u32mask = FL_IS_MODIFIED_u32(chmask) ? FL_UNSAVE_u32(chmask) : FL_MASTER_u32(chmask);
		for (i = 11; i <= 26; i++) {
			if (u32mask & (1UL << i)) {
				if (u8ch_idx) {
					V_PUTCHAR(',');
				}
				V_PRINT("%d", i);
				au8ch[u8ch_idx++] = i;
			}

			if (u8ch_idx == MAX_CHANNELS) {
				break;
			}
		}
	}
	V_PRINT(")%c" LB,
			FL_IS_MODIFIED_u32(chmask) ? '*' : ' ');

	V_PRINT(" o: set Output Tx Power (%d)%c" LB,
			FL_IS_MODIFIED_u8(power) ? FL_UNSAVE_u8(power) : FL_MASTER_u8(power),
			FL_IS_MODIFIED_u8(power) ? '*' : ' ');

	V_PRINT(" r: set Role (0x%X)%c" LB,
			FL_IS_MODIFIED_u8(role) ? FL_UNSAVE_u8(role) : FL_MASTER_u8(role),
			FL_IS_MODIFIED_u8(role) ? '*' : ' ');


	{
		uint32 u32baud = FL_IS_MODIFIED_u32(baud_safe) ? FL_UNSAVE_u32(baud_safe) : FL_MASTER_u32(baud_safe);
		if (u32baud & 0x80000000) {
			V_PRINT(" b: set UART baud (%x)%c" LB, u32baud,
					FL_IS_MODIFIED_u32(baud_safe) ? '*' : ' ');
		} else {
			V_PRINT(" b: set UART baud (%d)%c" LB, u32baud,
					FL_IS_MODIFIED_u32(baud_safe) ? '*' : ' ');
		}
	}

	{
		const uint8 au8name[] = { 'N', 'O', 'E' };
		V_PRINT(" p: set parity (%c)%c" LB,
					au8name[FL_IS_MODIFIED_u8(parity) ? FL_UNSAVE_u8(parity) : FL_MASTER_u8(parity)],
					FL_IS_MODIFIED_u8(parity) ? '*' : ' ');
	}

	{
		const uint8 au8name[] = { 'T', 'A', 'B', 'C', 'D' };
		V_PRINT(" m: set uart mode (%c)%c" LB,
					au8name[FL_IS_MODIFIED_u8(uart_mode) ? FL_UNSAVE_u8(uart_mode) : FL_MASTER_u8(uart_mode)],
					FL_IS_MODIFIED_u8(uart_mode) ? '*' : ' ');
	}

	{
		V_PRINT(" h: set handle name [%s]%c" LB,
			sAppData.sConfig_UnSaved.au8ChatHandleName[0] == 0xFF ?
				sAppData.sFlash.sData.au8ChatHandleName : sAppData.sConfig_UnSaved.au8ChatHandleName,
			sAppData.sConfig_UnSaved.au8ChatHandleName[0] == 0xFF ? ' ' : '*');
	}

	{
		V_PRINT(" C: set crypt mode (%d)%c" LB,
					FL_IS_MODIFIED_u8(Crypt) ? FL_UNSAVE_u8(Crypt) : FL_MASTER_u8(Crypt),
					FL_IS_MODIFIED_u8(Crypt) ? '*' : ' ');
	}

	{
		V_PRINT(" K: set crypt key [%s]%c" LB,
			sAppData.sConfig_UnSaved.au8AesKey[0] == 0xFF ?
				sAppData.sFlash.sData.au8AesKey : sAppData.sConfig_UnSaved.au8AesKey,
			sAppData.sConfig_UnSaved.au8AesKey[0] == 0xFF ? ' ' : '*');
	}

	V_PRINT("---"LB);

	V_PRINT(" S: save Configuration" LB " R: reset to Defaults" LB LB);
	//       0123456789+123456789+123456789+1234567894123456789+123456789+123456789+123456789
}

/** @ingroup MASTER
 * 送信完了応答を返す
 * @param u8Status True/False
 */
static void vSerResp_Ack(uint8 u8Status) {
	uint8 *q = sSerCmdTemp.au8data;

	S_OCTET(0xDB);
	S_OCTET(SERCMD_ID_ACK);
	S_OCTET(u8Status);

	sSerCmdTemp.u16len = q - sSerCmdTemp.au8data;
	sSerCmdTemp.vOutput(&sSerCmdTemp, &sSerStream);
}

/** @ingroup MASTER
 * モジュールのシリアル番号を出力する。
 * @param u8Status True/False
 */
static void vSerResp_GetModuleAddress() {
	uint8 *q = sSerCmdTemp.au8data;

	S_OCTET(0xDB);
	S_BE_DWORD(ToCoNet_u32GetSerial());

	sSerCmdTemp.u16len = q - sSerCmdTemp.au8data;
	sSerCmdTemp.vOutput(&sSerCmdTemp, &sSerStream);
}

/** @ingroup MASTER
 * 送信完了応答を返す
 * @param u8RspId 応答ID
 * @param u8Status 完了ステータス
 */
static void vSerResp_TxEx(uint8 u8RspId, uint8 u8Status) {
	uint8 *q = sSerCmdTemp.au8data;

	S_OCTET(0xDB);
	S_OCTET(SERCMD_ID_TRANSMIT_EX_RESP);
	S_OCTET(u8RspId);
	S_OCTET(u8Status);

	sSerCmdTemp.u16len = q - sSerCmdTemp.au8data;
	sSerCmdTemp.vOutput(&sSerCmdTemp, &sSerStream);
}

/** @ingroup MASTER
 * シリアルポートからの入力を処理する。
 * @param i16CharExt アプリケーション中から、本関数を呼びたい時に入力系列をパラメータ渡しする（ボタンにUARTと共通の機能を割りつけたい場合など）
 */
void vHandleSerialInput() {
	// handle UART command
	while (!SERIAL_bRxQueueEmpty(sSerPort.u8SerialPort)) {
		int16 i16Char = 0xFFFF;

		if (  sSerCmd_P3.bverbose ||
			(!sSerCmd_P3.bverbose &&
				(   (sAppData.sFlash.sData.u8uart_mode == UART_MODE_TRANSPARENT && sSerCmd.u16len < SERCMD_MAXPAYLOAD)
				||  (sAppData.sFlash.sData.u8uart_mode != UART_MODE_TRANSPARENT && !sSerSeqTx.bWaitComplete)) )) {
			i16Char = SERIAL_i16RxChar(sSerPort.u8SerialPort);
		} else {
			// TODO: フロー制御で送信停止させる？
			break;
		}

		// process
		if (i16Char >=0 && i16Char <= 0xFF) {
			//DBGOUT(0, "[%02x]", i16Char);
			if (INPSTR_bActive(&sSerInpStr)) {
				// 文字列入力モード
				uint8 u8res = INPSTR_u8InputByte(&sSerInpStr, (uint8)i16Char);

				if (u8res == E_INPUTSTRING_STATE_COMPLETE) {
					vProcessInputString(&sSerInpStr);
				} else if (u8res == E_INPUTSTRING_STATE_CANCELED) {
					V_PRINT("(canceled)");
					u16HoldUpdateScreen = 64;
				}
				continue;
			}

			{
				// コマンド書式の系列解釈、および verbose モードの判定
				uint8 u8res;
				u8res = SerCmdPlus3_u8Parse(&sSerCmd_P3, (uint8)i16Char);

				if (u8res != E_SERCMD_PLUS3_EMPTY) {
					if (u8res == E_SERCMD_PLUS3_VERBOSE_ON) {
						// verbose モードの判定があった
						vSerUpdateScreen();
						sSerCmd.u16timeout = 0;
					}

					if (u8res == E_SERCMD_PLUS3_VERBOSE_OFF) {
						vfPrintf(&sSerStream, "!INF EXIT INTERACTIVE MODE."LB);
						sSerCmd.u16timeout = 1000;
					}

					// still waiting for bytes.
					//continue;
				} else {
					; // コマンド解釈モードではない
				}
			}

			// Verbose モードのときは、シングルコマンドを取り扱う
			if (sSerCmd_P3.bverbose) {
				if (sAppData.sFlash.sData.u8uart_mode == UART_MODE_ASCII || sAppData.sFlash.sData.u8uart_mode == UART_MODE_BINARY) {
					// コマンドの解釈
					vProcessInputByte_FormatCmd(i16Char);

					// エコーバック出力
					if (sSerCmd.u8state != E_SERCMD_EMPTY) {
						if (sAppData.sFlash.sData.u8uart_mode == UART_MODE_ASCII) {
							V_PUTCHAR(i16Char);
						} else if (sAppData.sFlash.sData.u8uart_mode == UART_MODE_BINARY) {
							/* 文字の16進変換 */
							uint8 c = (i16Char >> 4) & 0xF;
							V_PUTCHAR(c < 0xA ? '0' + c : 'A' + c - 10);
							c = i16Char & 0xF;
							V_PUTCHAR(c < 0xA ? '0' + c : 'A' + c - 10);
						}
					}
				} else {
					sSerCmd.u8state = E_SERCMD_EMPTY;
				}

				if (sSerCmd.u8state == E_SERCMD_EMPTY) {
					// 書式入出力でなければ、１バイトコマンド
					vProcessInputByte(i16Char);
				}
			} else {
				if (sAppData.sFlash.sData.u8uart_mode == UART_MODE_TRANSPARENT) {
					vProcessInputByte_Transparent(i16Char);
				} else {
					vProcessInputByte_FormatCmd(i16Char);
				}
			}
		}
	}
}

/** @ingroup MASTER
 * １バイト入力コマンドの処理
 * @param u8Byte 入力バイト
 */
static void vProcessInputByte_Transparent(uint8 u8Byte) {
	// sSerCmd の au8data と u16len のみ利用している。
	// 送信は定期タイマーイベントで実行。
	sSerCmd.au8data[sSerCmd.u16len++] = u8Byte;
}

/** @ingroup MASTER
 * １バイト入力コマンドの処理
 * @param u8Byte 入力バイト
 */
static void vProcessInputByte_FormatCmd(uint8 u8Byte) {
	// チャットモードの制御コード
	if (sAppData.sFlash.sData.u8uart_mode == UART_MODE_CHAT) {
		if (u8Byte == 0x0c) { // Ctrl+L
			vfPrintf(&sSerStream, "%c[2J%c[H", 27, 27); // CLEAR SCREEN
			vSerChatPrompt();
			return;
		}
	}

	// コマンド書式の系列解釈、および verbose モードの判定
	uint8 u8Prev = sSerCmd.u8state;
	uint8 u8res = sSerCmd.u8Parse(&sSerCmd, (uint8)u8Byte);
	bool_t bPrompt = FALSE;

	// 完了時の処理
	if (u8res == E_SERCMD_COMPLETE || u8res == E_SERCMD_CHECKSUM_ERROR) {
		// 解釈完了

		if (u8res == E_SERCMD_CHECKSUM_ERROR) {
			// command complete, but CRC error
			V_PRINT(LB "!INF CHSUM_ERR? (might be %02X)" LB, sSerCmd.u16cksum);
		}

		if (u8res == E_SERCMD_COMPLETE) {
			// チャットモードのときは末尾にハンドル名を付記する
			// 書式：
			//   {入力テキスト} [0x0][0x0] <= ハンドル名未設定
			//   {入力テキスト} [0x0] {ハンドル名} [0x0]
			if (sAppData.sFlash.sData.u8uart_mode == UART_MODE_CHAT || sAppData.sFlash.sData.u8uart_mode == UART_MODE_CHAT_NO_PROMPT) {
				sSerCmd.au8data[sSerCmd.u16len] = 0x00; // 最初は０
				sSerCmd.u16len++;

				uint8 *p = sAppData.sFlash.sData.au8ChatHandleName;
				while(*p) {
					sSerCmd.au8data[sSerCmd.u16len] = *p;
					sSerCmd.u16len++;
					p++;
				}

				sSerCmd.au8data[sSerCmd.u16len] = 0x00; // 末尾も０
				sSerCmd.u16len++;
			}

			// PAYLOAD が大きすぎる場合は新しくスタート
			if (sSerCmd.u16len > SERCMD_MAXPAYLOAD) {
				sSerCmd.u8state = E_SERCMD_EMPTY;

				if (sAppData.sFlash.sData.u8uart_mode == UART_MODE_CHAT || sAppData.sFlash.sData.u8uart_mode == UART_MODE_CHAT_NO_PROMPT) {
					vfPrintf(&sSerStream, "(err:too long)");
				}

				return;
			}

			// process command
			if (sAppData.sFlash.sData.u8uart_mode == UART_MODE_CHAT || sAppData.sFlash.sData.u8uart_mode == UART_MODE_CHAT_NO_PROMPT) {
				// チャットモードの場合は、系列を送信する。
				if (sSerCmd.u16len) {
					i16TransmitSerMsg(
						sSerCmd.au8data, sSerCmd.u16len, NULL,
						ToCoNet_u32GetSerial(),
						sAppData.u8AppLogicalId,
						TOCONET_NWK_ADDR_NULL,
						LOGICAL_ID_BROADCAST,
						FALSE,
						sAppData.u8UartReqNum,
						sAppData.u8UartReqNum);
					sAppData.u8UartReqNum++;
					bPrompt = TRUE;
				}
			} else {
				vProcessSerialCmd(&sSerCmd);
			}
		}
	}

	// チャットモードの後出力処理
	if (sAppData.sFlash.sData.u8uart_mode == UART_MODE_CHAT) {
		// チャットモードのエコーバック
		if (u8res != E_SERCMD_EMPTY // 何か入力中
			&& u8res < 0x80 // エラー検出でもない
		) {
			if (u8Byte == 0x08 || u8Byte == 0x7F) {
				vPutChar(&sSerStream, 0x08);
			} else {
				vPutChar(&sSerStream, u8Byte);
			}
		}

		// エラー
		if (u8res > 0x80) {
			vfPrintf(&sSerStream, "(err)");
		}

		// EMPTY 状態での改行が来たらプロンプト
		if (u8res == E_SERCMD_EMPTY && u8Byte == 0x0d) {
			bPrompt = TRUE;
		}

		// 直前が入力処理中で、EMPTY に変化した
		if (u8Prev != u8res && u8res == E_SERCMD_EMPTY && u8Prev < 0x80) {
			bPrompt = TRUE;
			vfPrintf(&sSerStream, "(canceled)");
		}

		// 状態が確定した
		if (u8res >= 0x80) {
			bPrompt = TRUE;
		}

		if (bPrompt) {
			vSerChatPrompt();
		}
	}
}


/** @ingroup MASTER
 * １バイト入力コマンドの処理
 * @param u8Byte 入力バイト
 */
static void vProcessInputByte(uint8 u8Byte) {
	switch (u8Byte) {
	case 0x0d:
		// 画面の書き換え
		u16HoldUpdateScreen = 1;
		break;

	case 'a': // set application ID
		V_PRINT("Input Application ID (HEX:32bit): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_HEX, 8, E_APPCONF_APPID);
		break;

	case 'c': // チャネルの設定
		V_PRINT("Input Channel(s) (e.g. 11,16,21): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_STRING, 8, E_APPCONF_CHMASK);
		break;

	case 'o': // チャネルの設定
		V_PRINT("Input Channel(s) (0[min],1,2,3[max]): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 1, E_APPCONF_POWER);
		break;

	case 'i': // set application role
		V_PRINT("Input Device ID (DEC:1-100): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 3, E_APPCONF_ID);
		break;

	case 'r': // システムの役割定義
		V_PRINT("Input Role ID (HEX): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_HEX, 2, E_APPCONF_ROLE);
		break;

	case 'b': // ボーレートの変更
		V_PRINT("Input baud rate (DEC:9600-230400): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_STRING, 10, E_APPCONF_BAUD_SAFE);
		break;

	case 'p': // パリティの変更
		V_PRINT("Input parity (N,E,O): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_STRING, 1, E_APPCONF_BAUD_PARITY);
		break;

	case 'm': // パリティの変更
		V_PRINT("Input UART mode (A,B,C,T): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_STRING, 1, E_APPCONF_UART_MODE);
		break;

	case 'h':
		V_PRINT("Input handle name: ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_STRING, 32, E_APPCONF_HANDLE_NAME);
		break;

	case 'C':
		V_PRINT("Input crypt mode (0,1): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 2, E_APPCONF_CRYPT_MODE);
		break;

	case 'K':
		V_PRINT("Input crypt key: ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_STRING, 32, E_APPCONF_CRYPT_KEY);
		break;

	case 'S':
		// フラッシュへのデータ保存
		vConfig_SaveAndReset();
		break;

	case 'R':
		_C {
			vConfig_SetDefaults(&sAppData.sConfig_UnSaved);
			u16HoldUpdateScreen = 1;
		}
		break;

	case '$':
		_C {
			sAppData.u8DebugLevel++;
			if(sAppData.u8DebugLevel > 5) sAppData.u8DebugLevel = 0;

			V_PRINT("* set App debug level to %d." LB, sAppData.u8DebugLevel);
		}
		break;

	case '@':
		_C {
			static uint8 u8DgbLvl;

			u8DgbLvl++;
			if(u8DgbLvl > 5) u8DgbLvl = 0;
			ToCoNet_vDebugLevel(u8DgbLvl);

			V_PRINT("* set NwkCode debug level to %d." LB, u8DgbLvl);
		}
		break;

	case '!':
		// リセット
		V_PRINT("!INF RESET SYSTEM.");
		vWait(1000000);
		vAHI_SwReset();
		break;

	case '#': // info
		_C {
			V_PRINT("*** ToCoNet(ver%08X) ***" LB, ToCoNet_u32GetVersion());
			V_PRINT("* AppID %08x, LongAddr, %08x, ShortAddr %04x, Tk: %d" LB,
					sToCoNet_AppContext.u32AppId, ToCoNet_u32GetSerial(), sToCoNet_AppContext.u16ShortAddress, u32TickCount_ms);
			if (sAppData.bFlashLoaded) {
				V_PRINT("** Conf "LB);
				V_PRINT("* AppId = %08x"LB, sAppData.sFlash.sData.u32appid);
				V_PRINT("* ChMsk = %08x"LB, sAppData.sFlash.sData.u32chmask);
				V_PRINT("* Ch=%d, Role=%d, Layer=%d"LB,
						sToCoNet_AppContext.u8Channel,
						sAppData.sFlash.sData.u8role,
						sAppData.sFlash.sData.u8layer);
			} else {
				V_PRINT("** Conf: none"LB);
			}
		}
		break;

	default:
		break;
	}
}

/** @ingroup MASTER
 * 文字列入力モードの処理
 */
static void vProcessInputString(tsInpStr_Context *pContext) {
	uint8 *pu8str = pContext->au8Data;
	uint8 u8idx = pContext->u8Idx;

	switch(pContext->u32Opt) {
	case E_APPCONF_APPID:
		_C {
			uint32 u32val = u32string2hex(pu8str, u8idx);

			uint16 u16h, u16l;
			u16h = u32val >> 16;
			u16l = u32val & 0xFFFF;

			if (u16h == 0x0000 || u16h == 0xFFFF || u16l == 0x0000 || u16l == 0xFFFF) {
				V_PRINT("(ignored: 0x0000????,0xFFFF????,0x????0000,0x????FFFF can't be set.)");
			} else {
				sAppData.sConfig_UnSaved.u32appid = u32val;
			}

			V_PRINT(LB"-> %08X"LB, u32val);
		}
		break;

	case E_APPCONF_CHMASK:
		_C {
			// チャネルマスク（リスト）を入力解釈する。
			//  11,15,19 のように１０進数を区切って入力する。
			//  以下では区切り文字は任意で MAX_CHANNELS 分処理したら終了する。

			uint8 b = 0, e = 0, i = 0, n_ch = 0;
			uint32 u32chmask = 0; // 新しいチャネルマスク

			V_PRINT(LB"-> ");

			for (i = 0; i <= pContext->u8Idx; i++) {
				if (pu8str[i] < '0' || pu8str[i] > '9') {
					e = i;
					uint8 u8ch = 0;

					// 最低２文字あるときに処理
					if (e - b > 0) {
						u8ch = u32string2dec(&pu8str[b], e - b);
						if (u8ch >= 11 && u8ch <= 26) {
							if (n_ch) {
								V_PUTCHAR(',');
							}
							V_PRINT("%d", u8ch);
							u32chmask |= (1UL << u8ch);

							n_ch++;
							if (n_ch >= MAX_CHANNELS) {
								break;
							}
						}
					}
					b = i + 1;
				}

				if (pu8str[i] == 0x0) {
					break;
				}
			}

			if (u32chmask == 0x0) {
				V_PRINT("(ignored)");
			} else {
				sAppData.sConfig_UnSaved.u32chmask = u32chmask;
			}

			V_PRINT(LB);
		}
		break;

	case E_APPCONF_POWER:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);
			V_PRINT(LB"-> ");
			if (u32val <= 3) {
				sAppData.sConfig_UnSaved.u8power = u32val; // 0-3 3が最大
				V_PRINT("%d"LB, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_ID:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);
			V_PRINT(LB"-> ");
			if (u32val <= 0x7F || u32val == LOGICAL_ID_REPEATER) {
				sAppData.sConfig_UnSaved.u8id = u32val; // ０は未設定！
				V_PRINT("%d(0x%02x)"LB, u32val, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_ROLE:
		_C {
			uint32 u32val = u32string2hex(pu8str, u8idx);
			V_PRINT(LB"-> ");
			if (u32val == E_APPCONF_ROLE_MAC_NODE || u32val == E_APPCONF_ROLE_MAC_NODE_REPEATER) {
				sAppData.sConfig_UnSaved.u8role = u32val; // ０は未設定！
				V_PRINT("(0x%02x)"LB, u32val, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_BAUD_SAFE:
		_C {
			uint32 u32val = 0;

			if (pu8str[0] == '0' && pu8str[1] == 'x') {
				u32val = u32string2hex(pu8str + 2, u8idx - 2);
			} if (u8idx <= 6) {
				u32val = u32string2dec(pu8str, u8idx);
			}

			V_PRINT(LB"-> ");

			if (u32val) {
				sAppData.sConfig_UnSaved.u32baud_safe = u32val;
				if (u32val & 0x80000000) {
					V_PRINT("%x"LB, u32val);
				} else {
					V_PRINT("%d"LB, u32val);
				}
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_BAUD_PARITY:
		_C {
			V_PRINT(LB"-> ");

			if (pu8str[0] == 'N' || pu8str[0] == 'n') {
				sAppData.sConfig_UnSaved.u8parity = 0;
				V_PRINT("None"LB);
			} else if (pu8str[0] == 'O' || pu8str[0] == 'o') {
				sAppData.sConfig_UnSaved.u8parity = 1;
				V_PRINT("Odd"LB);
			} else if (pu8str[0] == 'E' || pu8str[0] == 'e') {
				sAppData.sConfig_UnSaved.u8parity = 2;
				V_PRINT("Even"LB);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_UART_MODE:
		_C {
			V_PRINT(LB"-> ");

			if (pu8str[0] == 'T' || pu8str[0] == 't') {
				sAppData.sConfig_UnSaved.u8uart_mode = UART_MODE_TRANSPARENT;
				V_PRINT("Transparent mode"LB);
			} else if (pu8str[0] == 'A' || pu8str[0] == 'a') {
				sAppData.sConfig_UnSaved.u8uart_mode = UART_MODE_ASCII;
				V_PRINT("Modbus ASCII mode"LB);
			} else if (pu8str[0] == 'B' || pu8str[0] == 'b') {
				sAppData.sConfig_UnSaved.u8uart_mode = UART_MODE_BINARY;
				V_PRINT("Binary mode"LB);
			} else if (pu8str[0] == 'C' || pu8str[0] == 'c') {
				sAppData.sConfig_UnSaved.u8uart_mode = UART_MODE_CHAT;
				V_PRINT("Chat mode"LB);
			} else if (pu8str[0] == 'D' || pu8str[0] == 'd') {
				sAppData.sConfig_UnSaved.u8uart_mode = UART_MODE_CHAT_NO_PROMPT;
				V_PRINT("Chat mode w/o prompt"LB);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_CRYPT_MODE:
		_C {
			if (pu8str[0] == '0') {
				sAppData.sConfig_UnSaved.u8Crypt = 0;
				V_PRINT(LB"--> Plain");
			} else if (pu8str[0] == '1') {
				sAppData.sConfig_UnSaved.u8Crypt = 1;
				V_PRINT(LB"--> AES128");
			} else {
				V_PRINT(LB"(ignored)");
			}
		}
		break;

	case E_APPCONF_CRYPT_KEY:
		_C {
			uint8 u8len = strlen((void*)pu8str);

			if (u8len == 0) {
				memset(sAppData.sConfig_UnSaved.au8AesKey, 0, sizeof(sAppData.sConfig_UnSaved.au8AesKey));
				V_PRINT(LB"(cleared)");
			} else
			if (u8len && u8len <= 32) {
				memset(sAppData.sConfig_UnSaved.au8AesKey, 0, sizeof(sAppData.sConfig_UnSaved.au8AesKey));
				memcpy(sAppData.sConfig_UnSaved.au8AesKey, pu8str, u8len);
				V_PRINT(LB);
			} else {
				V_PRINT(LB"(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_HANDLE_NAME:
		_C {
			uint8 u8len = strlen((void*)pu8str);

			if (u8len == 0) {
				memset(sAppData.sConfig_UnSaved.au8ChatHandleName, 0, sizeof(sAppData.sConfig_UnSaved.au8ChatHandleName));
				V_PRINT(LB"(cleared)");
			} else
			if (u8len && u8len < sizeof(sAppData.sConfig_UnSaved.au8ChatHandleName)) {
				memset(sAppData.sConfig_UnSaved.au8ChatHandleName, 0, sizeof(sAppData.sConfig_UnSaved.au8ChatHandleName));
				memcpy(sAppData.sConfig_UnSaved.au8ChatHandleName, pu8str, u8len);
				V_PRINT(LB);
			} else {
				V_PRINT(LB"(ignored)");
			}
		}
		break;
	default:
		break;
	}

	// 一定時間待って画面を再描画
	u16HoldUpdateScreen = 96; // 1.5sec
}

/** @ingroup MASTER
 * シリアルから入力されたコマンド形式の電文を処理する。
 * @param pSer
 */
static void vProcessSerialCmd(tsSerCmd_Context *pSer) {
	uint8 u8addr; // 送信先論理アドレス

	uint8 u8cmd;

	uint8 *p = pSer->au8data;
	uint8 *p_end;
	p_end = p + pSer->u16len; // the end points 1 byte after the data end.

	// COMMON FORMAT
	OCTET(u8addr); // [1] OCTET : Address information
	OCTET(u8cmd); // [1] OCTET : Command information

	DBGOUT(1, "* UARTCMD ln=%d cmd=%02x req=%02x %02x%0x2%02x%02x..." LB,
			pSer->u16len,
			u8addr,
			u8cmd,
			*p,
			*(p+1),
			*(p+2),
			*(p+3)
			);

	if (u8addr == 0xDB) {
		// ローカルコマンド
		switch(u8cmd) {
		case SERCMD_ID_ACK:
			vSerResp_Ack(TRUE);
			break;

		case SERCMD_ID_GET_MODULE_ADDRESS:
			vSerResp_GetModuleAddress();
			break;
		}
	} else if (u8cmd == SERCMD_ID_TRANSMIT_EX) {
		vProcessSerialCmd_TransmitEx(pSer);
	} else {
		if (pSer->u16len >= 3) {
			i16TransmitSerMsg(
				pSer->au8data, pSer->u16len, NULL,
				ToCoNet_u32GetSerial(),
				sAppData.u8AppLogicalId,
				TOCONET_NWK_ADDR_NULL,
				pSer->au8data[0],
				FALSE,
				sAppData.u8UartReqNum | 0x80,
				sAppData.u8UartReqNum | 0x80);
			sAppData.u8UartReqNum++;
		}
	}
}

/** @ingroup MASTER
 * シリアルから入力されたコマンド形式の電文を処理する。
 * @param pSer
 */
static void vProcessSerialCmd_TransmitEx(tsSerCmd_Context *pSer) {
	uint8 *p = pSer->au8data;
	uint8 *p_end = pSer->au8data + pSer->u16len;

	tsTxDataApp sTx; // 送信オプションのテンプレート
	memset(sTx.auData, 0, sizeof(tsTxDataApp));

	sTx.u8Retry = 0xFF;
	sTx.u16DelayMin = 0;
	sTx.u16DelayMax = 0;

	uint8 u8addr = G_OCTET();
	uint8 u8cmd = G_OCTET();

	uint8 u8resp = G_OCTET();

	// 拡張アドレス
	uint32 u32AddrDst = TOCONET_NWK_ADDR_NULL;
	if (u8addr == LOGICAL_ID_EXTENDED_ADDRESS) {
		u32AddrDst = G_BE_DWORD();
	}

	// コマンドオプション
	for (;;) {
		uint8 u8Opt = G_OCTET();
		if (u8Opt == TRANSMIT_EX_OPT_TERM) break;

		switch(u8Opt) {
		case TRANSMIT_EX_OPT_SET_MAC_ACK:
			if (	u8addr != LOGICAL_ID_CHILDREN
				&&	(u32AddrDst != TOCONET_MAC_ADDR_BROADCAST && u32AddrDst != TOCONET_NWK_ADDR_BROADCAST)) {
				sTx.bAckReq = TRUE;
				if (u32AddrDst == TOCONET_NWK_ADDR_NULL) {
					// １バイトアドレス利用時で ACK 要求が有る場合は、相手先の MAC アドレスを指定する
					u32AddrDst = SERCMD_ADDR_CONV_TO_SHORT_ADDR(u8addr);
				}
			}
			break;

		case TRANSMIT_EX_OPT_SET_APP_RETRY:
			sTx.u8Retry = G_OCTET();
			break;

		case TRANSMIT_EX_OPT_SET_DELAY_MIN_ms:
			sTx.u16DelayMin = G_BE_WORD();
			break;

		case TRANSMIT_EX_OPT_SET_DELAY_MAX_ms:
			sTx.u16DelayMax = G_BE_WORD();
			break;

		case TRANSMIT_EX_OPT_SET_RETRY_DUR_ms:
			sTx.u16RetryDur = G_BE_WORD();
			break;

		case TRANSMIT_EX_OPT_SET_PARALLEL_TRANSMIT:
			sTx.auData[TRANSMIT_EX_OPT_SET_PARALLEL_TRANSMIT] = TRUE;
			break;
		}
	}

	// アプリケーション再送回数を指定
	if (sTx.u8Retry == 0xFF) {
		sTx.u8Retry = sTx.bAckReq ? 0 : DEFAULT_TX_FFFF_COUNT;
	}

	{
		DBGOUT(1, "* TxEx: Dst:%02x,%08x Ac:%d Re:%d Di:%d Da:%d Dr:%d Ln:%d %02x%02x%02x%02x",
				u8addr,
				u32AddrDst,
				sTx.bAckReq,
				sTx.u8Retry,
				sTx.u16DelayMin,
				sTx.u16DelayMax,
				sTx.u16RetryDur,
				p_end - p,
				p[0], p[1], p[2], p[3]
		);

		int i;
		for (i = 0; i < 8 && p + i < p_end; i++) {
			DBGOUT(1, "%02x", p[i]);
		}
		if (i == 8) DBGOUT(1, "...");
	}

	if (p < p_end) {
		p = p - 2;
		p[0] = u8addr;
		p[1] = u8cmd;

		if (i16TransmitSerMsg(p, p_end - p, &sTx,
				ToCoNet_u32GetSerial(), sAppData.u8AppLogicalId,
				u32AddrDst, u8addr,
				FALSE, sAppData.u8UartReqNum++, u8resp) == -1) {

			// エラー応答の出力
			vSerResp_TxEx(u8resp, 0x0);
		}
	}

	return;
}

/** @ingroup MASTER
 * シリアルメッセージの送信要求。
 * パケットを分割して送信する。
 *
 *  - Packet 構造
 *   - [1] OCTET    : 識別ヘッダ(APP ID より生成), 下１ビットが1の時は中継済み
 *   - [1] OCTET    : プロトコルバージョン(バージョン間干渉しないための識別子)
 *   - [1] OCTET    : 応答ID(外部から指定される値、任意に使用出来るデータの一つ)
 *   - [1] OCTET    : 送信元、簡易アドレス
 *   - [4] BE_DWORD : 送信元、拡張アドレス
 *   - [1] OCTET    : 宛先、簡易アドレス
 *   - [4] BE_DWORD : 宛先、拡張アドレス
 *   - [2] BE_WORD  : 送信タイムスタンプ (64fps カウンタの値の下１６ビット, 約1000秒で１周する)
 *   - [1] OCTET    : パケット群の識別ID
 *   - [1] OCTET    : パケット数(total)
 *   - [1] OCTET    : パケット番号 (0...total-1)
 *
 * @param p 送信データ（実際に送信したいデータ部で各種ヘッダは含まない）
 * @param u16len データ長
 * @param pTxTemplate 送信オプションを格納した構造体（送信用の構造体を流用）
 * @param u32AddrSrc 送信元、拡張アドレス
 * @param u8AddrSrc 送信元、簡易アドレス
 * @param u32AddrDst 宛先、拡張アドレス
 * @param u8AddrDst 宛先、簡易アドレス
 * @param bRelay 中継フラグ、TRUEなら中継ビットを立てた状態で送信する
 * @param u8Req 識別ID、パケット群を識別するための内部的に利用するID。重複の除去などに用いる。
 * @param u8RspId 応答ID、外部向けの識別ID。成功失敗などの応答などにも用いる。
 * @return
 */
static int16 i16TransmitSerMsg(uint8 *p, uint16 u16len, tsTxDataApp *pTxTemplate,
		uint32 u32AddrSrc, uint8 u8AddrSrc, uint32 u32AddrDst, uint8 u8AddrDst,
		bool_t bRelay, uint8 u8Req, uint8 u8RspId) {

	if(IS_APPCONF_ROLE_SILENT_MODE()) return -1;

	// 処理中のチェック（処理中なら送信せず失敗）
	if (sSerSeqTx.bWaitComplete) {
		return -1;
	}

	// パケットを分割して送信する。
	tsTxDataApp sTx;
	if (pTxTemplate == NULL) {
		memset(&sTx, 0, sizeof(sTx));
	} else {
		memcpy(&sTx, pTxTemplate, sizeof(sTx));
	}
	uint8 *q; // for S_??? macros

	// sSerSeqTx は分割パケットの管理構造体
	sSerSeqTx.u8IdSender = sAppData.u8AppLogicalId;
	sSerSeqTx.u8IdReceiver = u8AddrDst;

	sSerSeqTx.u8PktNum = (u16len - 1) / SERCMD_SER_PKTLEN + 1;
	sSerSeqTx.u16DataLen = u16len;

	if (u16len > SERCMD_MAXPAYLOAD) {
		return -1; // ペイロードが大きすぎる
	}

	sSerSeqTx.u8RespID = u8RspId;
	sSerSeqTx.u8Seq = sAppData.u8UartSeqNext; // パケットのシーケンス番号（アプリでは使用しない）
	sAppData.u8UartSeqNext = (sSerSeqTx.u8Seq + sSerSeqTx.u8PktNum) & CBID_MASK_BASE; // 次のシーケンス番号（予め計算しておく）
	sSerSeqTx.u8ReqNum = u8Req; // パケットの要求番号（この番号で送信系列を弁別する）

	sSerSeqTx.u32Tick = u32TickCount_ms;
	if (sTx.auData[TRANSMIT_EX_OPT_SET_PARALLEL_TRANSMIT] && sSerSeqTx.u8PktNum == 1) {
		// 併行送信時は bWaitComplete の条件を立てない
		sSerSeqTx.bWaitComplete = FALSE;
	} else {
		sSerSeqTx.bWaitComplete = TRUE;
	}

	memset(sSerSeqTx.bPktStatus, 0, sizeof(sSerSeqTx.bPktStatus));

	DBGOUT(3, "* >>> Transmit(req=%d) Tick=%d <<<" LB, sSerSeqTx.u8ReqNum , u32TickCount_ms & 65535);

	sTx.u8Cmd = TOCONET_PACKET_CMD_APP_DATA; // data packet.

#ifdef USE_AES
	if (IS_CRYPT_MODE()) {
		sTx.bSecurePacket = TRUE;
	}
#endif

	if (sAppData.eNwkMode == E_NWKMODE_LAYERTREE) {
		// ネットワーク層経由の送受信

		sTx.u32SrcAddr = ToCoNet_u32GetSerial();
		sTx.u32DstAddr = TOCONET_NWK_ADDR_NULL;

		// 親機の場合
		if (IS_LOGICAL_ID_PARENT(sAppData.u8AppLogicalId)) {
			if (u32AddrDst != TOCONET_NWK_ADDR_NULL && TOCONET_NWK_ADDR_IS_LONG(u32AddrDst)) {
				// 32bitアドレスが指定されている場合は、そのアドレス宛に送信する
				sTx.u32DstAddr = u32AddrDst;
			} else {
				// 32bitアドレスが指定されていない場合は、ブロードキャストにて送信する
				sTx.u32DstAddr = TOCONET_NWK_ADDR_BROADCAST;
			}
		}

		// 子機の場合
		if (IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId)) {
			// 宛先は自動的に親機(子機間はNG)
			if (IS_LOGICAL_ID_PARENT(u8AddrDst) || u32AddrDst == TOCONET_NWK_ADDR_PARENT) {
				sTx.u32DstAddr = TOCONET_NWK_ADDR_PARENT;
			}
		}

		// 例外時は送信しない
		if (sTx.u32DstAddr == TOCONET_NWK_ADDR_NULL) {
			return -1;
		}
	} else {
		// MAC 直接の送受信

		// 送信設定の微調整を行う
		if (bRelay) {
			sTx.u8Retry = DEFAULT_TX_FFFF_COUNT; // ３回送信する
			sTx.u16DelayMin = DEFAULT_TX_FFFF_DELAY_ON_REPEAT_ms; // 中継時の遅延
			sTx.u16RetryDur = sSerSeqTx.u8PktNum * 10; // application retry
		} else
		if (pTxTemplate == NULL) {
			sTx.u8Retry = DEFAULT_TX_FFFF_COUNT; // ３回送信する
			sTx.u16DelayMin = DEFAULT_TX_FFFF_DELAY_ON_REPEAT_ms; // 中継時の遅延
			sTx.u16RetryDur = sSerSeqTx.u8PktNum * 10; // application retry
		} else {
			if (sTx.u16RetryDur <= sSerSeqTx.u8PktNum * 10) {
				sTx.u16RetryDur = sSerSeqTx.u8PktNum * 10; // application retry
			}
		}

		// 宛先情報(指定された宛先に送る)
		sTx.u32SrcAddr = sToCoNet_AppContext.u16ShortAddress;
		if (sTx.bAckReq) {
			sTx.u32DstAddr = u32AddrDst;
		} else {
			sTx.u32DstAddr  = TOCONET_MAC_ADDR_BROADCAST; // ブロードキャスト
		}
	}

	int i;
	for (i = 0; i < sSerSeqTx.u8PktNum; i++) {
		q = sTx.auData;
		sTx.u8Seq = (sSerSeqTx.u8Seq + i) & CBID_MASK_BASE;
		sTx.u8CbId = (sSerSeqTx.u8PktNum > 1) ? (sTx.u8Seq | CBID_MASK_SPLIT_PKTS) : sTx.u8Seq; // callback will reported with this ID

		if (bRelay) {
			sTx.u8CbId |= CBID_MASK_SILENT;
		}

		// コールバックIDと応答IDを紐づける
		au8TxCbId_to_RespID[sTx.u8CbId] = sSerSeqTx.u8RespID;

		// UARTモード間で混在しないように sTx.u8Cmd の値を変えておく
		sTx.u8Cmd = au8UartModeToTxCmdId[sAppData.sFlash.sData.u8uart_mode];

		// ペイロードを構成
		S_OCTET(sAppData.u8AppIdentifier + (bRelay ? 1 : 0));
		S_OCTET(APP_PROTOCOL_VERSION);

		S_OCTET(sSerSeqTx.u8RespID); // 応答ID

		S_OCTET(u8AddrSrc); // 送信元アプリケーション論理アドレス
		if (!(sAppData.eNwkMode == E_NWKMODE_LAYERTREE)) { // ネットワークモードの場合は、ロングアドレスは省略
			S_BE_DWORD(u32AddrSrc);  // シリアル番号
		}

		S_OCTET(sSerSeqTx.u8IdReceiver); // 宛先

		if (!(sAppData.eNwkMode == E_NWKMODE_LAYERTREE)) { // ネットワークモードの場合は、ロングアドレスは省略
			S_BE_DWORD(u32AddrDst); //最終宛先
		}

		S_OCTET(sSerSeqTx.u8ReqNum); // request number
		S_OCTET(sSerSeqTx.u8PktNum); // total packets
		S_OCTET(i); // packet number

		uint8 u8len_data = (u16len >= SERCMD_SER_PKTLEN) ? SERCMD_SER_PKTLEN : u16len;

		memcpy (q, p, u8len_data);
		q += u8len_data;

		sTx.u8Len = q - sTx.auData;

		if (sAppData.eNwkMode == E_NWKMODE_LAYERTREE) {
			ToCoNet_Nwk_bTx(sAppData.pContextNwk, &sTx);
		} else {
			ToCoNet_bMacTxReq(&sTx);
		}

		p += u8len_data;
		u16len -= SERCMD_SER_PKTLEN;
	}

	return 0;
}

/** @ingroup MASTER
 * シリアルメッセージの受信処理。分割パケットがそろった時点で UART に出力。
 * - 中継機なら新たな系列として中継。
 *   - 中継機と送信元が両方見える場合は、２つの系列を受信を行う事になる。
 * tsRxDataApp *pRx
 */
static void vReceiveSerMsg(tsRxDataApp *pRx) {
	uint8 *p = pRx->auData;

	/* ヘッダ情報の読み取り */
	uint8 u8AppIdentifier = G_OCTET();
	if ((u8AppIdentifier & 0xFE) != sAppData.u8AppIdentifier) { return; }
	uint8 u8PtclVersion = G_OCTET();
	if (u8PtclVersion != APP_PROTOCOL_VERSION) { return; }
	uint8 u8RespId = G_OCTET();
	uint8 u8AppLogicalId = G_OCTET(); (void)u8AppLogicalId;
	uint32 u32AddrSrc = pRx->bNwkPkt ? pRx->u32SrcAddr : G_BE_DWORD();
	uint8 u8AppLogicalId_Dest = G_OCTET();
	uint32 u32AddrDst = pRx->bNwkPkt ? pRx->u32DstAddr : G_BE_DWORD();
	uint8 bRepeatFlag = u8AppIdentifier & 0x1;

	/* ここから中身 */
	uint8 u8req = G_OCTET();
	uint8 u8pktnum = G_OCTET();
	uint8 u8idx = G_OCTET();

	uint8 u8len = (pRx->auData + pRx->u8Len) - p;
	uint16 u16offset = u8idx * SERCMD_SER_PKTLEN;

	/* 宛先と送信元のアドレスが一致する場合は処理しない */
	if (u32AddrSrc == u32AddrDst) return;
	if (u8AppLogicalId == u8AppLogicalId_Dest && u8AppLogicalId < 0x80) return;

	/* 宛先によって処理するか決める */
	bool_t bAcceptAddress = TRUE;
	bool_t bAcceptBroadCast = FALSE;

	if (IS_LOGICAL_ID_EXTENDED(u8AppLogicalId_Dest)) {
		// 拡張アドレスの場合 (アドレスによってパケットを受理するか決定する)
		if (u32AddrDst == TOCONET_MAC_ADDR_BROADCAST || u32AddrDst == TOCONET_NWK_ADDR_BROADCAST) {
			bAcceptBroadCast = TRUE; // ブロードキャストなので受理する
		} else if (u32AddrDst < 0xFFFF) {
			// ショートアドレス形式 (アドレスの一致が原則)
			if (u32AddrDst != sToCoNet_AppContext.u16ShortAddress) {
				bAcceptAddress = FALSE;
			}
		} else if (u32AddrDst & 0x80000000) {
			// 拡張アドレス (アドレスの一致が原則)
			if (u32AddrDst != ToCoNet_u32GetSerial()) {
				bAcceptAddress = FALSE;
			}
		} else {
			bAcceptAddress = FALSE;
		}
	} else if (u8AppLogicalId_Dest == LOGICAL_ID_BROADCAST) {
		// ブロードキャストは受信する
		bAcceptBroadCast = TRUE;
		bAcceptAddress = (u32AddrSrc == ToCoNet_u32GetSerial()) ? FALSE : TRUE;
	} else if (IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId)) {
		// 簡易アドレスが宛先で、自分が子機の場合
		if (u8AppLogicalId_Dest == sAppData.u8AppLogicalId || u8AppLogicalId_Dest == LOGICAL_ID_CHILDREN) {
			if (u8AppLogicalId_Dest == LOGICAL_ID_CHILDREN) {
				bAcceptBroadCast = TRUE;
			}
			if (u32AddrSrc == ToCoNet_u32GetSerial()) {
				// 自分が送ったパケットが中継機により戻ってきた場合
				bAcceptAddress = FALSE;
			}
		} else {
			bAcceptAddress = FALSE;
		}
	} else if (IS_LOGICAL_ID_PARENT(sAppData.u8AppLogicalId)) {
		//  簡易アドレスが宛先で、親機の場合
		if (u8AppLogicalId_Dest != LOGICAL_ID_PARENT) {
			// 親機同士の通信はしない
			bAcceptAddress = FALSE;
		}
	} else {
		bAcceptAddress = FALSE;
	}

	// 宛先が自分宛でない場合、非LayerNetwork のリピータなら一端受信する
	if (!bAcceptAddress) {
		if (!IS_REPEATER()) {
			return;
		}
	}

	// 受信パケットのチェック。
	//  - 分割パケットが混在したような場合は、新しい系列で始める。
	//    複数のストリームを同時受信できない！
	bool_t bNew = FALSE;
	if (sSerSeqRx.bWaitComplete) {
		// exceptional check
		if(u32TickCount_ms - sSerSeqRx.u32Tick > 2000) {
			// time out check
			bNew = TRUE;
		}
		if (u8req != sSerSeqRx.u8ReqNum) {
			// different request number is coming.
			bNew = TRUE;
		}
		if (u32AddrSrc != sSerSeqRx.u32SrcAddr) {
			// packet comes from different nodes. (discard this one!)
			bNew = TRUE;
		}
	} else {
		// 待ち状態ではないなら新しい系列
		bNew = TRUE;
	}

	if(bNew) {
		// treat this packet as new, so clean control buffer.
		memset(&sSerSeqRx, 0, sizeof(sSerSeqRx));
	}

	if (!sSerSeqRx.bWaitComplete) {
		// 新しいパケットを受信した

		// 最初にリクエスト番号が適切かどうかチェックする。
		uint32 u32key;
		if (DUPCHK_bFind(&sDupChk_SerMsg, u32AddrSrc, &u32key)) {
			int iPrev = u32key, iNow = u8req;

			if (iNow == iPrev || (uint8)(iNow - iPrev) > 0x80) {
				// 最近受信したものより新しくないリクエスト番号の場合は、処理しない
				bNew = FALSE;
			}
		}

		if (bNew) {
			sSerSeqRx.bWaitComplete = TRUE;
			sSerSeqRx.u32Tick = u32TickCount_ms;
			sSerSeqRx.u32SrcAddr = u32AddrSrc;
			sSerSeqRx.u32DstAddr = u32AddrDst;
			sSerSeqRx.u8PktNum = u8pktnum;
			sSerSeqRx.u8ReqNum = u8req;
			sSerSeqRx.u8RespID = u8RespId;
			sSerSeqRx.u8IdSender = u8AppLogicalId;
			sSerSeqRx.u8IdReceiver = u8AppLogicalId_Dest;

			DUPCHK_vAdd(&sDupChk_SerMsg, sSerSeqRx.u32SrcAddr, u8req);
		}
	}

	if (sSerSeqRx.bWaitComplete) {
		if (u16offset + u8len <= SIZEOF_au8SerBuffRx && u8idx < sSerSeqRx.u8PktNum) {
			// check if packet size and offset information is correct,
			// then copy data to buffer and mark it as received.
			if (!sSerSeqRx.bPktStatus[u8idx]) {
				sSerSeqRx.bPktStatus[u8idx] = 1;
				memcpy (au8SerBuffRx + u16offset, p, u8len);
			}

			// the last packet indicates full data length.
			if (u8idx == sSerSeqRx.u8PktNum - 1) {
				sSerSeqRx.u16DataLen = u16offset + u8len;
			}

			// 中継パケットのフラグを格納する
			if (bRepeatFlag) {
				sSerSeqRx.bRelayPacket = TRUE;
			}
		}

		// check completion
		int i;
		for (i = 0; i < sSerSeqRx.u8PktNum; i++) {
			if (sSerSeqRx.bPktStatus[i] == 0) break;
		}
		if (i == sSerSeqRx.u8PktNum) {
			// 分割パケットが全て届いた！

			// リピータで、パケットがブロードキャスト、または、送信アドレス
			if (IS_REPEATER() && (bAcceptBroadCast || (!bAcceptAddress))) {
				// まだ中継されていないパケットなら、中継する
				if (!sSerSeqRx.bRelayPacket) {
					i16TransmitSerMsg(
							au8SerBuffRx,
							sSerSeqRx.u16DataLen,
							NULL,
							sSerSeqRx.u32SrcAddr,
							sSerSeqRx.u8IdSender,
							sSerSeqRx.u32DstAddr,
							sSerSeqRx.u8IdReceiver,
							TRUE,
							sSerSeqRx.u8ReqNum,
							sSerSeqRx.u8RespID);
				}
				DBGOUT(1, "<RPT FR:%02X TO:%02X #:%02X>", sSerSeqRx.u8IdSender, sSerSeqRx.u8IdReceiver, sSerSeqRx.u8ReqNum);
			}

			// 自分宛のメッセージなら、UART に出力する
			if (bAcceptAddress && !IS_DEDICATED_REPEATER()) {
				// 受信データの出力
				if (sAppData.sFlash.sData.u8uart_mode == UART_MODE_TRANSPARENT) {
					// 透過モード
					int j;
					bool_t bOk = FALSE;

					if (IS_LOGICAL_ID_PARENT(sAppData.u8AppLogicalId)) {
						if (sAppData.u8AppLogicalId_Pair == sSerSeqRx.u8IdSender) {
							bOk = TRUE; // 送り元とIDが一致した場合
						}
					} else if (IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId)) {
						if (IS_LOGICAL_ID_PARENT(sSerSeqRx.u8IdSender)) {
							bOk = TRUE; // 相手が親機なら
						}
					}

					if (bOk) {
						for (j = 0; j < sSerSeqRx.u16DataLen; j++) {
							vPutChar(&sSerStream, au8SerBuffRx[j]);
						}
					}
				} else if (sAppData.sFlash.sData.u8uart_mode == UART_MODE_CHAT || sAppData.sFlash.sData.u8uart_mode == UART_MODE_CHAT_NO_PROMPT) {
					// Chat モードは送り元情報付きで送信する。
					int i, len = 0;

					sSerCmdOut.u16len = sSerSeqRx.u16DataLen;

					// ハンドル名の取り出し（パケット末尾のデータ）
					for (i = sSerSeqRx.u16DataLen - 2; sSerCmdOut.au8data[i] != 0; i--) {
						len++;
					}
					if (len > 31 || (len + 2 >= sSerCmdOut.u16len)) { // ハンドル名が長すぎるか、パケットより大きい
						return;
					}

					if (len == 0) {
						// ハンドル名なし
						vfPrintf(&sSerStream, LB"[%08X:%d] ", sSerSeqRx.u32SrcAddr, sSerSeqRx.u8ReqNum);
						sSerCmdOut.u16len -= 2; // 本文の長さに設定
					} else {
						// ハンドル名あり
						vfPrintf(&sSerStream, LB"[%s:%d] ", &sSerCmdOut.au8data[i+1], sSerSeqRx.u8ReqNum);
						sSerCmdOut.u16len = i; // 本文の長さに設定
					}

					// 本文の出力
					sSerCmdOut.vOutput(&sSerCmdOut, &sSerStream);

					// プロンプトの表示
					vSerChatPrompt();
				} else {
					if (sSerCmdOut.au8data[1] == SERCMD_ID_TRANSMIT_EX) {
						// 拡張形式のパケットの出力
						const int8 I8HEAD = -12; // １２バイト分のヘッダを追加して出力

						uint8 *q = sSerCmdOut.au8data + I8HEAD; // このバッファは後ろ１６バイトまで有効
						S_OCTET(sSerSeqRx.u8IdSender);
						S_OCTET(sSerCmdOut.au8data[1]);
						S_OCTET(sSerSeqRx.u8RespID);

						S_BE_DWORD(u32AddrSrc);
						S_BE_DWORD(u32AddrDst);

						S_OCTET(pRx->u8Lqi);

						// 実ペイロード長を格納
						S_OCTET((sSerSeqRx.u16DataLen - 2) >> 8);
						S_OCTET((sSerSeqRx.u16DataLen - 2) & 0xFF);

						sSerCmdOut.u16len = sSerSeqRx.u16DataLen - I8HEAD;
						sSerCmdOut.au8data = sSerCmdOut.au8data + I8HEAD;
						sSerCmdOut.vOutput(&sSerCmdOut, &sSerStream);
						sSerCmdOut.au8data = sSerCmdOut.au8data - I8HEAD;
					} else {
						// 標準形式のパケットの出力
						sSerCmdOut.u16len = sSerSeqRx.u16DataLen;
						sSerCmdOut.au8data[0] = sSerSeqRx.u8IdSender; // １バイト目のアドレスだけ入れ替える
						sSerCmdOut.vOutput(&sSerCmdOut, &sSerStream);
					}
				}
			}

			memset(&sSerSeqRx, 0, sizeof(sSerSeqRx));
		}
	}
}

#if 0
/** @ingroup MASTER
 * データ送受信パケットの処理
 * tsRxDataApp *pRx
 */
static void vReceiveSerMsgAck(tsRxDataApp *pRx) {
	uint8 *p = pRx->auData;

	uint8 u8head;
	OCTET(u8head);

	// application ack.
	/*
	 * ACKNOWLEDGEMENT to the sender, to inform the completion of data arrival.
	 *
	 * [1] OCTET  : 0x01 fixed(OKAY)
	 * [1] OCTET  : request number (data ID)
	 * [2] LE_WORD: data length
	 * [1] OCTET  : xor of data
	 *
	 */

	uint8 u8req;
	uint16 u16len;
	uint8 u8xor;

	// READ PAYLOAD
	OCTET(u8req);
	LE_WORD(u16len);
	OCTET(u8xor);

	static int16 i16req_prev = -1;
	static uint32 u32tick_prev;

	if (u32TickCount_ms - u32tick_prev > 100) {
		i16req_prev = -1;
	}

	if (u8req != i16req_prev) {
		u32tick_prev = u32TickCount_ms;
		i16req_prev = u8req;

		// PREPARE SERIAL MESSAGE
		uint8 *q = au8SerOutBuff;
		S_OCTET(u8head);
		S_OCTET(u8req);
		S_BE_WORD(u16len); // NOTE: ON SERIAL, Big Endian is used.
		S_OCTET(u8xor);
		vSerResp(&sSerStream, au8SerOutBuff, q - au8SerOutBuff);

		// VERBOSE MESSAGE
		DBGOUT(3, LB ">>> AppAck(tick=%d,req=#%d,len=%d,xor=%d) <<<",
				u32TickCount_ms & 65535,
				u8req, u16len, u8xor
				);
	}
}
#endif

/** @ingroup FLASH
 * フラッシュ設定構造体をデフォルトに巻き戻す。
 * @param p 構造体へのアドレス
 */
static void vConfig_SetDefaults(tsFlashApp *p) {
	p->u32appid = APP_ID;
	p->u32chmask = CHMASK;
	p->u8ch = CHANNEL;
	p->u8power = 3;
	p->u8id = 0x00;
	p->u8role = E_APPCONF_ROLE_MAC_NODE;
	p->u8layer = 1;

	p->u32baud_safe = UART_BAUD_SAFE;
	p->u8parity = 0; // none

	p->u8uart_mode = UART_MODE_DEFAULT; // chat

	p->au8AesKey[0] = 0x00;
	p->au8ChatHandleName[0] = 0x00;

	p->u8Crypt = 0;
}

/** @ingroup FLASH
 * フラッシュ設定構造体を全て未設定状態に巻き戻す。
 * @param p 構造体へのアドレス
 */
static void vConfig_UnSetAll(tsFlashApp *p) {
	memset (p, 0xFF, sizeof(tsFlashApp));
}

/** @ingroup FLASH
 * フラッシュまたはEEPROMへの保存とリセットを行う。
 */
void vConfig_SaveAndReset() {
	tsFlash sFlash = sAppData.sFlash;

	if (sAppData.sConfig_UnSaved.u32appid != 0xFFFFFFFF) {
		sFlash.sData.u32appid = sAppData.sConfig_UnSaved.u32appid;
	}
	if (sAppData.sConfig_UnSaved.u32chmask != 0xFFFFFFFF) {
		sFlash.sData.u32chmask = sAppData.sConfig_UnSaved.u32chmask;
	}
	if (sAppData.sConfig_UnSaved.u8id != 0xFF) {
		sFlash.sData.u8id = sAppData.sConfig_UnSaved.u8id;
	}
	if (sAppData.sConfig_UnSaved.u8ch != 0xFF) {
		sFlash.sData.u8ch = sAppData.sConfig_UnSaved.u8ch;
	}
	if (sAppData.sConfig_UnSaved.u8power != 0xFF) {
		sFlash.sData.u8power = sAppData.sConfig_UnSaved.u8power;
	}
	if (sAppData.sConfig_UnSaved.u8layer != 0xFF) {
		sFlash.sData.u8layer = sAppData.sConfig_UnSaved.u8layer;
	}
	if (sAppData.sConfig_UnSaved.u8role != 0xFF) {
		sFlash.sData.u8role = sAppData.sConfig_UnSaved.u8role;
	}
	if (sAppData.sConfig_UnSaved.u32baud_safe != 0xFFFFFFFF) {
		sFlash.sData.u32baud_safe = sAppData.sConfig_UnSaved.u32baud_safe;
	}
	if (sAppData.sConfig_UnSaved.u8parity != 0xFF) {
		sFlash.sData.u8parity = sAppData.sConfig_UnSaved.u8parity;
	}
	if (sAppData.sConfig_UnSaved.u8uart_mode != 0xFF) {
		sFlash.sData.u8uart_mode = sAppData.sConfig_UnSaved.u8uart_mode;
	}

	{
		int i;
		for (i = 0; i < sizeof(sAppData.sConfig_UnSaved.au8ChatHandleName); i++) {
			if (sAppData.sConfig_UnSaved.au8ChatHandleName[i] != 0xFF) break;
		}
		if (i != sizeof(sAppData.sConfig_UnSaved.au8ChatHandleName)) {
			memcpy(sFlash.sData.au8ChatHandleName,
					sAppData.sConfig_UnSaved.au8ChatHandleName,
					sizeof(sAppData.sConfig_UnSaved.au8ChatHandleName));
		}
	}

	if (sAppData.sConfig_UnSaved.u8Crypt != 0xFF) {
		sFlash.sData.u8Crypt = sAppData.sConfig_UnSaved.u8Crypt;
	}

	{
		int i;
		for (i = 0; i < sizeof(sAppData.sConfig_UnSaved.au8AesKey); i++) {
			if (sAppData.sConfig_UnSaved.au8AesKey[i] != 0xFF) break;
		}
		if (i != sizeof(sAppData.sConfig_UnSaved.au8AesKey)) {
			memcpy(sFlash.sData.au8AesKey,
					sAppData.sConfig_UnSaved.au8AesKey,
					sizeof(sAppData.sConfig_UnSaved.au8AesKey));
		}
	}

	// アプリケーション情報の記録
	sFlash.sData.u32appkey = APP_ID;
	sFlash.sData.u32ver = ((VERSION_MAIN << 16) | (VERSION_SUB << 8) | (VERSION_VAR));

	bool_t bRet = bFlash_Write(&sFlash, FLASH_SECTOR_NUMBER - 1, 0);
	V_PRINT("!INF Write config %s"LB, bRet ? "Success" : "Failed");
	vConfig_UnSetAll(&sAppData.sConfig_UnSaved);
	vWait(100000);

	V_PRINT("!INF RESET SYSTEM...");
	vWait(1000000);
	vAHI_SwReset();
}

#ifdef USE_DIO_SLEEP
/** @ingroup MASTER
 * スリープの実行
 * @param u32SleepDur_ms スリープ時間[ms]
 * @param bPeriodic TRUE:前回の起床時間から次のウェイクアップタイミングを計る
 * @param bDeep TRUE:RAM OFF スリープ
 */
static void vSleep(uint32 u32SleepDur_ms, bool_t bPeriodic, bool_t bDeep) {
	// print message.

	// stop interrupt source, if interrupt source is still running.
	;

	// set UART Rx port as interrupt source
	vAHI_DioSetDirection(PORT_SLEEP_WAKE_MASK, 0); // set as input

	(void)u32AHI_DioInterruptStatus(); // clear interrupt register
	vAHI_DioWakeEnable(PORT_SLEEP_WAKE_MASK, 0); // also use as DIO WAKE SOURCE
	// vAHI_DioWakeEdge(0, PORT_INPUT_MASK); // 割り込みエッジ（立下りに設定）
	vAHI_DioWakeEdge(PORT_SLEEP_WAKE_MASK, 0); // 割り込みエッジ（立上がりに設定）
	// vAHI_DioWakeEnable(0, PORT_INPUT_MASK); // DISABLE DIO WAKE SOURCE

	// wake up using wakeup timer as well.
	ToCoNet_vSleep(E_AHI_WAKE_TIMER_0, u32SleepDur_ms, bPeriodic, bDeep); // PERIODIC RAM OFF SLEEP USING WK0
}
#endif


/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
