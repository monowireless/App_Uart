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

/** @file
 * アプリケーションのメイン処理
 *
 * @defgroup MASTER アプリケーションのメイン処理
 */

#ifndef  MASTER_H_INCLUDED
#define  MASTER_H_INCLUDED

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <stdlib.h>
#include <string.h>

#include <jendefs.h>
#include <AppHardwareApi.h>

#include "ToCoNet.h"
#include "flash.h"
#include "btnMgr.h"

#include "common.h"

/** @ingroup MASTER
 * 使用する無線チャネル数の最大値 (複数設定すると Channel Agility を利用する)
 */
#define MAX_CHANNELS 3

/** @ingroup MASTER
 * ネットワークのモード列挙体 (ショートアドレス管理か中継ネットワーク層を使用したものか)
 */
typedef enum {
	E_NWKMODE_MAC_DIRECT,//!< ネットワークは構成せず、ショートアドレスでの通信を行う
	E_NWKMODE_LAYERTREE  //!< 中継ネットワークでの通信を行う
} teNwkMode;

/** @ingroup MASTER
 * アプリケーションの情報
 */
typedef struct {
	// ToCoNet
	uint32 u32ToCoNetVersion; //!< ToCoNet のバージョン番号を保持
	uint16 u16ToCoNetTickDelta_ms; //!< ToCoNet の Tick 周期 [ms]
	uint8 u8AppIdentifier; //!< AppID から自動決定

	// DEBUG
	uint8 u8DebugLevel; //!< デバッグ出力のレベル

	// Wakeup
	bool_t bWakeupByButton; //!< TRUE なら起床時に DI 割り込みにより起床した

	// mode3 fps
	uint8 u8FpsBitMask; //!< mode=3 連続送信時の秒間送信タイミングを判定するためのビットマスク (64fps のカウンタと AND を取って判定)

	// Network mode
	teNwkMode eNwkMode; //!< ネットワークモデル(未使用：将来のための拡張用)
	uint8 u8AppLogicalId; //!< ネットワーク時の抽象アドレス 0:親機 1~:子機
	uint8 u8AppLogicalId_Pair; //!< 透過モードの相手先のアドレス
	uint16 u16ShAddr_Pair; //!< 透過モードの相手先のショートアドレス

	// Network context
	tsToCoNet_Nwk_Context *pContextNwk; //!< ネットワークコンテキスト(未使用)
	tsToCoNet_NwkLyTr_Config sNwkLayerTreeConfig; //!< LayerTree の設定情報(未使用)

	// Flash Information
	tsFlash sFlash; //!< フラッシュからの読み込みデータ
	tsFlashApp sConfig_UnSaved; //!< フラッシュへの設定データ (0xFF, 0xFFFFFFFF は未設定)
	int8 bFlashLoaded; //!< フラッシュからの読み込みが正しく行われた場合は TRUE

	// config mode
	uint8 u8Mode; //!< 動作モード(IO M1,M2,M3 から設定される)

	// button manager
	tsBTM_Config sBTM_Config; //!< ボタン入力（連照により状態確定する）管理構造体
	PR_BTM_HANDLER pr_BTM_handler; //!< ボタン入力用のイベントハンドラ (TickTimer 起点で呼び出す)
	uint32 u32BTM_Tick_LastChange; //!< ボタン入力で最後に変化が有ったタイムスタンプ (操作の無効期間を作る様な場合に使用する)
	uint8 u8PortNow; //!< 現在のボタンの状況

	// Counter
	uint32 u32CtTimer0; //!< 64fps カウンタ。スリープ後も維持
	uint16 u16CtTimer0; //!< 64fps カウンタ。起動時に 0 クリアする

	uint8 u8UartReqNum; //!< UART 送信の要求番号を管理
	uint8 u8UartSeqNext; //!< UART 送信中の SEQ# の次の値 (0..127でループする)
} tsAppData;

/****************************************************************************
 * モード情報
 ***************************************************************************/
#define UART_MODE_TRANSPARENT 0
#define UART_MODE_ASCII 1
#define UART_MODE_BINARY 2
#define UART_MODE_CHAT 3
#define UART_MODE_CHAT_NO_PROMPT 4

/****************************************************************************
 * フラッシュ設定情報
 ***************************************************************************/

#define FL_MASTER_u32(c) sAppData.sFlash.sData.u32##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_UNSAVE_u32(c) sAppData.sConfig_UnSaved.u32##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_IS_MODIFIED_u32(c) (sAppData.sConfig_UnSaved.u32##c != 0xFFFFFFFF)  //!< 構造体要素アクセス用のマクロ @ingroup FLASH

#define FL_MASTER_u16(c) sAppData.sFlash.sData.u16##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_UNSAVE_u16(c) sAppData.sConfig_UnSaved.u16##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_IS_MODIFIED_u16(c) (sAppData.sConfig_UnSaved.u16##c != 0xFFFF)  //!< 構造体要素アクセス用のマクロ @ingroup FLASH

#define FL_MASTER_u8(c) sAppData.sFlash.sData.u8##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_UNSAVE_u8(c) sAppData.sConfig_UnSaved.u8##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_IS_MODIFIED_u8(c) (sAppData.sConfig_UnSaved.u8##c != 0xFF) //!< 構造体要素アクセス用のマクロ @ingroup FLASH

/** @ingroup FLASH
 * フラッシュ設定内容の列挙体
 */
enum {
	E_APPCONF_APPID,     //!<
	E_APPCONF_CHMASK,    //!<
	E_APPCONF_POWER,    //!<
	E_APPCONF_ID,        //!<
	E_APPCONF_ROLE,      //!<
	E_APPCONF_LAYER ,    //!<
	E_APPCONF_SLEEP4,    //!<
	E_APPCONF_SLEEP7,    //!<
	E_APPCONF_FPS,       //!<
	E_APPCONF_SYS_HZ,    //!<
	E_APPCONF_BAUD_SAFE, //!<
	E_APPCONF_BAUD_PARITY,
	E_APPCONF_UART_MODE,
	E_APPCONF_CRYPT_MODE,
	E_APPCONF_CRYPT_KEY,
	E_APPCONF_HANDLE_NAME,
	E_APPCONF_TEST
};

/** @ingroup FLASH
 * フラッシュ設定で ROLE に対する要素名の列挙体
 * (未使用、将来のための拡張のための定義)
 */
enum {
	E_APPCONF_ROLE_MAC_NODE = 0,  //!< MAC直接のノード（親子関係は無し）
	E_APPCONF_ROLE_MAC_NODE_REPEATER, //!< リピーター定義
	E_APPCONF_ROLE_NWK_MASK = 0x10, //!< NETWORKモードマスク
	E_APPCONF_ROLE_PARENT,          //!< NETWORKモードの親
	E_APPCONF_ROLE_ROUTER,        //!< NETWORKモードの子
	E_APPCONF_ROLE_ENDDEVICE,     //!< NETWORKモードの子（未使用、スリープ対応）
	E_APPCONF_ROLE_SILENT = 0x7F, //!< 何もしない（設定のみ)
};

/** サイレントモードの判定マクロ  @ingroup FLASH */
#define IS_APPCONF_ROLE_SILENT_MODE() (sAppData.sFlash.sData.u8role == E_APPCONF_ROLE_SILENT)

/** リピーターの判定 @ingroup FLASH */
#define IS_REPEATER() (sAppData.sFlash.sData.u8role == E_APPCONF_ROLE_MAC_NODE_REPEATER)
/** 専業リピーターの判定 @ingroup FLASH */
#define IS_DEDICATED_REPEATER() (IS_REPEATER() && IS_LOGICAL_ID_REPEATER(sAppData.u8AppLogicalId))
/** 子機兼リピーターの判定 @ingroup FLASH */
#define IS_CHILD_REPEATER() (IS_REPEATER() && IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId))

/** AES 利用のマクロ判定  @ingroup FLASH */
#define IS_CRYPT_MODE() (sAppData.sFlash.sData.u8Crypt)

#define CBID_MASK_BASE 0x3F
#define CBID_MASK_SPLIT_PKTS 0x40
#define CBID_MASK_SILENT 0x80

#endif  /* MASTER_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
