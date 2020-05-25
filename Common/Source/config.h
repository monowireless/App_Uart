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

#ifndef  CONFIG_H_INCLUDED
#define  CONFIG_H_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <AppHardwareApi.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
/* Serial Configuration */
#define UART_BAUD			115200UL //!< UART のボーレート（デフォルト）
#define UART_BAUD_SAFE		38400UL //!< UART のボーレート（他の設定）
#define UART_PARITY_ENABLE	E_AHI_UART_PARITY_DISABLE //!< パリティは none
#define UART_PARITY_TYPE 	E_AHI_UART_EVEN_PARITY //!< E_AHI_UART_PARITY_ENABLE 時の指定
#define UART_STOPBITS 		E_AHI_UART_1_STOP_BIT //!< ストップビットは１

/* Specify which serial port to use when outputting debug information */
#define UART_PORT_MASTER    E_AHI_UART_0 //!< UARTポートの指定

/* Specify the PAN ID and CHANNEL to be used by tags, readers and gateway */
#define APP_ID              0x67720103 //!< アプリケーションID。同じIDでないと通信しない。
//#define CHANNEL             17
//#define CHMASK              ((1UL << 11) | (1UL << 17) | (1UL << 25))
#define CHANNEL 18 //!< 使用するチャネル
#define CHMASK (1UL << CHANNEL) //!< チャネルマスク（３つまで指定する事が出来る）

// SERIAL BUFFERS
#define SERCMD_SER_PKTLEN 80 //!< シリアルメッセージのデータ部の最大バイト数
#define SERCMD_SER_PKTNUM 8 //!< シリアルメッセージの最大送信パケット数
#define SERCMD_MAXPAYLOAD (SERCMD_SER_PKTLEN*SERCMD_SER_PKTNUM) //!< シリアルメッセージのバッファサイズ

// その他の設定
#define USE_MODE_PIN
#define USE_BPS_PIN
#define UART_MODE_DEFAULT 3 //!< 0:Transparent, 1:Ascii format, 2:Binary, 3:Chat

#define DEFAULT_TX_FFFF_COUNT 0x82 //!< デフォルトの再送回数
#define DEFAULT_TX_FFFF_DUR_ms 4 //!< 再送時の間隔
#define DEFAULT_TX_FFFF_DELAY_ON_REPEAT_ms 20 //!< 中継時の遅延

#undef NWK_LAYER
#undef NWK_LAYER_FORCE
#define USE_AES
#define USE_DIO_SLEEP

// 設定セット
#undef CONFIG_000_1

/* このセットでは、
 *   UART1, 38400bps 8N1 で動作させる
 */
#ifdef CONFIG_000_1
#undef UART_BAUD
#define UART_BAUD			38400UL //!< UART のボーレート（デフォルト）
#undef UART_BAUD_SAFE
#define UART_BAUD_SAFE		9600UL //!< UART のボーレート（他の設定）
#undef UART_PORT_MASTER
#define UART_PORT_MASTER    E_AHI_UART_1 //!< UARTポートの指定
#undef USE_MODE_PIN
//#undef USE_BPS_PIN
#undef MODE_DEFAULT
#define MODE_DEFAULT 1 //!< 0:Transparent, 1:Ascii format, 2:Binary
#endif

#ifdef CONFIG_000_0
#undef UART_BAUD
#define UART_BAUD			38400UL //!< UART のボーレート（デフォルト）
#undef UART_BAUD_SAFE
#define UART_BAUD_SAFE		9600UL //!< UART のボーレート（他の設定）
#undef UART_PORT_MASTER
#define UART_PORT_MASTER    E_AHI_UART_0 //!< UARTポートの指定
#undef USE_MODE_PIN
//#undef USE_BPS_PIN
#undef MODE_DEFAULT
#define MODE_DEFAULT 1 //!< 0:Transparent, 1:Ascii format, 2:Binary
#endif

#ifdef CONFIG_001
#undef UART_PORT_MASTER
#define UART_PORT_MASTER    E_AHI_UART_0 //!< UARTポートの指定
#undef USE_MODE_PIN
#undef USE_BPS_PIN
#undef MODE_DEFAULT
#define MODE_DEFAULT 1 //!< 0:Transparent, 1:Ascii format, 2:Binary
#endif

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/**
 * 分割パケットを管理する構造体
 */
typedef struct {
	bool_t bPktStatus[SERCMD_SER_PKTNUM]; //!< パケットの受信フラグ（全部１になれば完了）
	uint8 u8PktNum; //!< 分割パケット数
	uint16 u16DataLen; //!< データ長

	uint8 u8RespID; //!< 応答を返すためのID(外部から指定される値)
	uint8 u8ReqNum; //!< 内部管理の送信ID
	uint8 u8Seq; //!< パケットのシーケンス番号（先頭）

	bool_t bResponse; //!< 応答返すかどうかのフラグ

	uint32 u32Tick; //!< タイムスタンプ

	uint8 u8IdSender; //!< 送り元簡易アドレス
	uint8 u8IdReceiver; //!< 宛先簡易アドレス
	uint32 u32SrcAddr; //!< 送り元拡張アドレス
	uint32 u32DstAddr; //!< 宛先拡張アドレス

	bool_t bParallel; //!< bWaitComplete を有効化させるかのフラグ
	bool_t bWaitComplete; //!< 終了フラグ

	bool_t bRelayPacket; //!< 中継パケットが含まれる？この場合再中継しない。
} tsSerSeq;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif  /* CONFIG_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
