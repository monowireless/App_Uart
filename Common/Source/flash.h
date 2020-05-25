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
 *
 * @defgroup FLASH FLASHメモリの読み書き関数群
 * FLASH への読み書き関数
 */

#ifndef FLASH_H_
#define FLASH_H_

#if defined(JN514x)
#define FLASH_TYPE E_FL_CHIP_AUTO
#define FLASH_SECTOR_SIZE (64L* 1024L) // 64KB
#define FLASH_SECTOR_NUMBER 8 // 0..7
#elif defined(JN516x)
#define FLASH_TYPE E_FL_CHIP_INTERNAL
#define FLASH_SECTOR_SIZE (32L* 1024L) // 32KB
#define FLASH_SECTOR_NUMBER 5 // 0..4
#endif

/** @ingroup FLASH
 * フラッシュ格納データ構造体
 */
typedef struct _tsFlashApp {
	uint32 u32appkey;		//!<
	uint32 u32ver;			//!<

	uint32 u32appid;		//!< アプリケーションID
	uint32 u32chmask;		//!< 使用チャネルマスク（３つまで）
	uint8 u8id;				//!< 論理ＩＤ (子機 1～100まで指定)
	uint8 u8ch;				//!< チャネル（未使用、チャネルマスクに指定したチャネルから選ばれる）
	uint8 u8power;			//!< 送信パワー
	uint8 u8role;			//!< 未使用(将来のための拡張)
	uint8 u8layer;			//!< 未使用(将来のための拡張)

	uint32 u32baud_safe;	//!< ボーレート
	uint8 u8parity;         //!< パリティ 0:none, 1:odd, 2:even

	uint8 u8uart_mode;		//!< UART の動作モード (0:透過, 1:テキスト電文, 2:バイナリ電文)

	uint8 au8ChatHandleName[32]; //!< チャットモードのハンドル名

	uint8 u8Crypt;          //!< 暗号化を有効化するかどうか (1:AES128)
	uint8 au8AesKey[33];    //!< AES の鍵
} tsFlashApp;

/** @ingroup FLASH
 * フラッシュデータ構造体
 * - u32Magic と u8CRC により、書き込みデータが有為かどうか判定する
 * - u8CRC は データ中の CRC8 チェックサム
 */
typedef struct _tsFlash {
	uint32 u32Magic;
	tsFlashApp sData;
	uint8 u8CRC;
} tsFlash;

bool_t bFlash_Read(tsFlash *psFlash, uint8 sector, uint32 offset);
bool_t bFlash_Write(tsFlash *psFlash, uint8 sector, uint32 offset);

#endif /* FLASH_H_ */
