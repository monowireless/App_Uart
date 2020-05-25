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

#ifndef APP_EVENT_H_
#define APP_EVENT_H_

#include "ToCoNet_event.h"

/** @ingroup MASTER
 * アプリケーション内で使用するイベント
 */
typedef enum
{
	E_EVENT_APP_BASE = ToCoNet_EVENT_APP_BASE,//!< ToCoNet 組み込みイベントと重複させないため
    E_EVENT_APP_TICK_A,                    //!< 64FPS のタイマーイベント
    E_EVENT_APP_ADC_COMPLETE,              //!< ADC完了
    E_EVENT_APP_TX_COMPLETE                //!< TX完了
} teEventApp;

/** @ingroup MASTER
 * アプリケーション内で使用する状態名
 */
typedef enum
{
	E_STATE_APP_BASE = ToCoNet_STATE_APP_BASE,//!< ToCoNet 組み込み状態と重複させないため
	E_STATE_APP_WAIT_IO_FIRST_CAPTURE,        //!< 最初のADCやDIの状態確定を待つ
	E_STATE_APP_SLEEPING                      //!< スリープ処理
} teStateApp;

#endif /* EVENT_H_ */
