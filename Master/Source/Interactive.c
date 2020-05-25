/*
 * Interactive.c
 *
 *  Created on: 2014/04/21
 *      Author: seigo13
 */

#include <string.h>

#include "Master.h"

#include "config.h"

#include "Version.h"

#include "utils.h"
#include "input_string.h"

#include "flash.h"

#include "common.h"
#include "config.h"

// Serial options
#include <serial.h>
#include <fprintf.h>
#include <sprintf.h>

#include "sercmd_plus3.h"
#include "sercmd_gen.h"

#include "Interactive.h"

extern tsAppData sAppData;
extern tsSerCmdPlus3_Context sSerCmd_P3;
extern tsFILE sSerStream;
tsInpStr_Context sSerInpStr;

extern uint16 u16HoldUpdateScreen;

/** @ingroup FLASH
 * フラッシュ設定構造体をデフォルトに巻き戻す。
 * @param p 構造体へのアドレス
 */
void vConfig_SetDefaults(tsFlashApp *p) {
	p->u32appid = APP_ID;
	p->u32chmask = CHMASK;
	p->u8ch = CHANNEL;
	p->u16power = 3;
	p->u8id = 0x00;
	p->u8role = E_APPCONF_ROLE_MAC_NODE;
	p->u8layer = 1;

	p->u32baud_safe = UART_BAUD_SAFE;
	p->u8parity = 0; // none

	p->u8uart_mode = UART_MODE_DEFAULT; // chat

	p->au8AesKey[0] = 0x00;
	p->au8ChatHandleName[0] = 0x00;

	p->u8Crypt = 0;
	p->u32Opt = 0;
}

/** @ingroup FLASH
 * フラッシュ設定構造体を全て未設定状態に巻き戻す。
 * @param p 構造体へのアドレス
 */
void vConfig_UnSetAll(tsFlashApp *p) {
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
	if (sAppData.sConfig_UnSaved.u16power != 0xFFFF) {
		sFlash.sData.u16power = sAppData.sConfig_UnSaved.u16power;
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

	if (sAppData.sConfig_UnSaved.u32Opt != 0xFFFFFFFF) {
		sFlash.sData.u32Opt = sAppData.sConfig_UnSaved.u32Opt;
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

/**  @ingroup MASTER
 * UART のオプションを表示する
 */
static void vSerPrintUartOpt(uint8 u8conf) {
	const uint8 au8name_bit[] = { '8', '7' };
	const uint8 au8name_parity[] = { 'N', 'O', 'E' };
	const uint8 au8name_stop[] = { '1', '2' };

	V_PRINT("%c%c%c",
				au8name_bit[u8conf & APPCONF_UART_CONF_WORDLEN_MASK ? 1 : 0],
				au8name_parity[u8conf & APPCONF_UART_CONF_PARITY_MASK],
				au8name_stop[u8conf & APPCONF_UART_CONF_STOPBIT_MASK ? 1 : 0]);
}


/** @ingroup MASTER
 * １バイト入力コマンドの処理
 * @param u8Byte 入力バイト
 */
void vProcessInputByte(uint8 u8Byte) {
	static uint8 u8lastbyte = 0xFF;

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

	case 'x': // チャネルの設定
#ifdef JN514x
		V_PRINT("Input Rf Power & Kbps"
				LB " X0YZ X=Kbps(0:250,1:500,2:667)"
				LB "      Y=Retry(0:default,F:0,1-9:count"
				LB "      Z=Power(3:Max,2,1,0:Min): ");
#elif defined(JN516x)
		V_PRINT("Input Rf Power & Kbps"
				LB "   YZ Y=Retry(0:default,F:0,1-9:count"
				LB "      Z=Power(3:Max,2,1,0:Min): ");
#endif
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_HEX, 4, E_APPCONF_POWER);
		break;

	case 'i': // set application role
		V_PRINT("Input Device ID (DEC:1-100): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 3, E_APPCONF_ID);
		break;

	case 'r': // システムの役割定義
		V_PRINT("Input Role ID (HEX): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_HEX, 2, E_APPCONF_ROLE);
		break;

	case 'l': // システムの役割定義
		V_PRINT("Input Layer (1-63): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 2, E_APPCONF_LAYER);
		break;

	case 'b': // ボーレートの変更
		V_PRINT("Input UART baud (DEC:9600-230400): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_STRING, 10, E_APPCONF_BAUD_SAFE);
		break;

	case 'B': // パリティの変更
		V_PRINT("Input UART option (e.g. 8N1): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_STRING, 3, E_APPCONF_BAUD_PARITY);
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

	case 'o':
		V_PRINT("Input option bits: ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_HEX, 8, E_APPCONF_OPT_BITS);
		break;

	case 'S':
		// フラッシュへのデータ保存
		if (u8lastbyte == 'R') {
			// R S と連続入力された場合は、フラッシュエリアを消去する
			V_PRINT("!INF CLEAR SAVE AREA.");
			bFlash_Erase(FLASH_SECTOR_NUMBER - 1); // SECTOR ERASE

			vWait(1000000);
			vAHI_SwReset();
		} else {
			// フラッシュへのデータ保存
			vConfig_SaveAndReset();
		}
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

				V_PRINT(LB "Nwk mode: %02x", sAppData.eNwkMode);
				if (sAppData.eNwkMode == E_NWKMODE_LAYERTREE) {
					tsToCoNet_NwkLyTr_Context *pc = (tsToCoNet_NwkLyTr_Context *)(sAppData.pContextNwk);

					V_PRINT(LB "Nwk Info: la=%d ty=%d ro=%02x st=%02x",
							pc->sInfo.u8Layer, pc->sInfo.u8NwkTypeId, pc->sInfo.u8Role, pc->sInfo.u8State);
					V_PRINT(LB "Nwk Parent: %08x", pc->u32AddrHigherLayer);
					V_PRINT(LB "Nwk LostParent: %d", pc->u8Ct_LostParent);
					V_PRINT(LB "Nwk SecRescan: %d, SecRelocate: %d", pc->u8Ct_Second_To_Rescan, pc->u8Ct_Second_To_Relocate);
				}
			} else {
				V_PRINT("** Conf: none"LB);
			}
		}
		break;

	default:
		break;
	}

	u8lastbyte = u8Byte;
}

/** @ingroup MASTER
 * 文字列入力モードの処理
 */
void vProcessInputString(tsInpStr_Context *pContext) {
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
						if (u8ch >= 11
#if defined(JN514x)
								&& u8ch <= 25
#elif defined(JN516x)
								&& u8ch <= 26
#endif
								) {
							if (n_ch) {
								V_PUTCHAR(',');
							}
							V_PRINT("%x", u8ch);
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
			uint32 u32val = u32string2hex(pu8str, u8idx);
			V_PRINT(LB"-> ");

			uint8 u8Bps = (u32val & 0xF000) >> 12;
			//uint8 u8Retry = (u32val & 0x00F0) >> 4;
			uint8 u8Pow = (u32val & 0x000F);

			if (u8Bps <= 2 && u8Pow <= 3) {
				sAppData.sConfig_UnSaved.u16power = u32val & 0xFFFF;
				V_PRINT("%x"LB, u32val);
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
			if (u32val == E_APPCONF_ROLE_MAC_NODE
				|| u32val == E_APPCONF_ROLE_MAC_NODE_REPEATER
				|| u32val == E_APPCONF_ROLE_PARENT
				|| u32val == E_APPCONF_ROLE_ROUTER
				|| u32val == E_APPCONF_ROLE_ENDDEVICE
			) {
				sAppData.sConfig_UnSaved.u8role = u32val; // ０は未設定！
				V_PRINT("(0x%02x)"LB, u32val, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_LAYER:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);
			V_PRINT(LB"-> ");
			if (u32val >= 1 && u32val <= 63) {
				sAppData.sConfig_UnSaved.u8layer = u32val; // ０は未設定！
				V_PRINT("(%02d)"LB, u32val);
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
			uint8 u8len = strlen((void*)pu8str);
			int i;

			// 既存の設定値を保存する
			uint8 u8parity = sAppData.sFlash.sData.u8parity & APPCONF_UART_CONF_PARITY_MASK;
			uint8 u8stop = sAppData.sFlash.sData.u8parity & APPCONF_UART_CONF_STOPBIT_MASK;
			uint8 u8wordlen = sAppData.sFlash.sData.u8parity & APPCONF_UART_CONF_WORDLEN_MASK;

			// １文字ずつ処理する（順番は任意でもよい）
			for (i = 0; i < u8len; i++) {
				switch(pu8str[i]) {
				case 'N': case 'n': u8parity = 0; break;
				case 'O': case 'o': u8parity = 1; break;
				case 'E': case 'e': u8parity = 2; break;
				case '1': u8stop = 0; break;
				case '2': u8stop = APPCONF_UART_CONF_STOPBIT_MASK; break;
				case '8': u8wordlen = 0; break;
				case '7': u8wordlen = APPCONF_UART_CONF_WORDLEN_MASK; break;
				default:
					break;
				}
			}

			// 新しい設定を計算する
			uint8 u8new = u8parity | u8stop | u8wordlen;
			if (u8new == sAppData.sFlash.sData.u8parity) {
				// 変化が無ければ無視
				V_PRINT("(ignored)");
			} else {
				sAppData.sConfig_UnSaved.u8parity = u8new;
				vSerPrintUartOpt(u8new);
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
				V_PRINT("(ignored)");
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

	case E_APPCONF_OPT_BITS:
		_C {
			uint32 u32val = u32string2hex(pu8str, u8idx);
			V_PRINT(LB"-> ");
			sAppData.sConfig_UnSaved.u32Opt = u32val;
			V_PRINT("(%08x)"LB, u32val);
		}
		break;

	default:
		break;
	}

	// 一定時間待って画面を再描画
	u16HoldUpdateScreen = 96; // 1.5sec
}

/** @ingroup MASTER
 * インタラクティブモードの画面を再描画する。
 */
void vSerUpdateScreen() {
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

	V_PRINT(" x: set RF Conf (%x)%c" LB,
			FL_IS_MODIFIED_u16(power) ? FL_UNSAVE_u16(power) : FL_MASTER_u16(power),
			FL_IS_MODIFIED_u16(power) ? '*' : ' ');

	V_PRINT(" r: set Role (0x%X)%c" LB,
			FL_IS_MODIFIED_u8(role) ? FL_UNSAVE_u8(role) : FL_MASTER_u8(role),
			FL_IS_MODIFIED_u8(role) ? '*' : ' ');

	V_PRINT(" l: set Layer (0x%X)%c" LB,
			FL_IS_MODIFIED_u8(layer) ? FL_UNSAVE_u8(layer) : FL_MASTER_u8(layer),
			FL_IS_MODIFIED_u8(layer) ? '*' : ' ');

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
		uint8 u8dat = FL_IS_MODIFIED_u8(parity) ? FL_UNSAVE_u8(parity) : FL_MASTER_u8(parity);

		V_PRINT(" p: set UART option (");
		vSerPrintUartOpt(u8dat);
		V_PRINT(")%c" LB,
					FL_IS_MODIFIED_u8(parity) ? '*' : ' ');
	}

	{
		const uint8 au8name[] = { 'T', 'A', 'B', 'C', 'D' };
		V_PRINT(" m: set UART mode (%c)%c" LB,
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

	V_PRINT(" o: set option bits (0x%08x)%c" LB,
			FL_IS_MODIFIED_u32(Opt) ? FL_UNSAVE_u32(Opt) : FL_MASTER_u32(Opt),
			FL_IS_MODIFIED_u32(Opt) ? '*' : ' ');

	V_PRINT("---"LB);

	V_PRINT(" S: save Configuration" LB " R: reset to Defaults" LB LB);
	//       0123456789+123456789+123456789+1234567894123456789+123456789+123456789+123456789
}
