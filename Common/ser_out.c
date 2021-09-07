/* Copyright (C) 2021 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-*J,*E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT).                                                   */

#include "ser_out.h"

/*
 * 外部依存変数
 */
extern tsAppData sAppData;
extern tsFILE sSerStream;
extern tsSerCmd_Context sSerCmd;
extern tsSerCmd_Context sSerCmd_Slave;

uint16 u16Time_sec = 0;

/** @ingroup SEROUT
 * 始動時メッセージの表示
 */
void vSerInitMessage() {
	// 始動メッセージ
	if (sAppData.u8uart_mode == UART_MODE_CHAT) {
		vfPrintf(&sSerStream, LB"!INF TWE UART APP V%d-%02d-%d, SID=0x%08X, LID=0x%02x",
				VERSION_MAIN, VERSION_SUB, VERSION_VAR, ToCoNet_u32GetSerial(), sAppData.u8AppLogicalId);
		vSerChatPrompt();
	} else
	if (sAppData.u8uart_mode == UART_MODE_CHAT_HEADER) {
		tsRxDataApp sRx;
		tsSerSeq sSerSeq;
		uint8 au8StartMsg[40];
		uint8 u8StartMsgLen;

		sSerSeq.u8IdSender = SERCMD_ADDR_FR_MODULE;
		sSerSeq.u32SrcAddr = ToCoNet_u32GetSerial();
		memset( &sRx, 0, sizeof(tsRxDataApp) );

		// デフォルトの書式で出力するためにハンドルネームの0バイト目を退避させる
//		uint8 tmp = sAppData.sFlash.sData.au8ChatHandleName[0];
//		sAppData.sFlash.sData.au8ChatHandleName[0] = 0;

		// ペイロードの作成。
		SPRINTF_vRewind();
		//                        012345678901234567890123456789
		vfPrintf(SPRINTF_Stream, "!INF APP_UART V%d-%02d-%d/%03d", VERSION_MAIN, VERSION_SUB, VERSION_VAR, sAppData.u8AppLogicalId);
		u8StartMsgLen = SPRINTF_u16Length();
		memcpy(au8StartMsg, SPRINTF_pu8GetBuff(), u8StartMsgLen);

		vSerOutputFmt( au8StartMsg, u8StartMsgLen, &sSerSeq, &sRx );

		// 0バイト目を元に戻す
//		sAppData.sFlash.sData.au8ChatHandleName[0] = tmp;
			
	} else
	if (sAppData.u8uart_mode == UART_MODE_ASCII || sAppData.u8uart_mode == UART_MODE_BINARY) {
		// ASCII, BINARY モードでは自分のアドレスを表示する
		vSerResp_GetModuleAddress();
	}
}

/**  @ingroup SEROUT
 * Chatモードのプロンプト表示
 */
void vSerChatPrompt() {
	if (sAppData.u8uart_mode == UART_MODE_CHAT) {
		if (sAppData.sFlash.sData.au8ChatHandleName[0] == 0x00) {
			vfPrintf(&sSerStream, LB"%08X:%d> ", ToCoNet_u32GetSerial(), sAppData.u8UartReqNum);
		} else {
			vfPrintf(&sSerStream, LB"%s:%d> ", sAppData.sFlash.sData.au8ChatHandleName, sAppData.u8UartReqNum);
		}

		// 入力中なら表示を回復する
		if (sSerCmd.u8state < 0x80 && sSerCmd.u8state != E_SERCMD_EMPTY) {
			sSerCmd.vOutput(&sSerCmd, &sSerStream);
		} else
		if (sSerCmd_Slave.u8state < 0x80 && sSerCmd_Slave.u8state != E_SERCMD_EMPTY) {
			sSerCmd_Slave.vOutput(&sSerCmd_Slave, &sSerStream);
		}
	}
}

/**  @ingroup SEROUT
 * Dモード時の出力（書式設定）
 *
 *  *   : ペイロードをそのまま出力
 *  #   : バイナリ出力とトグル (%? がバイナリのデータとして出力される)
 *  <, >: XOR の計算ポイント (%X は <> の後に配置すること)
 *  &hl : 文字コードhlに対応する文字を出力する (hl は16進文字)
 *  
 *  \n   : CRLF
 *  \t   : NUL
 *  \A-\Z: 0x00-0x19 の制御コード出力
 * 
 *  %%    : %
 *  %A %a : SIDの出力 (4bytes)
 *  %I %i : LIDの出力 (1byte)
 *  %Q %q : LQIの出力 (1byte)
 *  %Z %z : ペイロードのバイト数の出力 (1byte)
 *  %X %x : XORの出力 (1byte) 
 *          ※範囲指定がない場合は出力の先頭から現在位置まで。
 *  %T %t : 現在の時刻の出力 (2byte)
 *  %S %s : シーケンス番号の出力 (1byte)
 *  %P %p : 区切り文字の出力(1~2byte)
 * 
 *  ※ 上記は大文字で指定の場合は16進数(4bytes 8桁,2bytes 4桁,1byte 2桁)、
 *           小文字で指定の場合は10進数(4bytes10桁,2bytes 5桁,1byte 3桁)
 */
void vSerOutputFmt(uint8 *pOutputData, uint16 u16len, tsSerSeq *pSerSeqRx,  tsRxDataApp *pRx) {
	uint8 au8outbuff[256], *q = au8outbuff, *q_end = au8outbuff + sizeof(au8outbuff) - 16;
	uint8 *p;

	uint8 *p_xor_start = au8outbuff;
	uint8 *p_xor_end = NULL;
	
	if (sAppData.sFlash.sData.au8ChatHandleName[0] == 0) {
		//           0123456789012345678901234567890123456789
		p = (uint8*)";U;%t;%i;0x%A;%q;%s;<*>;%X;\\n";
	} else {
		p = sAppData.sFlash.sData.au8ChatHandleName;
	}

	bool_t bBinary = FALSE;
	uint8_t bSglQuote = 0;

	uint8 c;

	do {
		c = G_OCTET();

		if (bSglQuote == 1) { // '\\'
			bSglQuote = 0;

			if (c >= 'A' && c <= 'Z') {
				*q++ = (c - 'A');
			} else
			switch(c) {

			/*
			 * ヌル文字
			 */
			case '0':
				*q++ = 0;
				break;

			/*
			 * CRLF の出力
			 */
			case 'n':
				*q++ = '\r';
				*q++ = '\n';
				break;

			/*
			 * TAB
			 */
			case 't': // TAB
				*q++ = 0x09;
				break;

			/*
			 * そのまま出力 (\* -> *)
			 */
			default:
				S_OCTET(c);
				break;
			}
		} else
		if (bSglQuote == 2) { // %
			bSglQuote = 0;

			switch(c) { 
			case '%': S_OCTET(c); break; // %% -> %

			/*
			 * 送信元のアドレス
			 */ 
			case 'A': case 'a':
				if (bBinary) {
					S_BE_DWORD(pSerSeqRx->u32SrcAddr);
				} else {
					SPRINTF_vRewind();
					vfPrintf(SPRINTF_Stream, c <= 'Z' ? "%08X" : "%010u", (unsigned int)pSerSeqRx->u32SrcAddr);
					memcpy(q, SPRINTF_pu8GetBuff(), SPRINTF_u16Length());
					q += SPRINTF_u16Length();
				}
				break;

			/*
			 * 送信元のロジカルIDを出力
			 */ 
			case 'I': case 'i':
				if (bBinary) {
					S_OCTET(pSerSeqRx->u8IdSender);
				} else {
					SPRINTF_vRewind();
					vfPrintf(SPRINTF_Stream, c <= 'Z' ? "%02X" : "%03u", pSerSeqRx->u8IdSender);
					memcpy(q, SPRINTF_pu8GetBuff(), SPRINTF_u16Length());
					q += SPRINTF_u16Length();
				}
				break;

			/*
			 * 現在のTickTimerの値を秒で出力する。
			 */ 
			case 'T': case 't':
				_C{
					if(bBinary){
						S_BE_WORD(u16Time_sec);
					}else{
						SPRINTF_vRewind();
						vfPrintf(SPRINTF_Stream, c <= 'Z' ? "%04X" : "%05u", u16Time_sec);
						memcpy(q, SPRINTF_pu8GetBuff(), SPRINTF_u16Length());
						q += SPRINTF_u16Length();	
					}
				}
				break;

			/*
			 * シーケンス番号を出力する。
			 */
			case 'S': case 's':
				if (bBinary) {
					S_OCTET(pRx->u8Seq);
				} else {
					SPRINTF_vRewind();
					vfPrintf(SPRINTF_Stream, c <= 'Z' ? "%02X" : "%03u", pRx->u8Seq);
					memcpy(q, SPRINTF_pu8GetBuff(), SPRINTF_u16Length());
					q += SPRINTF_u16Length();
				}
				break;

			/*
			 * LQI を出力する。
			 */
			case 'Q': case 'q':
				if (bBinary) {
					S_OCTET(pRx->u8Lqi);
				} else {
					SPRINTF_vRewind();
					vfPrintf(SPRINTF_Stream, c <= 'Z' ? "%02X" : "%03u", pRx->u8Lqi);
					memcpy(q, SPRINTF_pu8GetBuff(), SPRINTF_u16Length());
					q += SPRINTF_u16Length();
				}
				break;

			/*
			 * ペイロード長さを出力
			 */
			case 'Z': case 'z':
				if (bBinary) {
					S_BE_WORD(u16len);
				} else {
					SPRINTF_vRewind();
					vfPrintf(SPRINTF_Stream, c <= 'Z' ? "%02X" : "%03u", u16len);
					memcpy(q, SPRINTF_pu8GetBuff(), SPRINTF_u16Length());
					q += SPRINTF_u16Length();
				}
				break;

			/*
			 * チェックサムの出力(ペイロードではなく出力バイト列に対して)
			 */
			case 'X': case 'x':
				_C {
					uint8 x = 0, *r = p_xor_start, *q_end = p_xor_end;
					if (q_end == NULL) q_end = q;

					while (r < q) {
						x ^= *r;
						r++;
					}
					if (bBinary) {
						S_OCTET(x);
					} else {
						SPRINTF_vRewind();
						vfPrintf(SPRINTF_Stream, c <= 'Z' ? "%02X" : "%03u", x);
						memcpy(q, SPRINTF_pu8GetBuff(), SPRINTF_u16Length());
						q += SPRINTF_u16Length();
					}
				}
				break;
			/* 
			 * (今設定されている)区切り文字の出力
			 */
			case 'P': case 'p':
				_C{
					if(IS_APPCONF_OPT_TX_TRIGGER_CHAR()){
						if(sAppData.sFlash.sData.u16uart_lnsep < 0x0100){
							S_OCTET(sAppData.sFlash.sData.u16uart_lnsep&0xFF);
						}else{
							S_BE_WORD(sAppData.sFlash.sData.u16uart_lnsep);
						}
					}
				}
				break;

			/*
			 * 未解釈は % 付きでそのまま出力
			 */
			default:
				*q++ = '?';
				*q++ = '%';
				*q++ = c;
				*q++ = '?';
				break;
			}
		} else {
			switch(c) {
			/*
			 * 制御文字開始
			 */				
			case '\\': bSglQuote = 1; break;
			
			/*
			 * 制御文字開始
			 */				
			case '%': bSglQuote = 2; break;
			
			/*
			 * XOR 計算開始終了位置を指定
			 */
			case '<': p_xor_start = q; break; // XOR の計算開始ポイント
			case '>': p_xor_end = q; break; // XOR の計算開始ポイント
			
			/*
			 * バイナリー出力をトグル
			 */
			case '#': bBinary = !bBinary; break;

			/*
			 * ペイロードを出力する
			 */
			case '*':
				_C {
					if (q + u16len < q_end) {
						memcpy(q, pOutputData, u16len);
						q += u16len;
					}
				}
				break;

			/*
			 * 任意の16進に対応する文字を出力する
			 *   &20 -> 0x20(SPACE) を出力する
			 */
			case '&':
				_C {
					uint8 v = 0;
					int i;
					for (i = 0; i < 2; i++) {
						v <<= 4;
						uint8 h = G_OCTET();

						if (h >= '0' && h <= '9') {
							v += (h - '0');
						} else if (h >= 'A' && h <= 'F') {
							v += (h - 'A' + 10);
						} else if (h >= 'a' && h <= 'f') {
							v += (h - 'a' + 10);
						}
					}
					S_OCTET(v);
				}
				break;

			default: // そのまま出力
				S_OCTET(c);
			}
		}
	} while (c != 0 && q < q_end);

	// UART 出力(系列が分断されるのを防ぐため一気に出力)
	p = au8outbuff;
	while(p < q) {
		vPutChar(&sSerStream, *p);
		p++;
	}
}

/**  @ingroup SEROUT
 * 稼働時間の更新
 */
uint16 u16SerTimeUpdate()
{
	u16Time_sec++;
	return u16Time_sec;
}