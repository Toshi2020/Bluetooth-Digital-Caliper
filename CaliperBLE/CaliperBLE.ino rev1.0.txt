/*****************************************************************************
*
*	CaliperBLE.ino -- デジタルノギスデータBLE送信 XIAO nRF52840
*
*	Arduino IDEの設定は
*		追加のボードマネージャのURL：
*		https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
*		ボードマネージャ：Speeed nR52 boards
*		ボード:Speeed XIAO nRF52840
*		書き込み装置:Bootloadre DFU for Bluefruit nRF52
*		Pythonをインストールしたディレクトリを環境変数のpathに設定しておく
*		(参考)Pythonダウンロードリンク
*		https://pythonlinks.python.jp/ja/index.html
*		Add python.exe to PATHにチェックを入れてインストール
*
*	rev1.0	2025/09/29 initial revision by Toshi
*
*****************************************************************************/
#include <bluefruit.h>
#include <Adafruit_SPIFlash.h>
#include <nrf.h>
#include <nrf_lpcomp.h>

#define DEBUG 0
#define INVERT_LOGIC 0	// ノギス信号論理反転(NPNTrによるレベル変換)なら1
#define USE_LPCOMP 1	// ノギスクロック検出にLPCOMPを使うなら

// ピン設定
#define DATA_PIN A0		// ノギスデータ入力ピン
#define CLOCK_PIN A1	// ノギスクロック入力ピン(analogRead用)
#define SW_PIN D2		// スイッチ入力ピン

#define LED_ON digitalWrite(LED_GREEN, LOW);
#define LED_OFF digitalWrite(LED_GREEN, HIGH);

#if INVERT_LOGIC	// NPN Trによるレベル変換なら
 #define CLOCK_SPACE (digitalRead(CLOCK_PIN) == LOW)
 #define DATA_SPACE (digitalRead(DATA_PIN) == LOW)
#else
 #if USE_LPCOMP	// クロック検出にLPCOMPを使うなら
  #define CLOCK_SPACE (get_lpcomp(false))
 #else
  #define CLOCK_SPACE (analogRead(CLOCK_PIN) >= 256)
 #endif //USE_LPCOMP
 #define DATA_SPACE (analogRead(DATA_PIN) >= 256)
#endif //INVERT_LOGIC

#define DEBOUNCE_MS 30	// チャタリング除去時間[ms]
#define LONGPUSH_MS 300	// SW長押し判定時間[ms]
#define SYSOFFTIME (30 * 60000L)	// 未操作時オートパワーオフまでの時間[ms]
#define SYSOFFTIME2 (30 * 1000L)	// 未接続時オートパワーオフまでの時間[ms]

#if DEBUG
 #define Print(n); Serial1.print(n);
 #define Println(n); Serial1.println(n);
#else
 #define Print(n);
 #define Println(n);
#endif //DEBUG

// グローバル変数
BLEDis bledis;
BLEHidAdafruit blehid;
BLEBas blebas;
bool fRestart;	// オートパワーオフからの復帰フラグ
uint32_t RxErr;	// ノギス読み取りエラーコード

// 外部QSPI Flash Memory（省電力化のために使用）
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash flash(&flashTransport);

// プロトタイプ宣言
void connectCallback(uint16_t conn_handle);
bool get_lpcomp(bool finit);
uint32_t ReadCaliper();
void Transmit(uint32_t rxbit, bool flongpush);
uint16_t ReadVDD();
void NotifyBattery(uint16_t mv);

/*----------------------------------------------------------------------------
	セットアップ
----------------------------------------------------------------------------*/
void setup() 
{
	// リセット原因をクリア
	NRF_POWER->RESETREAS = NRF_POWER->RESETREAS;

	// ピンモードの設定
	pinMode(SW_PIN, INPUT_PULLUP);

	// オンボードLEDを消灯
	pinMode(LED_RED, OUTPUT);
	pinMode(LED_GREEN, OUTPUT);
	pinMode(LED_BLUE, OUTPUT);
	digitalWrite(LED_RED, HIGH);
	digitalWrite(LED_GREEN, HIGH);
	digitalWrite(LED_BLUE, HIGH);

	// オンボードFlashのピン電位を固定(フロートだと電流が増えるボードあり)
	pinMode(PIN_QSPI_SCK, INPUT_PULLUP);
	pinMode(PIN_QSPI_CS, INPUT_PULLUP);

	// チップ内2段目の3.3V→1.3VをLDOからDCDCに変更して効率UP
	NRF_POWER->DCDCEN  = 1;

	// 初回起動かどうか判定
	if (NRF_POWER->GPREGRET == 0)	// 最初の通電直後なら
	{
		NRF_POWER->GPREGRET = 1;	// SWを押して再起動した後は通常起動

		// 外部Flash MemoryをDeep Power-downモードに
		flashTransport.begin();
		flashTransport.runCommand(0xB9);
		delayMicroseconds(5);
		flashTransport.end();

		GotoSystemOff();	// システムオフ
	}
	else if (NRF_POWER->GPREGRET == 1)	// 通電後の初回のSWオンによる起動なら
	{
		NRF_POWER->GPREGRET = 2;	// 次回からはオートパワーオフからの復帰
	}
	else	// オートパワーオフからの復帰なら
	{
		fRestart = true;	// リスタートしたことを記録
	}

	#if DEBUG
	Serial1.begin(115200);
	#endif //DEBUG

	// DWT(Data Watchpoint and Trace)初期化(自前のmicrosのため)
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	// BLE開始
	setup_ble();

	#if USE_LPCOMP	// クロック検出にLPCOMPを使うなら
	// LPCOMP初期化
	setup_lpcomp();
	#endif //USE_LPCOMP
}
/*----------------------------------------------------------------------------
	メインループ
----------------------------------------------------------------------------*/
void loop()
{
	uint32_t t0;
	uint32_t rxbit;
	bool flongpush;
	static uint32_t noptime = 0;	// 操作していない時間[ms]
	static uint32_t nobletime = 0;	// BLE未接続時間[ms]

	// BLE接続が確定している？
	if (Bluefruit.Periph.connected())
	{
		// キーが押されたかオートパワーオフからの復帰
		if (digitalRead(SW_PIN) == LOW || fRestart)
		{
			LED_ON
			fRestart = false;	// リスタートフラグクリア
			t0 = millis();
			while (millis() - t0 < LONGPUSH_MS)	// 長押し判定ループ
			{
				// 短押しなら抜ける
				if (digitalRead(SW_PIN) == HIGH)
				{
					break;
				}
			}
			flongpush = millis() - t0 >= LONGPUSH_MS;	// 長押しだったか？

			rxbit = ReadCaliper();		// ノギスデータを読む
			Transmit(rxbit, flongpush);	// データを送信
			NotifyBattery(ReadVDD());	// バッテリー状態を通知
			noptime = 0;				// 操作なし時間をクリア
			LED_OFF

			// 長押し時SWが離されるのを待つ
			while (digitalRead(SW_PIN) == LOW);
		}
		nobletime = 0;	// BLE未接続時間クリア
	}
	else
	{
		nobletime += DEBOUNCE_MS;	// BLE未接続時間を加算
	}
	noptime += DEBOUNCE_MS;		// 操作なし時間を加算
	if (noptime >= SYSOFFTIME || nobletime >= SYSOFFTIME2)// 設定値を超えた？
	{
		#if USE_LPCOMP	// クロック検出にLPCOMPを使うなら
		end_lpcomp();	// LPCOMP 停止
		#endif //USE_LPCOMP

		// 念のためアドバタイズ停止
		Bluefruit.Advertising.stop();
		Bluefruit.Advertising.clearData();
		digitalWrite(LED_BLUE, HIGH);	// BLUE LED消灯
		delay(10);	// 終了するまで待つ

		GotoSystemOff();		// オートパワーオフ
	}
	delay(DEBOUNCE_MS);	// デバウンス除去のための待ち時間を兼ねてスリープ
}
/*----------------------------------------------------------------------------
	自前のmicros()
----------------------------------------------------------------------------*/
inline uint32_t micros_hw()
{
	return DWT->CYCCNT / (SystemCoreClock / 1000000); // μs 単位
}
/*----------------------------------------------------------------------------
	BLEセットアップ
----------------------------------------------------------------------------*/
void setup_ble()
{
	// BLEデバイスの設定
	Bluefruit.begin();
	Bluefruit.setTxPower(0);	// 省エネのため0dBmに
	Bluefruit.setName("CaliperData");

	// BLEデバイス・インフォメーション・サービスの設定
	bledis.setManufacturer("AT R&D");
	bledis.setModel("nRF52840");
	bledis.begin();
 
	// BLE HIDデバイス開始
	blehid.begin();

	// BLEバッテリーサービス開始
	blebas.begin();
 
	///// BLEのアドバタイズの設定 /////
	Bluefruit.Advertising.addFlags(
						BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
	Bluefruit.Advertising.addTxPower();
	Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_KEYBOARD);
 	// BLE HIDサービスをを登録
	Bluefruit.Advertising.addService(blehid);
 	// BLE バッテリーサービスをを登録
	Bluefruit.Advertising.addService(blebas);
 	// 開発者名を登録
	Bluefruit.Advertising.addName();
  	// 接続切断後に再アドバタイズ
	Bluefruit.Advertising.restartOnDisconnect(true);
	// 高頻度と低頻度のインターバル x0.625ms
	Bluefruit.Advertising.setInterval(32, 244);	// 20ms〜152.5ms
	// 高速から低速への移行時間[s]
	Bluefruit.Advertising.setFastTimeout(10);
	// アドバタイズ終了までの時間[s]
	Bluefruit.Advertising.start(0);	// 無制限
	// BLE接続時のコールバック関数
	Bluefruit.Periph.setConnectCallback(connectCallback);
}
/*----------------------------------------------------------------------------
	BLE接続時のコールバック
----------------------------------------------------------------------------*/
void connectCallback(uint16_t conn_handle)
{
	NotifyBattery(ReadVDD());		// バッテリー状態を通知
	Bluefruit.Advertising.stop();	// 念のためアドバタイズ停止
}
#if USE_LPCOMP	// クロック検出にLPCOMPを使うなら
/*----------------------------------------------------------------------------
	LPCOMP初期化
----------------------------------------------------------------------------*/
void setup_lpcomp()
{
	// ノギスクロックピン
	NRF_LPCOMP->PSEL = CLOCK_PIN;
	// vrefはVDD * 2/8とする
	NRF_LPCOMP->REFSEL = LPCOMP_REFSEL_REFSEL_Ref2_8Vdd;
	// 検出は両方
	NRF_LPCOMP->ANADETECT = LPCOMP_ANADETECT_ANADETECT_Cross;
	// イネイブル
	NRF_LPCOMP->ENABLE = 1;
	// スタート
	NRF_LPCOMP->TASKS_START = 1;
}
/*----------------------------------------------------------------------------
	LPCOMP終了
----------------------------------------------------------------------------*/
void end_lpcomp()
{
	NRF_LPCOMP->PSEL = 0;
	NRF_LPCOMP->ENABLE = 0;
	NRF_LPCOMP->TASKS_START = 0;
	NRF_LPCOMP->EVENTS_UP = NRF_LPCOMP->EVENTS_DOWN = 0;
}
/*----------------------------------------------------------------------------
	LPCOMPデータ取得
----------------------------------------------------------------------------*/
bool get_lpcomp(bool finit)
{
	static bool bit = true;

	if (finit)	// 初期化要求か？
	{
		// イベント結果クリア
		NRF_LPCOMP->EVENTS_UP = NRF_LPCOMP->EVENTS_DOWN = 0;
		bit = true;	// パルスが来ないときにHigh
	}
	yield();	// TASK側に一瞬制御を渡す(重要！)
	if (NRF_LPCOMP->EVENTS_UP)		// 立ち上がりがあった？
	{
		NRF_LPCOMP->EVENTS_UP = 0;	// イベント結果クリア
		bit = true;					// ラインはHigh
	}
	if (NRF_LPCOMP->EVENTS_DOWN)	// 立下りがあった？
	{
		NRF_LPCOMP->EVENTS_DOWN = 0;// イベント結果クリア
		bit = false;				// ラインはLow
	}
	return bit;
}
#endif //USE_LPCOMP
/*----------------------------------------------------------------------------
	ノギスのシリアルデータを読む
----------------------------------------------------------------------------*/
uint32_t ReadCaliper()
{
	uint32_t t0;
	uint32_t rxbit = 0;

	#if INVERT_LOGIC	// NPN Trによるレベル変換ならプルアップ
	pinMode(DATA_PIN, INPUT_PULLUP);
	pinMode(CLOCK_PIN, INPUT_PULLUP);
	#endif //INVERT_LOGIC

	#if USE_LPCOMP	// クロック検出にLPCOMPを使うなら
	get_lpcomp(true);	// LPCOMPイベント結果初期化
	#endif //USE_LPCOMP

	while (1)	// 読み取れるまでリトライ
	{
		// クロックの開始を待つ
		t0 = millis();
		while (CLOCK_SPACE);		// クロックがHの期間期間待つ
		if (millis() - t0 >= 10)	// Lになるまでに10ms以上かかった？
		{
			if (ReadCaliperCore(&rxbit))	// エラーなく読み取れたら
			{
				#if INVERT_LOGIC	// プルアップ戻す
				pinMode(DATA_PIN, INPUT);
				pinMode(CLOCK_PIN, INPUT);
				#endif //INVERT_LOGIC
				return rxbit;
			}
			else
			{
				Println(RxErr);
			}
		}
	}
}
/*----------------------------------------------------------------------------
	ノギスのシリアルデータを読むコア処理(エラー判定付き)
----------------------------------------------------------------------------*/
bool ReadCaliperCore(uint32_t* value)
{
	uint32_t t0;
	uint32_t rxbit = 0;
	uint32_t nowtime, lasttime, errtime;

	t0 = millis();
	for (int8_t i = 0; i < 24; i++)
	{
		lasttime = micros_hw();
		while (!CLOCK_SPACE)	// クロックの立ち上がりを待つ
		{
			nowtime = micros_hw();
			if (nowtime - lasttime > 250)	// 通常Lの時間は220μs
			{
				RxErr = 1;
				return false;
			}
		}
		lasttime = micros_hw();
		if (DATA_SPACE)			// クロック立ち上がり時のデータ
		{
			rxbit |= (1L << i);	// ビット取得
		}
		nowtime = micros_hw();
		// 通常のADC処理は24μs。クロックがHの期間で読み取れればOK
		if (nowtime - lasttime > 80)
		{
			RxErr = 2;
			return false;
		}
		if (i < 23)	// 最終ビットでなければ
		{
			errtime = (i % 4) == 3 ? 430 : 120;	// 通常Hの時間は393/88μs
			lasttime = micros_hw();
			while (CLOCK_SPACE)	// クロックの立下りを待つ
			{
				nowtime = micros_hw();
				if (nowtime - lasttime > errtime)
				{
					RxErr = 3;
					return false;
				}
			}
		}
	}
	if (millis() - t0 > 13)	// 通常24bitパルス列はトータル9ms
	{
		RxErr = 4;
		return false;
	}
	*value = (int32_t)(rxbit & 0xFFFFFF);
	return true;
}
/*----------------------------------------------------------------------------
	ノギスからのデータを送信
----------------------------------------------------------------------------*/
void Transmit(uint32_t rxbit, bool flongpush)
{
	float val;
	char txbuff[32];

	if (bitRead(rxbit, 23))	// インチモードなら
	{
		val = (float)((rxbit >> 1) & 0xFFFF) / 1000.0;
		if (rxbit & 0x01)	// 最下位ビットが立っていたら
		{
			val += 0.0005;
		}
		if (bitRead(rxbit, 20))	// 負の値か？
		{
			val *= -1.0;
		}
		if (rxbit & 0x01)	// 最下位ビットが立っていたら
		{
			sprintf(txbuff, "%.4f", val);
		}
		else
		{
			sprintf(txbuff, "%.3f", val);
		}
	}
	else	// mmモードなら
	{
		val = (float)(rxbit & 0xFFFF) / 100.0;
		if (bitRead(rxbit, 20))	// 負の値か？
		{
			val *= -1.0;
		}
		sprintf(txbuff, "%.2f", val);
	}
	if (flongpush)		// 長押しなら
	{
		strcat(txbuff, "\t");	// TAB
	}
	else
	{
		strcat(txbuff, "\n");	// CR
	}
	Print(txbuff);
	blehid.keySequence(txbuff, 10);	// BLEキーボードに送信
}
/*----------------------------------------------------------------------------
	電源電圧を読み取る[mV] ADCのデフォルトはvref=0.6V gain=1/6 10bit
----------------------------------------------------------------------------*/
uint16_t ReadVDD()
{
	return analogReadVDD() * 3600 / 1024;	// フルスケール3.6V
}
/*----------------------------------------------------------------------------
	バッテリー状態を通知
----------------------------------------------------------------------------*/
void NotifyBattery(uint16_t mv)
{
	uint16_t pct;

	pct = map(mv, 1700, 3000, 0, 100);  // 1.7V～3.0V→0～100%
	pct = constrain(pct, 0, 100);
	blebas.notify(pct);					// バッテリー状態を通知
}
/*----------------------------------------------------------------------------
	システムオフ
----------------------------------------------------------------------------*/
void GotoSystemOff()
{
	// SW ピンを wake-up 対応に設定
	pinMode(SW_PIN, INPUT_PULLUP_SENSE);
	delay(10);	// 念のため安定化を待つ
	// System OFF
	NRF_POWER->SYSTEMOFF = 1;
	__SEV();
	__WFE();
	__WFE();
}
/*** end of "CaliperBLE.ino" ***/
