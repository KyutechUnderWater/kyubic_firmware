# Sensor Board ESP32 Firmware

ESP32を使用したセンサボード用ファームウェアです。深度、環境、RTC、GNSS、漏水、RP2040状態を取得し、W5500経由でUDP通信を行います。

現在のファームウェアバージョンは **V31.0** です。

## V31.0変更レポート

電源再投入後にOLEDが点灯せず、プログラムが停止または再起動を繰り返す問題への対策として、起動処理、I²C、WDT、通信処理および入力検証を全面的に見直しました。

### 起動・再起動対策

- 起動直後からシリアルログを出力
- `esp_reset_reason()` による再起動原因の表示
- OLEDをEthernetや各センサより先に初期化
- 周辺機器の初期化段階をシリアルログへ出力
- mutex確保失敗を検出
- FreeRTOSタスク生成失敗を検出
- 致命的な初期化失敗時はESP32を再起動

### Watchdog対策

- Task Watchdogのタイムアウトを10秒に設定
- `Task_Main`、`Task_Depth`、`Task_NetRx`を監視
- 時間のかかるGNSS設定中にもWDTをリセット
- WDTの初期化・タスク登録エラーをログ出力

### I²C対策

- `Wire`、`Wire1`に50 msの通信タイムアウトを設定
- 深度センサのバス再初期化後にもタイムアウトを再設定
- I²Cバス復旧処理をオープンドレイン方式へ修正
- SDA/SCLをHIGHへ強制駆動する処理を廃止
- クロック9回とSTOP条件によるバス解放を実装

### MS5837深度センサ

- PROM校正データのCRC4検証を追加
- 不正な校正データを使用しないよう修正
- MS5837-30BAの二次温度補償を追加
- NaN、無限大、異常温度を検出
- 初期化失敗時は1秒間隔で再試行
- 連続通信失敗時はI²Cバスを自動復旧
- 海水・淡水設定を読み取り単位で確定し、データ競合を防止

### OLED・RTC・環境センサ

- RTCの初期化結果を保存
- RTC未接続時の不正な`rtc.now()`呼び出しを防止
- RTC読み取り値の有効性を検証
- RTCおよびGNSSの異常をOLEDに表示
- BME280のNaN、無限大、異常範囲を拒否
- 異常な環境値をUDP送信しないよう修正
- OLED描画データを排他保護されたスナップショットから取得
- GUI枠の右下描画ミスを修正

### GNSS

- PPS設定結果をログ出力
- 起動時のGNSS設定先をRAMのみに変更
- 毎起動時のFlash書き込みを廃止
- GNSS設定中にWDTを定期リセット
- 時刻同期時に時刻と日付の両方を検証
- RTCがない場合はRTC時刻の送信を停止

### Ethernet・UDP

- W5500の存在と全UDPソケットの生成結果を確認
- Ethernet再送タイムアウトを300 msから100 msへ変更
- Ethernet再送回数を0回から2回へ変更
- UDP受信中のEthernet排他時間を短縮
- UART送信とIMUリセット待機をEthernet排他区間の外へ移動
- Ethernet停止中の送信処理を抑制
- 繰り返しエラーログを最大1 Hzに制限

### UDP入力検証

制御UDPに以下の検証を追加しました。

- 送信元IPが`192.168.9.100`であること
- パケットサイズが受信バッファ以下であること
- UDPパケットを全長受信できたこと
- Nanopbのデコードが成功したこと
- 必要なoptionalフィールドが存在すること
- Servo値が`0～180`
- RGB値が`0x000000～0xFFFFFF`
- 電圧が`0～60 V`
- 電流の絶対値が`200 A以下`
- 浮動小数点値がNaNまたは無限大ではないこと

検証に失敗したパケットは破棄され、制御出力には反映されません。

> [!IMPORTANT]
> 正規の制御送信元が`192.168.9.100`以外の場合は、`main.cpp`の`pcIp`と許可条件を変更してください。送信元IPの検証は暗号学的な認証ではないため、信頼できないネットワークで使用する場合は通信プロトコル側に認証機構が必要です。

### RP2040 UART

- 受信フレームを`<status>`の3バイトとして検証
- 終端文字`>`がないフレームを破棄
- 破損データによる状態更新を防止
- 送信時の符号付き右シフトを符号なし処理へ変更

### バッテリー計算

- Logicバッテリーを4Sとして固定
- Actuatorバッテリーを6Sとして固定
- 電圧からセル数を推測する処理を廃止
- 通信間隔が5秒を超えた場合は積算を停止
- 消費電流は正方向のみ積算
- 積算値を`0～バッテリー容量`に制限
- 不正な電圧・電流をRP2040へ転送しないよう修正

### 漏水検出

- 漏水入力へ50 msのデバウンスを追加
- 瞬間的なノイズによる誤警報を抑制
- Ethernet停止中でも送信試行を1 Hzに制限

### 並行処理

- 複数コアから参照される電圧、電流、表示値、水種設定をクリティカルセクションで保護
- Ethernet、UART、I²C、深度共有値をmutexで保護
- FreeRTOSの待ち時間を`pdMS_TO_TICKS()`へ統一

## ネットワーク設定

| 項目 | アドレス |
|---|---|
| ESP32 | `192.168.9.5` |
| 制御PC | `192.168.9.100` |
| ブロードキャスト | `192.168.9.255` |

### 送信ポート

| データ | ポート |
|---|---:|
| Battery | 9000 |
| Depth | 9002 |
| Environment | 9003 |
| Leak | 9005 |
| GNSS | 9007 |
| RTC Time | 9008 |
| System Signal | 9012 |

### 受信ポート

| 制御 | ポート |
|---|---:|
| Buzzer | 9001 |
| Depth Config | 9002 |
| IMU Reset | 9004 |
| RGB | 9006 |
| Servo | 9009 |
| ZED Power | 9010 |
| Power State | 9011 |

## ビルド

プロジェクトディレクトリから以下を実行します。

```bash
pio run -d sensor_board_esp32
```

PlatformIOがPATHにない場合：

```bash
/home/ryoukun/.platformio/penv/bin/pio run -d sensor_board_esp32
```

### ビルド検証結果

- ビルド成功
- プロジェクト側コンパイル警告なし
- RAM使用量：23,088 bytes / 327,680 bytes（7.0%）
- Flash使用量：382,065 bytes / 1,310,720 bytes（29.1%）
- `firmware.bin`：382,432 bytes
- `git diff --check`：問題なし

警告設定は`platformio.ini`の`build_src_flags`で有効化しています。

## 書き込み

`platformio.ini`では以下のデバイスを使用します。

```ini
upload_port = /dev/sensor_esp32
monitor_port = /dev/sensor_esp32
monitor_speed = 115200
```

書き込み：

```bash
pio run -d sensor_board_esp32 -t upload
```

シリアルモニタ：

```bash
pio device monitor --port /dev/sensor_esp32 --baud 115200
```

## 起動ログ

正常起動時は、おおむね以下の順番でログが出力されます。

```text
[Boot] reset reason=...
[Boot] waiting for peripheral power...
[Boot] watchdog ready
[Boot] I2C buses ready
[Boot] OLED init...
[Boot] OLED OK
[Boot] Ethernet init...
[Boot] Ethernet=OK
[Boot] BME280=OK, RTC=OK
[Boot] GNSS init...
[Boot] GNSS PPS config=OK
[Boot] GNSS=OK
[Boot] tasks started
[Depth] sensor ready
```

### 主なリセット理由

| 値 | 内容 |
|---:|---|
| 1 | 電源投入 |
| 3 | ソフトウェアリセット |
| 4 | Panic |
| 5 | Interrupt WDT |
| 6 | Task WDT |
| 9 | Brownout |

`reset reason=6`が繰り返される場合はタスク停止、`reset reason=9`の場合は電源電圧低下や電源容量不足を確認してください。

## 実機確認項目

ビルド検証は完了していますが、以下は実機で確認する必要があります。

1. USB書き込み後の動作
2. 外部電源によるコールドブート
3. 電源再投入を複数回行った場合の起動
4. OLED、BME280、RTC、GNSSを個別に切断した場合の動作
5. MS5837切断時の自動復旧
6. Ethernetケーブル切断・再接続
7. 不正IPおよび不正サイズのUDPパケット拒否
8. 漏水入力のデバウンスと緊急表示
9. 10秒WDTによるフリーズ復旧
