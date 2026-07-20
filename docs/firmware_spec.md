# UzuCast V3 ファームウェア仕様書

Master（ESP32-S3）と Slave（ESP32）のプログラム構成・データフロー・スレッド構成をまとめた文書です。

| 項目 | Master | Slave |
|------|--------|-------|
| プロジェクト | `platformio/uzuCastV3_Main` | `platformio/uzuCastV3_Sub` |
| ボード | ESP32-S3 Dev Module | ESP32 Dev Module |
| ビルド番号 | `FIRMWARE_BUILD` **10046** | `BUILD_NUMBER` **96** |
| バージョン文字列 | `UZU CAST Version 1.00` | `UZU CAST Sub MP3 1.03 I2S-441-R-TAG` |
| Arduino 実行コア | Core 1 | Core 1 |

---

## 1. システム全体像

```mermaid
flowchart LR
  subgraph Master["Master (ESP32-S3)"]
    SD[(SDカード)]
    WiFi[WiFi SoftAP + WebSocket]
    CLI_M[シリアル CLI]
    TDM_TX[TDM I2S TX 8ch]
    I2C_H[I2C Host]
    SD --> Player
    WiFi --> Stream
    CLI_M --> Ctrl_M[制御・テスト]
    Player --> TDM_TX
    Stream --> TDM_TX
    Ctrl_M --> TDM_TX
    Ctrl_M --> I2C_H
  end

  subgraph Slave["Slave (ESP32)"]
    I2S_RX[I2S RX ステレオ]
    Tag[TDMタグ解析]
    MP3[minimp3 デコード]
    PCM[PCM リングバッファ]
    BT[A2DP Source]
    I2C_S[I2C Slave]
    I2S_RX --> Tag
    Tag -->|RAW| PCM
    Tag -->|MP3| MP3 --> PCM
    PCM --> BT
    I2C_S -.-> BT
  end

  TDM_TX -->|BCLK/WS/DOUT| I2S_RX
  I2C_H <-->|SDA/SCL| I2C_S
  BT --> Speaker[Bluetooth スピーカー]
```

### 1.1 音声データの2系統

| 系統 | Master 出力形式 | Slave 受信・処理 | 用途 |
|------|----------------|-----------------|------|
| **RAW** | TDM CH1–7 = PCM、CH8 = `0xAAAA` | I2S L=PCM、R=`0xAAAA` として解釈しそのまま BT へ | `.UZU` 再生、WiFi ストリーム、SINTEST RAW |
| **MP3** | TDM CH1 = スロットワード、CH8 = `AA00`→`AA55`×N→`5500`→`0000` | スロットを組み立てて minimp3 デコード後 BT へ | `.uzc` 再生、SINTEST MP3 |

### 1.2 物理接続

| 信号 | Master GPIO | Slave GPIO | 備考 |
|------|-------------|------------|------|
| I2S BCLK | 5 | 32 | Master=TDM TX、Slave=RX Slave |
| I2S WS | 16 | 33 | |
| I2S データ | 4 (DOUT) | 35 (DIN) | 8ch TDM → Slave はステレオ I2S で受信 |
| I2C SDA | 41 | 21 | `UZU_ENABLE_I2C=1` 時のみ |
| I2C SCL | 40 | 22 | |

> **補足:** 現行ビルドでは Master/Slave ともに `UZU_ENABLE_I2C=0` がデフォルト。Slave はローカルボタンで BT 接続する構成。

---

## 2. Master ファームウェア

ソース: `platformio/uzuCastV3_Main/src/main.ino`（モノリシック、約 4800 行）

関連モジュール:

| ファイル | 役割 |
|----------|------|
| `main.ino` | 電源管理、SD、TDM、プレイヤー、ストリーム、Web UI、CLI |
| `uzc_tdm.cpp` | `.uzc` MP3 コンテナ読み出し・タグ付き TDM 送信 |
| `uzc_mp3_slot.h` | SINTEST MP3 用 441Hz 埋め込みスロット |
| `i2c_host.cpp` | I2C マスター（Slave モード同期・BT 仲裁） |
| `i2c_mode.cpp` | `TdmDataMode`（RAW / MP3）状態管理 |
| `i2c_protocol.h` | I2C アドレス・コマンド定義 |

### 2.1 起動シーケンス

```mermaid
flowchart TD
  A[setup] --> B[Serial 115200 開始]
  B --> C[PowerManager::begin<br/>電源ラッチ・ボタン待ち]
  C --> D[performSystemInit]
  D --> E[SD_MMC マウント]
  E --> F[TDM I2S TX 初期化<br/>44100Hz 8ch 16bit]
  F --> G[i2cModeBegin RAW]
  G --> H{UZU_ENABLE_I2C?}
  H -->|Yes| I[i2cHostBegin]
  H -->|No| J[scanUzuFiles]
  I --> J
  J --> K[WiFi SoftAP 192.168.4.1]
  K --> L[DNS + WebSocket :80]
  L --> M[ensureStreamPlaybackTask]
  M --> N[ensurePlayerPlaybackTask]
  N --> O[CommandParser 開始<br/>プロンプト >]
```

**電源管理 (`PowerManager`)**

- `POWER_PIN` で電源ラッチ保持
- 起動時 3 秒長押し待ち（`UZU_SKIP_POWER_BUTTON=1` でスキップ可）
- 動作中 3 秒長押しで `POWER_OFF` → `loop()` 内で無限待機

### 2.2 メインループ (`loop()`)

Arduino `loop` タスク（Core 1）で以下をポーリング実行します。音声の実出力は別タスクが担当します。

```mermaid
flowchart TD
  L[loop 開始] --> P[g_power.process]
  P --> PO{電源 OFF?}
  PO -->|Yes| HALT[stopAllPlayback → 無限待機]
  PO -->|No| CLI[g_parser.process シリアルCLI]
  CLI --> IM[i2cModeProcess]
  IM --> IH[i2cHostProcess]
  IH --> IT[i2sTestProcess]
  IT --> ST[sintestProcess]
  ST --> SD[processSdCardDetect]
  SD --> NET[dns + ws.loop + ws通知]
  NET --> UZC{g_uzcPlaying かつ<br/>UZC タスク終了?}
  UZC -->|Yes| DIS[tdm_disable_output]
  UZC -->|No| L
  DIS --> L
```

### 2.3 FreeRTOS タスク（Master）

| タスク名 | エントリ関数 | スタック | コア | 優先度 | 起動条件 | 処理内容 |
|----------|-------------|--------|------|--------|----------|----------|
| **Arduino loop** | `loop()` | (framework) | 1 | default | 常時 | CLI、I2C、I2STEST、SINTEST、WebSocket、SD 検出 |
| **streamAudio** | `streamPlaybackTask` | 4096 | 1 | 5 | `ensureStreamPlaybackTask()` | `g_deviceMode==STREAM` 時に `g_streamEngine.process()` |
| **playerAudio** | `playerPlaybackTask` | 4096 | 1 | 5 | `ensurePlayerPlaybackTask()` | SD の `.UZU` PCM 再生（`g_player.process()`） |
| **uzcTdm** | `UzcTdmPlayer::playbackTask` | 16384 | 1 | 5 | `.uzc` 再生開始時 | `.uzc` 読み出し・MP3 タグ TDM 送信、終了後自己削除 |

**playerAudio の動作ゲート**

以下のいずれかが真のときは `g_player.process()` を呼ばず 5ms スリープ:

- `g_deviceMode == STREAM`
- `g_uzcPlaying`
- `g_i2sTestActive`
- `g_sintestActive`

### 2.4 デバイスモード

#### 2.4.1 `DeviceMode`（グローバル再生モード）

```mermaid
stateDiagram-v2
  [*] --> SD
  SD --> STREAM: WebSocket set_mode STREAM
  STREAM --> SD: stop / 最終クライアント切断
```

| モード | 定数 | 音声ソース | TDM 出力 |
|--------|------|-----------|----------|
| SD 再生 | `DeviceMode::SD` | SD 上の `.UZU` / `.uzc` | 各プレイヤー経由 |
| WiFi ストリーム | `DeviceMode::STREAM` | ブラウザ → WebSocket バイナリ | `StreamEngine` 経由 |

#### 2.4.2 SD 再生ルーティング

```mermaid
flowchart TD
  PLAY[PLAY n / WS play] --> STOP[stopAllPlayback]
  STOP --> FMT{ファイル形式}
  FMT -->|UZUCPW1 .UZU| PCM[UzuTdmPlayer<br/>TdmDataMode RAW]
  FMT -->|.uzc MP3| UZC[UzcTdmPlayer タスク<br/>TdmDataMode MP3]
  PCM --> TDM[TDM I2S 出力]
  UZC --> TDM
```

- 出力サンプルレートは常に **44100 Hz**（48 kHz ソースは Master 側でリサンプル）
- `TDM_NUM_CH=8`、`FRAMES_PER_WRITE=256` フレーム単位で `i2s_channel_write`

#### 2.4.3 WiFi ストリーム

```mermaid
sequenceDiagram
  participant B as ブラウザ
  participant WS as WebSocket
  participant SE as StreamEngine
  participant RB as PcmRingBuffer
  participant TDM as TDM I2S

  B->>WS: set_mode STREAM
  B->>WS: file_info (rate/ch/bits)
  B->>WS: prepare
  Note over SE: streamBeginSession<br/>RAW + I2C clear
  B->>WS: バイナリ PCM 256ch×2B
  WS->>RB: pushPcm()
  B->>WS: start
  Note over SE: BUFFERING → used≥cap/3
  SE->>TDM: processOneAudioBlock<br/>CH8=0xAAAA
  SE-->>B: buffer_info / underrun 通知
```

**`StreamState` 状態遷移**

```mermaid
stateDiagram-v2
  [*] --> IDLE
  IDLE --> PREPARED: prepare()
  PREPARED --> BUFFERING: pushPcm()
  BUFFERING --> PLAYING: start() かつ used≥capacity/3
  PLAYING --> PAUSED: pause()
  PAUSED --> PLAYING: resume()
  PLAYING --> IDLE: stop()
  BUFFERING --> IDLE: stop()
```

**リングバッファ (`PcmRingBuffer`)**

| 定数 | サイズ |
|------|--------|
| `PSRAM_TARGET_BYTES` | 1,048,576 バイト |
| `PSRAM_FALLBACK_BYTES` | 393,216 バイト |
| `INTERNAL_FALLBACK_BYTES` | 49,152 バイト |

#### 2.4.4 テストモード（SD/STREAM とは独立フラグ）

| モード | 起動 | 実行コンテキスト | 内容 |
|--------|------|-----------------|------|
| **I2STEST** | `I2STEST [sec]` | `loop()` | CH1 方形波 440Hz + CH8=`0xAAAA`、デフォルト 5 秒 |
| **SINTEST** | `TESTMODE` + `SINTEST` | `loop()` | Slave 検証用。RAW=440Hz サイン / MP3=441Hz スロット |

**SINTEST コマンド**

```
TESTMODE RAW|MP3
SINTEST 1|2|3|4|5|ALL|OFF
```

- RAW: 有効 CH にサイン波。CH1 ON 時は CH2・CH8 に `0xAAAA`
- MP3: `uzc_mp3_slot.h` の埋め込みスロットを TDM タグ列で送出
- I2C へも `RAW` / `MP3` をブロードキャスト

### 2.5 TDM I2S 出力仕様

| 定数 | 値 |
|------|-----|
| `TDM_NUM_CH` | 8 |
| CH1–CH7 (index 0–6) | オーディオデータ |
| CH8 (index 7) | 制御タグレーン |
| `I2S_TAG_RAW_TDM` | `0xAAAA` |
| `I2S_TAG_MP3_START` | `0xAA00` |
| `I2S_TAG_SLOT_DATA` | `0xAA55` |
| `I2S_TAG_MP3_END` | `0x5500` |
| `UZC_MP3_PERIOD_STEREO` | 1152 フレーム / 期間 |

**出力経路まとめ**

```mermaid
flowchart LR
  UZU[.UZU PCM] --> UP[UzuTdmPlayer]
  UZC_F[.uzc MP3] --> UT[UzcTdmPlayer タスク]
  STR[WebSocket PCM] --> SE[StreamEngine]
  IST[I2STEST] --> LOOP1[loop]
  SIN[SINTEST] --> LOOP2[loop]
  UP --> BUF[g_tdmTxBuf]
  UT --> BUF
  SE --> BUF
  LOOP1 --> BUF
  LOOP2 --> BUF
  BUF --> I2S[i2s_channel_write]
```

### 2.6 I2C Host（Master → Slave 制御）

`UZU_ENABLE_I2C=1` 時に有効。200ms 周期で `i2cHostProcess()` を実行。

| コマンド | ID | 用途 |
|----------|-----|------|
| `GET_STATUS` | 0x01 | Slave 状態取得 |
| `SET_MODE` | 0x02 | WAV_PCM(0x00) / MP3(0x01) |
| `GET_CONNECTED_TARGET` | 0x03 | 接続中 BT MAC |
| `GET_PENDING_TARGET` | 0x04 | 接続待ち BT MAC |
| `NOTIFY_CONNECT_PERMISSION` | 0x05 | BT 接続許可/拒否 |
| `CLEAR_BUFFER` | 0x07 | バッファクリア |
| `SET_DELAY` | 0x09 | 遅延 ms 設定 |

アドレス: `0x30` + チャンネル番号（CH1–CH5）

**BT 仲裁 (`processBtCoordination`)**

複数 Slave が同一 BT デバイスへ同時接続しようとした場合、Master が先着 MAC を許可し他を拒否します。

### 2.7 シリアル CLI（Master）

115200 8N1、プロンプト `>`。コマンドは大文字化して解析。

| カテゴリ | コマンド |
|----------|----------|
| システム | `HELP`, `RESET`, `STAT`, `CHECKDISK`, `MOUNT`, `FREQ` |
| ファイル | `DIR`, `FSTAT` |
| 再生 | `PLAY`, `PAUSE`, `STOP`, `SEEK`, `VOL` |
| モード | `MODE RAW\|MP3` |
| Slave テスト | `TESTMODE RAW\|MP3`, `SINTEST` |
| ベンチ | `I2STEST`, `I2STEST STOP`, `I2STEST DUMP` |
| I2C | `I2CSCAN`（I2C 有効時） |

### 2.8 `stopAllPlayback()` — 相互排他の中心

以下を一括停止します。モード切替・新規再生の前に必ず呼ばれます。

1. `i2sTestStop()`
2. `sintestStop()`
3. `g_uzcPlayer.stop()`
4. `g_player.stop()`
5. `g_streamEngine.stop()`

---

## 3. Slave ファームウェア

ソース: `platformio/uzuCastV3_Sub/src/main.ino`（モノリシック、約 3400 行）

関連モジュール:

| ファイル | 役割 |
|----------|------|
| `main.ino` | I2S RX、タグ解析、MP3 デコード、PCM バッファ、BT FSM |
| `minimp3.h` | MP3 デコード（minimp3） |
| `i2c_slave.cpp` | I2C スレーブ（オプション） |
| `i2c_protocol.h` | プロトコル定数 |

### 3.1 起動シーケンス

```mermaid
flowchart TD
  A[setup] --> B[低電力プリセット<br/>WiFi OFF 等]
  B --> C[Serial 115200]
  C --> D[GPIO: LED×2, ボタン]
  D --> E{i2cSlaveBegin?}
  E -->|I2C有効| F[I2C Slave 初期化]
  E -->|I2C無効| G[i2s_rx_init_slave_stereo_16]
  F --> G
  G --> H[resetAudioPipeline<br/>resetUzcSlotAssembler]
  H --> I[NVS から前回 BT MAC 読込]
  I --> J[a2dp コールバック登録]
  J --> K[a2dp.start]
  K --> L[enterIdle → ST_IDLE]
```

### 3.2 メインループ (`loop()`)

Slave は **専用 FreeRTOS タスクを作成せず**、Arduino `loop` タスク（Core 1）で全処理を協調的に実行します。末尾で `delay(1)`。

```mermaid
flowchart TD
  L[loop] --> I2S[serviceI2SInput<br/>I2S 読み出し・タグ解析]
  I2S --> BENCH{UZU_I2S_TEST_BYPASS_BT?}
  BENCH -->|Yes| CLI[pollSerialTestCommands]
  BENCH -->|Yes| DRAIN[i2sBenchDrainPcm]
  BENCH -->|No| IC
  CLI --> IC[i2cSlavePoll]
  DRAIN --> IC
  IC --> HB[PCM 5秒ハートビート]
  HB --> BTN[pollButton]
  BTN --> LED[updateLed]
  LED --> TO[BT タイムアウト処理]
  TO --> GAP[GAP ディスカバリ進行]
  GAP --> MS[遅延 Media Start 実行<br/>esp_a2d_media_ctrl]
  MS --> D1[delay 1ms]
  D1 --> L
```

> **設計上の重要点:** `esp_a2d_media_ctrl(START)` は I2S/BT コールバック内では呼ばず、必ず `loop()` から実行します。

### 3.3 スレッド / 実行コンテキスト（Slave）

| コンテキスト | コア | 処理 |
|-------------|------|------|
| **Arduino loop** | 1 | I2S 入力、MP3 デコード（同期）、BT 状態機械、ボタン/LED |
| **ESP-IDF BT スタック** | 内部（通常 0） | GAP、A2DP 接続、SBC エンコード |
| **audio_cb** | BT/オーディオタスク | `g_pcm_hold` から A2DP フレーム供給 |

`uzcDecodeTask` / `g_uzc_decode_task` はスキャフォールドのみで、**実際の MP3 デコードは loop 内で同期実行**されます。

### 3.4 I2S 受信と TDM タグ解析

Slave は I2S **ステレオ** Slave RX（`I2S_NUM_1`）で Master の TDM を受けます。

- **L チャンネル** = データ（PCM サンプルまたは MP3 スロットワード）
- **R チャンネル** = タグ

```mermaid
stateDiagram-v2
  [*] --> WaitTag
  WaitTag --> RawPcm: R=0xAAAA
  RawPcm --> WaitTag: L を PCM バッファへ

  WaitTag --> Mp3Open: R=0xAA00
  Mp3Open --> Mp3Data: R=0xAA55
  Mp3Data --> Mp3Data: L をスロットバッファへ
  Mp3Data --> Mp3Decode: R=0x5500
  Mp3Decode --> PadSkip: minimp3 デコード → PCM バッファ
  PadSkip --> WaitTag: 1152 フレーム期間の残りをスキップ
```

| タグ (R) | 値 | 動作 |
|----------|-----|------|
| `I2S_TAG_RAW` | `0xAAAA` | 44100Hz に設定、L をステレオ PCM として `pcmHoldAppend` |
| `I2S_TAG_MP3_START` | `0xAA00` | スロット組み立て開始 |
| `I2S_TAG_SLOT_DATA` | `0xAA55` | L のバイトをスロットバッファへ追加 |
| `I2S_TAG_MP3_END` | `0x5500` | スロット完成 → `decodeMp3SlotWorker` |
| `I2S_TAG_PAD` | `0xFFFF` | 無視 |

**I2S 処理ゲート (`uzcShouldProcessI2s`)**

| ビルド | 条件 |
|--------|------|
| `UZU_I2S_TEST_BYPASS_BT=1`（現行デフォルト） | 常に I2S 処理（BT 未接続でも可） |
| 通常ビルド | `ST_CONNECT` かつ `g_audio_enable` のときのみ |

### 3.5 MP3 デコードパス

```mermaid
flowchart TD
  A[i2sTagDispatchMp3Slot] --> B[i2sTagPrepareDecodeSlot<br/>ヘッダ整合]
  B --> C[decodeMp3SlotWorker]
  C --> D[mp3dec_decode_frame minimp3]
  D --> E[モノ→ステレオ複製]
  E --> F[pcmHoldAppend<br/>最大 1152 フレーム]
```

- スロット最大: `UZC_SLOT_BUF_MAX = 1024` バイト
- 1 期間: `UZC_MP3_PERIOD_STEREO_441 = 1152` ステレオフレーム

### 3.6 PCM バッファリング

| 定数 | フレーム数 | 時間 @ 44.1kHz | 役割 |
|------|-----------|----------------|------|
| `PCM_HOLD_FRAMES` | 5888 | 約 133 ms | リングバッファ総容量 |
| `PCM_PREBUFFER_FRAMES` | 3584 | 約 81 ms | BT Media Start 前の蓄積目標 |
| `PCM_PLAYBACK_MIN_HOLD` | 3072 | 約 70 ms | `audio_cb` 排出開始閾値 |
| `PCM_HOLD_PREPLAY_MAX` | 3968 | 約 90 ms | 再生前の上限（溢れ時ドロップ） |

```mermaid
flowchart LR
  I2S_IN[I2S タグ処理] --> APPEND[pcmHoldAppend]
  APPEND --> HOLD[(g_pcm_hold リング)]
  HOLD -->|used ≥ PREBUFFER| MS[scheduleMediaStart]
  MS --> LOOP[loop: esp_a2d_media_ctrl START]
  LOOP --> CB[audio_cb 排出]
  CB --> BT_OUT[Bluetooth A2DP]
  HOLD -->|満杯| DROP[push_drop_frames++]
  CB -->|不足| UND[underrun_frames++ 無音埋め]
```

**適応 I2S バースト (`streamI2sBurstLimit`)**

PCM バッファ水位に応じて 1 回の `loop` で読む I2S チャンク数を 2–64 に制限し、過剰流入を防ぎます。

### 3.7 Bluetooth A2DP 出力

ESP32 は **A2DP Source**（送信側）として動作します。

```mermaid
sequenceDiagram
  participant BTN as ボタン/GAP
  participant FSM as BtState FSM
  participant I2S as I2S 入力
  participant PCM as PCM バッファ
  participant A2DP as audio_cb

  BTN->>FSM: 接続開始
  FSM->>FSM: ST_CONNECTING → ST_CONNECT
  Note over FSM: g_audio_enable=true<br/>resetAudioPipeline
  I2S->>PCM: タグ PCM 蓄積
  PCM->>FSM: used ≥ PREBUFFER
  FSM->>FSM: scheduleMediaStart (loop 待ち)
  Note over FSM: esp_a2d_media_ctrl START
  FSM->>A2DP: AUDIO_STATE_STARTED
  A2DP->>A2DP: beginPcmPlaybackToBt
  loop I2S→PCM→A2DP: 継続再生
```

**`BtState` 状態一覧**

```mermaid
stateDiagram-v2
  [*] --> ST_IDLE
  ST_IDLE --> ST_PAIRING: 短押し(未登録)
  ST_IDLE --> ST_CONNECTING: 短押し(登録済)
  ST_PAIRING --> ST_CONNECT: GAP→接続成功
  ST_CONNECTING --> ST_CONNECT: 接続成功
  ST_CONNECTING --> ST_RECONNECT_WAIT: 失敗
  ST_RECONNECT_WAIT --> ST_CONNECTING: リトライ
  ST_IDLE --> ST_ERASE: 8秒長押し
  ST_PAIRING --> ST_IDLE: 3秒長押し
  ST_CONNECT --> ST_IDLE: 切断/3秒長押し
  ST_ERROR --> ST_TIMEOUT: 3秒表示
  ST_TIMEOUT --> ST_IDLE
```

### 3.8 I2C Slave（オプション）

`UZU_ENABLE_I2C=1` 時:

- アドレス `0x30 + SEL[2:0]`（GPIO 17/18/19）
- Master から `SET_MODE` で WAV_PCM / MP3 モード同期
- BT 接続は GAP 発見後 `queueBtConnectPending` → Master の `NOTIFY_CONNECT_PERMISSION` 待ち

現行デフォルト (`UZU_ENABLE_I2C=0`): ボタン操作で直接 `a2dp.connect_to()`。

### 3.9 シリアル（Slave）

| 種別 | 内容 |
|------|------|
| 常時 | 起動バナー、`[PCM]` 5 秒ハートビート |
| ベンチモードのみ | `PIPE RST`, `BT GO`（`UZU_I2S_TEST_BYPASS_BT=1`） |

---

## 4. Master ↔ Slave 連携シーケンス

### 4.1 通常再生（SD → Slave → BT）

```mermaid
sequenceDiagram
  participant User as ユーザー
  participant M as Master
  participant S as Slave
  participant SPK as BT スピーカー

  User->>M: PLAY 1 (.UZU)
  M->>M: i2cModeSet RAW + I2C broadcast
  M->>M: UzuTdmPlayer → TDM CH1-7 PCM, CH8=AAAA
  M-->>S: TDM/I2S ストリーム
  S->>S: 0xAAAA 検出 → pcmHoldAppend
  S->>S: PREBUFFER 到達 → Media Start
  S->>SPK: A2DP PCM
```

### 4.2 SINTEST（Slave 経路検証）

```mermaid
sequenceDiagram
  participant User as ユーザー
  participant M as Master
  participant S as Slave

  User->>M: TESTMODE RAW
  User->>M: SINTEST 1
  M->>M: 440Hz サイン CH1 + CH2/CH8=AAAA
  M->>M: I2C SET_MODE WAV_PCM
  M-->>S: I2S タグストリーム
  S->>S: RAW パスで PCM 生成（BT ベンチ時は drain も可）

  User->>M: TESTMODE MP3
  User->>M: SINTEST 1
  M->>M: 441Hz MP3 スロット + タグ列
  M->>M: I2C SET_MODE MP3
  M-->>S: MP3 タグストリーム
  S->>S: スロット組立 → minimp3 → PCM
```

### 4.3 WiFi ストリーム（Master のみ、Slave 不使用）

ブラウザが Master AP (`192.168.4.1`) に接続し WebSocket で PCM を送信。Slave は関与しません（TDM 出力先は配線次第）。

---

## 5. スレッド構成一覧（両端末）

```mermaid
flowchart TB
  subgraph MasterTasks["Master (ESP32-S3)"]
    ML[loop タスク Core1]
    SA[streamAudio Core1 pri5]
    PA[playerAudio Core1 pri5]
    UZ[uzcTdm Core1 pri5 動的]
    ML --- SA
    ML --- PA
    ML --- UZ
  end

  subgraph SlaveTasks["Slave (ESP32)"]
    SL[loop タスク Core1]
    BTC[BT スタック内部タスク]
    AC[audio_cb コールバック]
    SL --- BTC
    BTC --- AC
  end
```

| 端末 | タスク / コンテキスト | 数 | 音声関連 |
|------|----------------------|-----|----------|
| Master | `loop` | 1 | I2STEST / SINTEST 出力 |
| Master | `streamAudio` | 1 | ストリーム TDM 出力 |
| Master | `playerAudio` | 1 | SD PCM TDM 出力 |
| Master | `uzcTdm` | 0–1（再生時のみ） | SD MP3 TDM 出力 |
| Slave | `loop` | 1 | I2S 入力・MP3 デコード・BT FSM |
| Slave | BT スタック + `audio_cb` | (IDF 内部) | A2DP 排出 |

---

## 6. ビルドフラグ（主要）

### Master (`uzuCastV3_Main/platformio.ini`)

| フラグ | デフォルト | 意味 |
|--------|-----------|------|
| `UZU_ENABLE_I2C` | 0 | I2C Host 機能 |
| `UZU_SKIP_POWER_BUTTON` | 1（開発時） | 起動時長押しスキップ |

### Slave (`uzuCastV3_Sub/platformio.ini`)

| フラグ | デフォルト | 意味 |
|--------|-----------|------|
| `UZU_ENABLE_I2C` | 0 | I2C Slave・Master 経由 BT 制御 |
| `UZU_I2S_TEST_BYPASS_BT` | 1 | BT 未接続でも I2S 処理継続 |

---

## 7. 改訂履歴

| 日付 | 内容 |
|------|------|
| 2026-07-16 | 初版作成（Master BUILD 10046 / Slave BUILD 96 時点） |
