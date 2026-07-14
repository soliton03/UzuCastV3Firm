# UzuCast Master-Slave I2C 通信仕様

## 1. 概要

本仕様は、UzuCast における Main（Master ESP32-S3）と Slave ESP32 間の制御通信を定義する。

音声データは I2S/TDM 系統で転送し、本仕様の I2C 通信は、Slave の状態確認、動作モード設定、Bluetooth 接続制御、バッファ制御、遅延調整などの制御用途に使用する。

## 2. 基本方針

- 通信方式は I2C とする。
- Main を I2C Master とする。
- Slave を I2C Slave とする。
- 通信は Master 主導のポーリング方式とする。
- Slave からの自発送信は行わない。
- I2C ブロードキャストは使用しない。
- 各 Slave は個別の I2C アドレスを持つ。
- コマンドはパケット形式ではなく、コマンド番号と必要な引数、および応答で構成する。

## 3. ピンアサイン

### 3.1 Main / Master ESP32-S3

| 信号 | GPIO |
|---|---:|
| SCL | 40 |
| SDA | 41 |

### 3.2 Slave ESP32

| 信号 | GPIO |
|---|---:|
| SCL | 22 |
| SDA | 21 |
| SEL0 | 17 |
| SEL1 | 18 |
| SEL2 | 19 |

## 4. Slave ID と I2C アドレス

Slave 側では、SEL0〜SEL2 を内部プルアップ付き入力として初期化する。
起動時に SEL[2:0] を読み取り、その値を Slave ID として使用する。

SEL は負論理入力として扱う想定とし、GND に接続されたビットを 0、未接続またはプルアップされたビットを 1 として読む。
ただし、実装時には基板仕様に合わせて論理を確認すること。

### 4.1 Slave ID 割り当て

| チャンネル | Slave ID | I2C アドレス |
|---|---:|---:|
| CH1 | 0 | 0x30 |
| CH2 | 1 | 0x31 |
| CH3 | 2 | 0x32 |
| CH4 | 3 | 0x33 |
| CH5 | 4 | 0x34 |
| 予備 | 5 | 0x35 |

I2C アドレスは以下で決定する。

```text
I2C Address = 0x30 + Slave ID
```

## 5. 通信形式

通信は以下の形式とする。

### 5.1 書き込みコマンド

```text
Master → Slave:
[CMD] [PARAM0] [PARAM1] ...
```

### 5.2 読み取りコマンド

```text
Master → Slave:
[CMD]

Master ← Slave:
[RESPONSE0] [RESPONSE1] ...
```

### 5.3 ACK 応答

書き込み系コマンドの応答は、原則として 1 バイトの ACK コードとする。

| 値 | 名称 | 意味 |
|---:|---|---|
| 0x00 | OK | 正常完了 |
| 0x01 | UNKNOWN_COMMAND | 未定義コマンド |
| 0x02 | INVALID_PARAM | パラメータ不正 |
| 0x03 | BUSY | 処理中 |
| 0x04 | DENIED | 実行不可 |
| 0x05 | ERROR | その他エラー |

## 6. MAC アドレス表現

Bluetooth ターゲットの識別には、Bluetooth MAC アドレスを 6 バイトのバイナリで使用する。

例：

```text
AA:BB:CC:DD:EE:FF
```

は以下の 6 バイトとして扱う。

```text
[0xAA] [0xBB] [0xCC] [0xDD] [0xEE] [0xFF]
```

無効な MAC アドレス、または対象なしの場合は以下を使用する。

```text
00:00:00:00:00:00
```

## 7. コマンド一覧

| CMD | 名称 | 内容 |
|---:|---|---|
| 0x01 | GET_STATUS | ステータス要求 |
| 0x02 | SET_MODE | 動作モード設定 |
| 0x03 | GET_CONNECTED_TARGET | 接続中ターゲット確認 |
| 0x04 | GET_PENDING_TARGET | 接続保留デバイス確認 |
| 0x05 | NOTIFY_CONNECT_PERMISSION | 接続可否通知 |
| 0x06 | SOFTWARE_RESET | ソフトウェアリセット要求 |
| 0x07 | CLEAR_BUFFER | バッファクリア要求 |
| 0x09 | SET_DELAY | 遅延設定 |

## 8. コマンド詳細

## 8.1 GET_STATUS

Slave の現在状態を取得する。

```text
CMD: 0x01
```

### Master → Slave

```text
[0x01]
```

### Slave → Master

```text
[status]
```

### status bit 定義

| bit | 名称 | 意味 |
|---:|---|---|
| 0 | READY | Slave 初期化完了 |
| 1 | PLAYING | 再生中 |
| 2 | BT_CONNECTED | Bluetooth 接続中 |
| 3 | BT_CONNECT_PENDING | Bluetooth 接続保留中 |
| 4 | BUFFER_ACTIVE | バッファ使用中 |
| 5 | ERROR | エラーあり |
| 6 | MODE_MP3 | MP3 モード中 |
| 7 | RESERVED | 予約 |

## 8.2 SET_MODE

Slave の動作モードを設定する。

```text
CMD: 0x02
```

### Master → Slave

```text
[0x02] [mode]
```

### mode

| 値 | 内容 |
|---:|---|
| 0x00 | WAV / PCM モード |
| 0x01 | MP3 モード |

### Slave → Master

```text
[ACK]
```

## 8.3 GET_CONNECTED_TARGET

Slave が現在接続中の Bluetooth ターゲットを取得する。

```text
CMD: 0x03
```

### Master → Slave

```text
[0x03]
```

### Slave → Master

```text
[connected_flag] [mac0] [mac1] [mac2] [mac3] [mac4] [mac5]
```

### connected_flag

| 値 | 内容 |
|---:|---|
| 0x00 | 未接続 |
| 0x01 | 接続中 |

未接続の場合、MAC アドレスは `00:00:00:00:00:00` とする。

## 8.4 GET_PENDING_TARGET

Slave が接続を保留している Bluetooth ターゲットを取得する。

Slave は接続予定の Bluetooth ターゲットを発見した場合、ただちに接続せず、接続を保留する。
その後、Master から本コマンドで確認されるまで待機する。

```text
CMD: 0x04
```

### Master → Slave

```text
[0x04]
```

### Slave → Master

```text
[pending_flag] [mac0] [mac1] [mac2] [mac3] [mac4] [mac5]
```

### pending_flag

| 値 | 内容 |
|---:|---|
| 0x00 | 保留なし |
| 0x01 | 接続許可待ち |

保留なしの場合、MAC アドレスは `00:00:00:00:00:00` とする。

## 8.5 NOTIFY_CONNECT_PERMISSION

Master が Slave に対して、保留中 Bluetooth ターゲットへの接続可否を通知する。

```text
CMD: 0x05
```

### Master → Slave

```text
[0x05] [result] [mac0] [mac1] [mac2] [mac3] [mac4] [mac5]
```

### result

| 値 | 内容 |
|---:|---|
| 0x00 | 接続不可 |
| 0x01 | 接続可 |

### Slave → Master

```text
[ACK]
```

### 動作

- 接続可の場合、Slave は指定された MAC アドレスの Bluetooth ターゲットへ接続を開始する。
- 接続不可の場合、Slave は指定された MAC アドレスの Bluetooth ターゲットを無視し、他のデバイスを探索する。
- 通知された MAC アドレスが現在保留中の MAC アドレスと一致しない場合、Slave は `INVALID_PARAM` または `DENIED` を返す。

## 8.6 SOFTWARE_RESET

Slave にソフトウェアリセットを要求する。

```text
CMD: 0x06
```

### Master → Slave

```text
[0x06] [reset_code]
```

### reset_code

| 値 | 内容 |
|---:|---|
| 0xA5 | リセット実行 |

誤動作防止のため、`reset_code` が `0xA5` の場合のみリセットを実行する。

### Slave → Master

```text
[ACK]
```

ACK 応答後、Slave はソフトウェアリセットを実行する。

## 8.7 CLEAR_BUFFER

Slave 内部のバッファをクリアする。

```text
CMD: 0x07
```

### Master → Slave

```text
[0x07] [buffer_type]
```

### buffer_type

| 値 | 内容 |
|---:|---|
| 0x00 | 全バッファクリア |
| 0x01 | 受信バッファクリア |
| 0x02 | デコードバッファクリア |
| 0x03 | 遅延バッファクリア |

### Slave → Master

```text
[ACK]
```

### 備考

動作中に各 Slave の遅延がばらついた場合、Master は `buffer_type = 0x03` を指定して遅延バッファを強制的にクリアできる。

## 8.8 SET_DELAY

Slave ごとの再生遅延を設定する。

```text
CMD: 0x09
```

### Master → Slave

```text
[0x09] [delay_L] [delay_H]
```

### delay_ms

`delay_L` と `delay_H` は、リトルエンディアンの符号付き 16 ビット整数として扱う。

```text
delay_ms = int16_t(delay_H << 8 | delay_L)
```

| 項目 | 内容 |
|---|---|
| 単位 | ms |
| 型 | int16_t |
| 推奨範囲 | -1000 ～ +1000 ms |

例：

| 設定値 | int16表現 | 送信バイト |
|---:|---:|---|
| +100 ms | 0x0064 | `[0x64] [0x00]` |
| -100 ms | 0xFF9C | `[0x9C] [0xFF]` |

### Slave → Master

```text
[ACK]
```

## 9. Bluetooth 接続管理フロー

Bluetooth ターゲットの重複接続を防ぐため、Master が全 Slave の接続状態を管理する。

### 9.1 基本ルール

- Slave は Bluetooth ターゲットを発見しても、ただちに接続しない。
- Slave は発見した Bluetooth ターゲットを接続保留状態にする。
- Master は各 Slave をポーリングし、保留中デバイスの有無を確認する。
- Master は既に接続中の Bluetooth ターゲットと比較し、接続可否を判断する。
- Slave は Master から接続可通知を受けた場合のみ接続を開始する。

### 9.2 接続判定ルール

Master は、保留中 MAC アドレスについて以下の判定を行う。

| 条件 | 判定 |
|---|---|
| 他の Slave が同じ MAC アドレスに接続中 | 接続不可 |
| 同一 Slave が同じ MAC アドレスに接続中 | 接続可 |
| どの Slave も同じ MAC アドレスに接続していない | 接続可 |

同一 Slave が同じ MAC アドレスに接続中の場合は、再接続または接続維持の可能性があるため、接続可として扱う。

### 9.3 処理シーケンス

```text
1. Slave が Bluetooth ターゲットを発見
2. Slave は接続を保留する
3. Master が GET_PENDING_TARGET で保留中 MAC を取得
4. Master が各 Slave の GET_CONNECTED_TARGET を確認
5. Master が重複接続の有無を判定
6. Master が NOTIFY_CONNECT_PERMISSION で接続可否を通知
7. 接続可の場合、Slave は接続を開始
8. 接続不可の場合、Slave は該当 MAC を無視して探索を継続
```

## 10. ポーリング周期

ポーリング周期は実装時に調整する。

初期値の目安：

| 処理 | 目安周期 |
|---|---:|
| GET_STATUS | 100 ～ 500 ms |
| GET_PENDING_TARGET | 100 ～ 500 ms |
| GET_CONNECTED_TARGET | 500 ～ 1000 ms |

Bluetooth 接続保留中は、対象 Slave へのポーリングを一時的に高頻度化してもよい。

## 11. 予約・拡張

以下のコマンド番号は将来拡張用に予約する。

| CMD | 候補 |
|---:|---|
| 0x08 | GET_ERROR |
| 0x0A | GET_DELAY |
| 0x0B | GET_VERSION |
| 0x0C | GET_BT_RSSI |
| 0x0D | CLEAR_ERROR |

## 12. 実装上の注意

- I2C アドレスは暫定的に `0x30 + Slave ID` とする。
- MAC アドレスは文字列ではなく 6 バイトバイナリとして扱う。
- ブロードキャストは使用しない。
- Slave から Master への自発送信は行わない。
- Slave は Master からの接続可否通知が来るまで、保留中ターゲットへ接続しない。
- Master は全 Slave の接続中 MAC アドレスを内部に保持する。
- 接続不可通知を受けた Slave は、該当 MAC アドレスを一定時間無視することが望ましい。
- 遅延設定変更後、必要に応じて遅延バッファクリアを実行する。

