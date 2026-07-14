# UZU Compressed TDM/I2S Transfer Specification

# UZU圧縮データ TDM/I2S転送仕様書

Revision: R0  
Author: Serry / ChatGPT  
Status: Draft

---

# 1. 概要

本仕様は、UZU CAST における圧縮音声データの、  
Main ESP32 から FPGA、および Slave ESP32 への  
TDM / I2S 転送方式について定義する。

本仕様では、  
MP3圧縮データをリアルタイム同期転送する。

---

# 2. システム構成

```text
UZU File
↓
Main ESP32
↓ TDM
FPGA
↓ I2S x N
Slave ESP32
↓
MP3 Decode
↓
Bluetooth Audio
```

---

# 3. 基本思想

非圧縮モードでは、  
TDM/I2SはPCMオーディオ転送として動作する。

圧縮モードでは、  
TDM/I2Sは「同期付き圧縮データ転送バス」として動作する。

つまり、  
I2S信号上をPCMではなく、  
MP3 Frameデータが流れる。

---

# 4. 非圧縮モード

## 4.1 概要

通常のPCM転送を行う。

| 項目         | 内容              |
| ---------- | --------------- |
| Format     | I2S             |
| BitWidth   | 16bit           |
| Channels   | 8CH TDM         |
| SampleRate | 44.1kHz / 48kHz |

---

## 4.2 データ構造

```text
CH1 Sample0
CH2 Sample0
CH3 Sample0
...
CH8 Sample0

CH1 Sample1
CH2 Sample1
...
```

---

# 5. 圧縮モード

## 5.1 概要

各Slave ESP32へ、  
MP3 Frameデータをリアルタイム転送する。

Slave ESP32側では、  
受信したMP3 Frameをデコードし、  
44.1kHz PCMを生成する。

---

## 5.2 転送構造

Main ESP32では、  
UZU圧縮ファイルから各CHのFrameを抽出し、  
TDM転送を行う。

```text
FrameBlock0:
  CH1 MP3 Frame0
  CH2 MP3 Frame0
  CH3 MP3 Frame0
  ...

FrameBlock1:
  CH1 MP3 Frame1
  CH2 MP3 Frame1
  ...
```

---

# 6. MP3 Frame仕様

## 6.1 基本仕様

| 項目              | 内容      |
| --------------- | ------- |
| Codec           | MP3     |
| Mode            | Mono    |
| Bitrate         | CBR     |
| SampleRate      | 44.1kHz |
| SamplesPerFrame | 1152    |

---

## 6.2 Frame Duration

MP3 Frameは、  
1152 sample単位で構成される。

Frame時間は以下となる。

```text
1152 / 44100 ≒ 26.122ms
```

---

# 7. Channel Slot構造

## 7.1 Slot構造

各CHは固定長Slotを持つ。

```text
[MaxFrameSizeField]    ; 2 byte UINT16 LE（各スロット先頭・必須）
[RealFrameSizeField]   ; 2 byte UINT16 LE（必須）
[FrameData]
[Padding]
```

---

## 7.2 Slot詳細

| 項目                 | 内容                                                         |
| ------------------ | ---------------------------------------------------------- |
| MaxFrameSizeField  | 2 byte, UINT16 LE。FrameData+Padding の固定長（I2S 側のスロット長決定用） |
| RealFrameSizeField | 2 byte, UINT16 LE。当該スロットの MP3 実バイト数                          |
| FrameData          | MP3 Frame（長さ = RealFrameSize）                               |
| Padding            | 未使用領域（デコーダ非入力）                                             |

---

## 7.3 推奨サイズ

| 項目                 | 値       |
| ------------------ | ------- |
| MaxFrameSize       | 420byte |
| MaxFrameSizeField  | 2byte   |
| RealFrameSizeField | 2byte   |
| SlotSize           | 424byte (4 + MaxFrameSize) |

---

# 8. TDM転送仕様

## 8.1 概要

TDMでは、  
各CH Slotを順次転送する。

---

## 8.2 転送順序

```text
CH1 Slot
CH2 Slot
CH3 Slot
...
CH8 Slot
```

---

## 8.3 Block期間

1 Blockは、  
1 MP3 Frame期間に一致する。

```text
26.122ms / Block
```

---

## 8.4 TDM転送レート

転送レートは、  
以下条件を満たす必要がある。

```text
1 Block分の全CH Slotを、
26.122ms以内に転送完了すること
```

---

## 8.5 計算例

```text
SlotSize = 422byte
CH = 8
```

Blockサイズ：

```text
422 × 8 = 3376byte
```

必要転送レート：

```text
3376 × 8 / 0.026122
≒ 1.03Mbps
```

---

# 9. I2S仕様

## 9.1 概要

FPGAは、  
TDMデータを各Slave用I2Sへ分配する。

---

## 9.2 I2S構成

I2Sは、  
既存非圧縮モードとの互換性維持のため、  
16bit Stereo形式を使用する。

---

## 9.3 使用方法

| Channel | 用途    |
| ------- | ----- |
| Left    | 圧縮データ |
| Right   | 未使用   |

---

## 9.4 理由

非圧縮モードとのハードウェア互換性を維持するため。

---

# 10. Slave ESP32動作

## 10.1 受信

Slave ESP32は、  
I2S経由でSlotデータを受信する。

---

## 10.2 処理

```text
I2S Receive
↓
Slot Buffer
↓
RealFrameSize取得
↓
Padding除去
↓
MP3 Decode
↓
1152 PCM Sample生成
↓
Bluetooth送信
```

---

# 11. 同期

## 11.1 基本思想

各Slave ESP32は、  
26.122msごとに1 MP3 Frameを受信する。

これにより、  
各CHの再生同期を維持する。

---

## 11.2 FPGA役割

FPGAは、  
全Slaveへ同時同期転送を行う。

---

# 12. 今後の拡張

将来的に以下対応を想定する。

- AAC

- Opus

- CRC

- Frame番号

- Timestamp

- 再同期制御

- Error Recovery

---

# 13. メリット

- Main ESP32負荷低減

- FPGAとの高親和性

- 高速シーク可能

- 非圧縮互換維持

- 各Slave独立動作

- Bluetooth処理分散

---

# 14. 注意事項

I2Sは、  
通常のPCM Audioとしては動作しない。

圧縮モード時のI2Sは、  
「同期付き圧縮データ転送バス」  
として扱う必要がある。

Slave ESP32側では、  
PCMとして解釈してはならない。

---
