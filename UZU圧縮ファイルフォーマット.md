# UZU File Compression Format Specification

# UZU圧縮ファイルフォーマット仕様書

Revision: R0  
Author: Serry / ChatGPT  
Format: Markdown  
Status: Draft

---

# 1. 概要

本仕様は、UZU CAST 用のマルチチャンネル圧縮音声フォーマットについて定義する。

UZUフォーマットでは、各チャンネルを独立したモノラル圧縮ストリームとして扱い、
時間順にインターリーブして格納する。

本方式の特徴は以下である。

- 各チャンネルを独立してデコード可能
- Bluetoothモジュール側で通常のMP3ストリームとして扱える
- メインCPU側でMP3展開不要
- 固定長スロット構造により高速シーク可能
- DMA / FPGA / TDMとの親和性が高い
- 将来的なCodec変更に対応可能

---

# 2. 基本構造

UZU圧縮ファイルは以下の構造を持つ。

```text
[File Header]
[Frame Block 0]
[Frame Block 1]
[Frame Block 2]
...
```

---

# 3. Codec仕様

## 3.1 初期対応Codec

初期仕様では以下を使用する。

| 項目         | 内容      |
| ---------- | ------- |
| Codec      | MP3     |
| Mode       | Mono    |
| Bitrate    | CBR     |
| SampleRate | 44.1kHz |
| Channels   | 最大8CH   |

---

## 3.2 将来拡張

将来的に以下Codecへの対応を想定する。

- AAC
- Opus
- LC3
- ADPCM

Codec識別は File Header 内の CodecType により行う。

---

# 4. File Header

## 4.1 Header構造

| Offset | Size | 内容                |
| ------ | ---- | ----------------- |
| 0x00   | 4    | Magic "UZUC"      |
| 0x04   | 2    | Version           |
| 0x06   | 2    | CodecType         |
| 0x08   | 2    | ChannelCount      |
| 0x0A   | 4    | SampleRate        |
| 0x0E   | 4    | BitRatePerChannel |
| 0x12   | 2    | MaxFrameSize      |
| 0x14   | 2    | FrameDurationMs   |
| 0x16   | 4    | TotalFrameCount   |

---

## 4.2 CodecType

| 値   | 内容    |
| --- | ----- |
| 0   | MP3   |
| 1   | AAC   |
| 2   | Opus  |
| 3   | LC3   |
| 4   | ADPCM |

---

# 5. Frame Block構造

## 5.1 概要

各Frame Blockは、全チャンネルの同一時間位置の圧縮データを含む。

例：

```text
FrameBlock0:
  CH1 Frame0
  CH2 Frame0
  CH3 Frame0
  CH4 Frame0

FrameBlock1:
  CH1 Frame1
  CH2 Frame1
  CH3 Frame1
  CH4 Frame1
```

---

# 6. Channel Slot構造

## 6.1 固定長スロット方式

各チャンネルは固定長スロットを持つ。

```text
[MaxFrameSizeField]    ; 2 byte, 必須（各スロット先頭）
[RealFrameSizeField]   ; 2 byte, 必須
[FrameData]
[Padding]
```

File Header の `MaxFrameSize`（0x12）はファイル全体の上限値である。
**各スロット先頭の `MaxFrameSizeField` は通常これと同値**とし、受信側がヘッダ無しでも
固定スロット長（`SlotSize`）を決定できるようにする。

---

## 6.2 Slot詳細

| 項目                 | Offset (スロット内)       | Size   | 内容                                                            |
| ------------------ | -------------------- | ------ | ------------------------------------------------------------- |
| MaxFrameSizeField  | 0x00                 | 2 byte | FrameData+Padding 領域の固定長（UINT16 LE, 通常 = Header.MaxFrameSize） |
| RealFrameSizeField | 0x02                 | 2 byte | 実際の圧縮フレームサイズ（UINT16 LE）                                       |
| FrameData          | 0x04                 | 可変     | MP3フレームデータ（長さ = RealFrameSize）                                |
| Padding            | 0x04 + RealFrameSize | 残り     | 未使用（0 推奨）。デコーダへ渡してはならない                                       |

- `MaxFrameSizeField` は **4 以上**、File Header の `MaxFrameSize` と一致すること（推奨・必須扱い）
- `RealFrameSize` は **1 以上**、**MaxFrameSizeField 以下**
- `FrameData` の先頭は MP3 同期語（`0xFF` / `0xE0` 以上）であること
- I2S / TDM 転送時も **スロット全体（両フィールドを含む）** を送受信する

---

## 6.3 Slotサイズ

Slot全体サイズは固定。

```text
SlotSize = MaxFrameSizeField (2) + RealFrameSizeField (2) + MaxFrameSize
         = 4 + MaxFrameSize
```

File Header 0x1E の `SlotSize` は上式と一致すること。

---

## 6.4 Padding

Padding領域は未使用データであり、
MP3デコーダへ入力してはならない。

Bluetoothモジュール側では、
RealFrameSize分のみをデコーダへ入力する。

---

# 7. MP3フレームについて

## 7.1 MP3フレーム構造

MP3フレームは以下の構造を持つ。

```text
[Header]
[CRC(Optional)]
[Side Information]
[Main Data]
```

---

## 7.2 フレームサイズ

MP3 CBRであっても、
フレームサイズはPadding Bitにより若干変動する。

例：

```text
417byte
418byte
417byte
418byte
```

このためUZUフォーマットでは、
固定長スロット内に可変長フレームを格納する。

---

# 8. シーク動作

## 8.1 概要

固定長スロット方式により、
フレーム位置を直接計算可能。

---

## 8.2 計算式

```text
FrameOffset =
  HeaderSize +
  FrameNo * BlockSize
```

---

## 8.3 利点

- 高速シーク
- SDアクセス効率向上
- DMA処理簡略化
- FPGA実装容易化

---

# 9. 再生動作

## 9.1 メインESP32側

メインESP32ではMP3展開を行わない。

処理内容：

```text
UZUファイル読込
↓
CH別Frame抽出
↓
各Bluetoothモジュールへ転送
```

---

## 9.2 Bluetoothモジュール側

Bluetoothモジュール側では、
自身のチャンネルのみデコードする。

```text
Frame受信
↓
Padding除去
↓
MP3 Decode
↓
PCM生成
↓
Bluetooth送信
```

---

# 10. メリット

- メインCPU負荷低減
- Bluetoothモジュールの独立性
- 高速シーク
- MP3ストリーミング互換
- 既存構成との互換性維持
- 将来的なCodec拡張可能

---

# 11. 今後の課題

- 最適Bitrateの検証
- ESP32側MP3デコード負荷測定
- Bluetooth遅延との整合性確認
- Blockサイズ最適化
- CRC追加検討
- Index Table追加検討

---

# 12. 推奨初期パラメータ

| 項目            | 値        |
| ------------- | -------- |
| Codec         | MP3 Mono |
| Bitrate       | 96kbps   |
| SampleRate    | 44.1kHz  |
| Channels      | 8CH      |
| MaxFrameSize  | 420byte  |
| FrameDuration | 約26ms    |

---

# 13. 参考

本仕様は、
「複数独立音源をリアルタイム空間再配置する」
ことを目的として設計されている。

一般的なStereo音声圧縮とは異なり、
各チャンネルの独立性を重視する。

Joint Stereoは使用しない。
