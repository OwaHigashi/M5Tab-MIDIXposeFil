# M5Tab-MIDIXposeFil

M5Stack Tab5 (ESP32-P4) を母艦にした MIDI ライブパフォーマンス機。
従来の M5Core2 版 [M5Core2-MIDIXposeFilBT](../M5Core2-MIDIXposeFilBT/) を Tab5 の 1280×720 大画面とタッチ操作に合わせて作り直し、`SMF プレーヤー` と `MP3 プレーヤー` を統合した 3 アプリ構成にしている。

起動時に "**OWAMIDICON-Tab**" のスプラッシュロゴを 3 秒表示してからアプリ画面に入る。

```
┌──────────┬──────────┬──────────┐
│  XPOSE   │   MSG    │   PLAY   │   ← 大メニュー (アプリ — ヘッダ右側)
├──────────┼──────────┼──────────┤
│  DIR.    │  FILTER  │   SMF    │
│  KEY     │  MAPPER  │   MP3    │
│  INST.   │  BYPASS/ │          │
│  SEQ.    │  ACTIVE  │          │
└──────────┴──────────┴──────────┘
```

ヘッダ左の本体タイトルは選択中のアプリ/モードに合わせて自動で変わる:

| 選択 | タイトル |
|---|---|
| XPOSE 全モード | `MIDI Transposer` |
| MSG / FILTER | `MIDI Filter` |
| MSG / MAPPER | `MIDI Mapper` |
| PLAY / SMF | `SMF Player` |
| PLAY / MP3 | `MP3 Player` |

ヘッダ右側にはアプリ切替タブと、現在状態 (転調値 / `PLAY MM:SS` 経過時間 / 音量) が並ぶ。ツールバー (2 段目) は各アプリのサブモードタブ + トランスポート/補助ボタン専用で、横幅を有効に使えるレイアウトにしてある。

メインスケッチは [`M5Tab-MIDIXposeFil.ino`](./M5Tab-MIDIXposeFil.ino)。

---

## 機能概要

### 1. XPOSE — 転調

入力された MIDI Note On / Note Off を半音単位でずらして出力する。4 つの操作モードを場面で使い分ける。

| サブモード | 用途 |
|---|---|
| **DIR.** (DIRECT)  | 12 ボタンの直接選択。-5..+6 / 0..+11 / -11..0 のレンジ切替 |
| **KEY**            | メジャー / マイナー鍵盤からキー指定で転調値を決定 |
| **INST.** (INSTANT)| 0 / ±1 / ±2 / ±3 / ±5 をワンタップで呼び出す |
| **SEQ.** (SEQUENCE)| 16 パターン × 6 ステップの転調列を切替再生 (SD 保存) |

### 2. MSG — MIDI フィルタ / マッパ

転調処理の前段に「不要な MIDI を遮断する FILTER」「MIDI を別メッセージに書き換える MAPPER」を入れている。

```
MIDI IN → FILTER → MAPPER → Transpose → MIDI OUT
```

`FILTER` と `MAPPER` はそれぞれ独立に `BYPASS` ⇄ `ACTIVE` を切り替えられる。両方 BYPASS にすれば素通しの低遅延転調だけが残る。

#### FILTER

条件にマッチした MIDI メッセージを破棄する。1 ルールあたり次を指定する:

- `EN/DIS` (個別有効化)
- `Type` (NoteOff / NoteOn / KeyPrs / PrgChg / CtrlChg / ChPrs / Bend / SysEx / MTC / SongPos / SongSel / TuneReq / Clock / Start / Cont / Stop / ActSn / Reset)
- `Ch` (`ALL` / `Ch1..Ch16`)

#### MAPPER

リスト先頭から順に評価し、最初にマッチした 1 ルールだけが適用される。Tab5 版では **SOURCE と DESTINATION を左右並列で同時表示** していて、PG1/PG2 切替操作は不要。

| 項目 | 意味 |
|---|---|
| `Type`        | メッセージ種別 |
| `Ch`          | チャンネル (送信側は `KEEP` で「元のまま」) |
| `Data1`       | 1 バイト目の値 (`ANY`/`KEEP` で「条件不問」「元のまま」) |
| `Min` / `Max` | 値レンジ。`Min/Max` を別レンジにすればスケーリング |

### 3. PLAY — 再生

| サブモード | 用途 |
|---|---|
| **SMF** | SD 上の `/smf/*.mid` を再生。PREV / PLAY / NEXT / LOOP ON/OFF |
| **MP3** | SD 上の `/mp3/*.mp3` を内蔵スピーカで再生。PREV / PLAY / NEXT / VOL± |

両方とも転調回路は通らない (素のデータをそのまま再生)。再生中はヘッダ右に `PLAY MM:SS` で経過時間が出る (SMF は 200 ms 周期、MP3 は 500 ms 周期で更新)。

---

## ハードウェア

| 項目 | 値 |
|---|---|
| ボード | M5Stack Tab5 (ESP32-P4 + ESP32-C6) |
| 画面 | 1280 × 720 静電容量タッチ |
| MIDI I/O | PortA を UART に転用 (RX = G54, TX = G53, 31250 bps) |
| Bluetooth | BLE HID (フットペダル) — ESP32-C6 経由 ESP-Hosted |
| ストレージ | microSD (SPI: SCK=43, MOSI=44, MISO=39, CS=42) |

ペダルは BLE HID 仕様の機種を使用 (HID Service `0x1812`)。
古典 BT L2CAP HID の SPT-10 は ESP32-P4 では使えないため、最近の BLE HID ペダルを使う。

---

## ライブラリ

`sketch.yaml`:

```yaml
default_fqbn: esp32:esp32:m5stack_tab5
libraries:
  - M5Unified (0.2.13)
  - M5GFX (0.2.20)
  - SdFat (2.2.2)
  - ESP8266Audio (1.9.9)
```

esp32-core 3.3.8 には `BOARD_SDMMC_POWER_CHANNEL` が未定義の問題があるので、コンパイルは付属の [`build.cmd`](./build.cmd) を経由する (`-DBOARD_SDMMC_POWER_CHANNEL=4` を注入)。

```cmd
build.cmd
arduino-cli upload -p COM7 --fqbn esp32:esp32:m5stack_tab5 .
```

---

## USB シリアルコマンド

PC から本体を遠隔操作できる (115200 bps、改行 LF / CRLF)。

| コマンド | 動作 |
|---|---|
| `HELP` | コマンド一覧 |
| `STATUS` | アプリ / モード / 転調値 / FILTER・MAPPER 状態 / MIDI in/out / BT を 1 行で返す |
| `REDRAW` | 画面再描画 |
| `INFO SCREEN` | 画面サイズを返す |
| `BUTTON A\|B\|C [LONG]` | 物理ボタン疑似操作 (Tab5 はハードボタンが無いので疑似) |
| `TOUCH x y` | 画面の (x,y) をタップ (現状は座標エコーのみ) |
| `MODE DIRECT\|KEY\|INSTANT\|SEQUENCE\|FILTER\|MAPPER\|MIDI\|SMF\|MP3` | 指定モードへジャンプ |
| `GROUP TRANSPOSE\|MIDI` | アプリ切替 |
| `SET TRANSPOSE n` | 転調値を直接設定 (-12..12) |
| `SET FILTER BYPASS\|ACTIVE` | FILTER 全体の有効化 |
| `SET MAPPER BYPASS\|ACTIVE` | MAPPER 全体の有効化 |

例:

```text
> MODE FILTER
OK MODE FILTER
> SET FILTER ACTIVE
OK SET FILTER ACTIVE
> STATUS
OK STATUS app=MIDI mode=DIRECT transpose=0 range=2 filter_bypass=0 mapper_bypass=1 ...
```

---

## ファイル構成

| パス | 用途 |
|---|---|
| [`M5Tab-MIDIXposeFil.ino`](./M5Tab-MIDIXposeFil.ino) | メインスケッチ |
| [`src/MD_MIDIFile.{h,cpp}`](./src/) | SMF パーサ |
| [`src/MD_MIDIHelper.{h,cpp}`](./src/) | SMF 補助 |
| [`src/MD_MIDITrack.cpp`](./src/) | SMF トラック |
| [`src/AudioOutputM5Speaker.h`](./src/) | M5.Speaker 用 ESP8266Audio 出力 |
| [`src/ble_hid.{h,cpp}`](./src/) | BLE HID ホスト (フットペダル) |
| [`sketch.yaml`](./sketch.yaml) | Arduino-CLI プロファイル |
| [`build.cmd`](./build.cmd) | ビルドラッパー (PSRAM/POWER_CHANNEL workaround 入り) |

---

## 設計メモ

- **MIDI 処理パイプライン** は `processMIDIByte` でバイトを溜めて `MidiMessage` 構造体を組み立て、`handleParsedMidiMessage` 経由で FILTER → MAPPER → Transpose → `Serial2` の順に流す。SysEx もペイロード単位で FILTER 対象。
- **Tab5 用の MAPPER 編集 UI** は M5Core2 と違って PG1/PG2 を切り替えず、**SOURCE 側と DESTINATION 側を左右並列**で同時表示する。1280 px 幅を生かしている。
- **大メニュー / サブメニュー** は 2 段構成: 大メニュー (XPOSE/MSG/PLAY) はヘッダ右側に常駐、サブモードタブ + 機能ボタンはその下のツールバーに展開する。アプリを切り替えるとツールバーの内容が DIR./KEY/INST./SEQ. (XPOSE) → FILTER/MAPPER/BYPASS (MSG) → SMF/MP3 + トランスポート (PLAY) と差し替わる。
- 転調実行中の Note On / Note Off は `currentNoteStates[]` / `savedNoteStates[]` でペアリング保持しているので、転調値変更時も「同じ鍵が止まらない」現象が出ない。

## 関連プロジェクト

- [`M5Core2-MIDIXposeFilBT`](../M5Core2-MIDIXposeFilBT/) — 元の M5Core2 + 古典 BT HID 版。同じ FILTER/MAPPER/SEQUENCE 機能のオリジナル。
