# M5Tab-MIDIXposeFil

M5Stack Tab5 (ESP32-P4) を母艦にした MIDI ライブパフォーマンス機。
従来の M5Core2 版 [M5Core2-MIDIXposeFilBT](../M5Core2-MIDIXposeFilBT/) を Tab5 の 1280×720 大画面とタッチ操作に合わせて作り直し、`SRC 音源切替` / `SMF プレーヤー` / `MP3 プレーヤー` を統合した 3 アプリ構成にしている。

起動時に "**OWAMIDICON-Tab**" のスプラッシュロゴを 3 秒表示してからアプリ画面に入る。

```
┌──────────┬──────────┬──────────┐
│  XPOSE   │   MSG    │   PLAY   │   ← 大メニュー (アプリ — ヘッダ右側)
├──────────┼──────────┼──────────┤
│  DIR.    │  FILTER  │   SRC    │
│  KEY     │  MAPPER  │   SMF    │
│  INST.   │  BYPASS/ │   MP3    │
│  SEQ.    │  ACTIVE  │          │
└──────────┴──────────┴──────────┘
```

ヘッダ左の本体タイトルは選択中のアプリ/モードに合わせて自動で変わる:

| 選択 | タイトル |
|---|---|
| XPOSE 全モード | `MIDI Transposer` |
| MSG / FILTER | `MIDI Filter` |
| MSG / MAPPER | `MIDI Mapper` |
| PLAY / SRC | `Sound Source` |
| PLAY / SMF | `SMF Player` |
| PLAY / MP3 | `MP3 Player` |

ヘッダ右側はアプリ切替タブ (`XPOSE` / `MSG` / `PLAY`) の右に、`BASE` (転調基準ピッカー) → `CONF` (`/config.json` エディタ) → `MIX` (入力ソース選択 USB/MIDIIN/MIX) の順でボタンが並び、最右端に状態表示 (`K +n` 転調値 / `PLAY MM:SS` 経過時間) が右寄せで載る。最も視線が集まる現在値が常に画面右端という配置にしている。`BASE` / `CONF` を開いている間は `XPOSE` 等のアプリタブのハイライトは解除され、現在のオーバーレイのボタンだけがアクティブ表示になる。ツールバー (2 段目) は各アプリのサブモードタブ + トランスポート/補助ボタン専用で、横幅を有効に使えるレイアウトにしてある。

画面最上端の中央〜右には **MIDI アクティビティ・インジケータ**として 2 本の細い縞 (青 = MIDI IN、赤 = MIDI OUT) が並んでいて、メッセージが流れたときにだけ 120 ms ほど明るく光る。`0xF8` (Clock) / `0xFE` (Active Sense) のハートビートは無視するので、演奏が止まれば縞も止まる。数値カウンタは画面には載せず、`STATUS` USB シリアル応答の `midi_in` / `midi_out` (生バイト数) と `midi_in_real` / `midi_out_real` (ハートビート除外) で照会できる。

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

### 3. PLAY — 再生 / 音源切替

| サブモード | 用途 |
|---|---|
| **SRC** (Sound Source) | M5 Unit MIDI (SAM2695) GM 音源のプログラム / Volume / Pitch Bend / Sustain をチャンネル別に設定。GM 128 音色 + Drum Kit 9 種から選択 |
| **SMF** | SD 上の `/smf/**/*.mid` を再生。PREV / PLAY / NEXT / LOOP ON/OFF |
| **MP3** | SD 上の `/mp3/**/*.mp3` を内蔵スピーカで再生。PREV / PLAY / NEXT / VOL± |

SMF / MP3 は転調回路を通らない (素のデータをそのまま再生)。再生中はヘッダ右に `PLAY MM:SS` で経過時間が出る (SMF は 200 ms 周期、MP3 は 500 ms 周期で更新)。

#### SMF / MP3 プレイリスト UI

両モード共通の曲選択リスト仕様:

- 各行は **FONT_SMALL × 44 px lineH** の大きめタッチターゲット。指で押し間違えにくい。
- リスト右上に **4 つのページング・ボタン** (右寄せ、高さ 64 px): `▲▲` (140 px、ページ上送り) / `▲` (90 px、1 行上) / `▼` (90 px、1 行下) / `▼▼` (140 px、ページ下送り)。`▲▲` / `▼▼` の中の二重三角は **横並び** で描画され、単発ボタンと一目で区別が付く。1 ページ送りは `visible − 1` 行。
- **自然順ソート (case-insensitive)**: `0_song.mid` → `1_song.mid` → `2_song.mid` → `10_song.mid` の順。ファイル名先頭の数字も数値として比較するので、桁数違いでも期待通りに並ぶ。
- **サブフォルダ対応**: `/smf` または `/mp3` の直下・配下にサブディレクトリがあれば `[フォルダ名]` 形式 (シアン色) でリストに混ざる。タップで降り、親に戻るときは先頭の `[..]` 行をタップ。並び順は `[..]` → サブフォルダ群 → ファイル群、それぞれ自然順。
- **1 フォルダあたり最大 1024 ファイル**。プレイリストバッファは PSRAM に動的確保 (初回スキャン時)。フォルダ移動のたびに **そのフォルダだけ** を再スキャンする方式で、サブツリー全体を舐めることはしない。内部 DRAM は消費しない。
- PREV / NEXT トランスポート (ヘッダ下のツールバー) は `[..]` やサブフォルダ entry を自動でスキップし、ファイルだけを循環。
- 隠しファイル / ドット始まり (`.DS_Store` 等) は無視。

#### SRC — 音源切替モード (PLAY のデフォルト)

転調パイプライン (FILTER → MAPPER → Transpose → 出力) と並行して動作し、M5 Unit MIDI の GM 音色を切替えるためのライブ操作画面。受信中の MIDI は通常通り転調されて出力される (鍵盤で演奏しながら音色を選べる)。

レイアウト:

```
┌── Toolbar ─────────────────────────────────────────────────────┐
│  [SRC] [SMF] [MP3]  ... [GMRST] [GSRST] [INIT] [AUTO]          │
├────────────────────────────────────────────────────────────────┤
│  [01][02][03][04][05][06][07][08][09][10][11][12]...[16]       │ ← Ch ストリップ
│                                                                 │
│  [PRG-]      PRG:001  Grand Piano 1            [PRG+]          │ ← タップで全画面ピッカー
│                                                                 │
│  ┌── ピアノロール (active ch のみ、3秒ウィンドウ、12色) ──┐    │
│  │       │                                                │     │
│  │   ╿╿  │   ╿                                            │     │
│  │   ││  │   │                                            │     │
│  └──────────────────────────────────────────────────────────┘   │
│  ╞══════ 88 鍵盤 (押下中の鍵が音名カラーで点灯) ═════════╡    │
├────────────────────────────────────────────────────────────────┤
│  [V-] VOL:100 [V+]  [PB-] PB:+0 [PB+]  [SUS ON/OFF]            │ ← navArea
└────────────────────────────────────────────────────────────────┘
```

| 操作 | 動作 |
|---|---|
| Ch セルタップ | アクティブチャンネル変更。同時に `AUTO OFF` (手動操作で自動追従を解除) |
| `AUTO` ボタン | 受信 MIDI の Ch でアクティブセルを自動移動するモード ON/OFF (色で状態表示) |
| Ch10 選択 | ピッカーが GM Drum Kit (Standard/Room/Power/Electronic/TR-808/Jazz/Brush/Orchestra/SoundFX) に切替 |
| **PRG 名バナーをタップ** | 全画面 Instrument Picker を開く (4×8 = 32 cell × 4 ページで全 128 音色) |
| `PRG-` / `PRG+` | 1 つずつカタログ送り (細かい調整用) |
| `V-` / `V+` | CC#7 (Channel Volume) を ±4 |
| `PB-` / `PB+` | Pitch Bend を ±512 (中心 8192) |
| `SUS ON/OFF` | CC#64 (Sustain Pedal) トグル |
| `GMRST` | `F0 7E 7F 09 01 F7` (Universal Non-Realtime GM System On) を送信し全 ch 状態を初期化 |
| `GSRST` | Roland GS Reset SysEx を送信し全 ch 状態を初期化 |
| `INIT` | All Notes Off + GS Reset + CC#121 (Reset All Controllers) で完全リセット (パニックボタン) |

**音名カラー (12 ピッチクラス)**: ピアノロールのバーと、点灯中の鍵盤キーは音名 (C/C#/D/.../B) ごとに HSV 色相 0°/30°/60°/.../330° の 12 色で塗り分ける。**オクターブ違いは同じ色** なので、和音の構成音が一目で分かる。

**ピアノロール**: アクティブ ch の Note On/Off を 3 秒ウィンドウで縦軸スクロール表示。下端 = 現在、上方向 = 過去。バーの x 座標は下の 88 鍵盤と完全にカラム整列するので、どの鍵がいつ押されたかが視覚的に判る。

**ヘッダ右** にはアクティブ Ch と現在の Program 番号 (`CH:nn` / `PRG:nnn` または `DRUM:nnn`) が小フォントで表示される。`AUTO ON` 中は緑、手動指定中はオレンジで色分け。

**永続化**: 選択値は CONFIG SAVE で `/config.json` に保存され、再起動時にアクティブ Ch / Program / Volume が復元される。

#### Instrument Picker (全画面オーバーレイ)

PRG 名バナーをタップで開く。レイアウト:

- ヘッダ: `GM Instruments — Ch01  (page 1/4)` (Ch10 では `Drum Kits — Ch10  (page 1/1)`)
- 4 列 × 8 行 = 32 cell/page、現在選択中の音色がハイライト
- 開いたとき自動的に現在のセルがあるページにジャンプ
- フッター: `< PAGE` / `PAGE >` / `CLOSE`(赤)
- セルタップ → 即 Program Change 送信 → 自動で閉じる

---

## ハードウェア

| 項目 | 値 |
|---|---|
| ボード | M5Stack Tab5 (ESP32-P4 + ESP32-C6) |
| 画面 | 1280 × 720 静電容量タッチ |
| MIDI I/O | PortA を UART に転用 (RX = G54, TX = G53, 31250 bps) |
| ストレージ | microSD (SPI: SCK=43, MOSI=44, MISO=39, CS=42) |

フットペダルには非対応 (Tab5 は BT Classic を持たず、BLE HID 経路は安定性問題で削除)。転調の prev/next は画面下端の PREV / NEXT ボタンで操作する。

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

## ブート設定 `/config.json`

SD ルートに `/config.json` (ArduinoJson 形式) があると起動時に読まれて、デフォルトのアプリ / 転調値 / 入力ソース等が決まる。SD 未挿入・ファイル無し・パース失敗のいずれも組込のデフォルト動作にフォールバック。スキーマは M5Core2-MIDIXposeFilBTUM と完全に共通:

| キー | 値の例 / 意味 |
|---|---|
| `DefaultApp` | `Transpose` / `Play` / `SRC` / `SMF` / `MP3` / `Filter` / `Change` |
| `DefaultTransposeMode` | `DIRECT` / `KEY` / `INSTANT` / `SEQUENCE` |
| `InitialTranspose` | `TransposeBase` からのオフセット (整数) |
| `TransposeBase` | 転調基準 (整数, -12..+12) |
| `InitialAllNotesOff` / `InitialFilterBypass` / `InitialMapperBypass` | bool |
| `TransposeRange` | `0..11` / `-11..0` / `-5..6` |
| `MidiInputSource` | `USB` / `MIDIIN` / `MIX` (Tab5 で実際に切替) |
| `MajorUpperTranspose` / `ShowSplash` | bool |
| `StartupGSReset` | bool — 起動時に GS Reset SysEx を送るか (default `false`) |
| `SmfStartGSReset` | bool — SMF 再生開始時に GS Reset SysEx を送るか (default `true`) |
| `SrcInitChannel` | 1..16 — SRC 起動時のアクティブチャンネル (default `1`) |
| `SrcInitProgram` | 0..127 — SRC 起動時のプログラム番号 (default `0`) |
| `SrcInitVolume` | 0..127 — SRC 起動時の CC#7 Volume (default `100`) |
| `SrcAutoChannel` | bool — 受信 MIDI の Ch を自動追従 (default `true`) |

設定は本体の **CONFIG エディタ** から編集 → SAVE / APPLY できる。

> **デフォルト挙動の変更 (2026-05-08)**: 起動時の GS Reset 送信は **デフォルトで OFF**。SAM2695 が前回の音色設定を保持しているとき、これを毎回ブートでクリアしないようにした。従来通り起動時にリセットしたい場合は CONFIG で `StartupGSRst: ON` に切替える。SMF 再生開始時のリセット (`SmfStartGSRst`) は従来通り ON のまま。

### 画面トリガ

- ヘッダ右の **CONF** ボタン (新, アプリタブの右隣 2 つめ) → `CONFIG_EDIT_MODE` を直接開く。フィールドをタップで巡回、**4 件 × 5 ページ** (合計 18 項目) + footer の SAVE / CANCEL / APPLY。`PAGE-` / `PAGE+` でページ間を往復。
- ヘッダ右の **BASE** ボタン (新, アプリタブの右隣 1 つめ) → `BASE_SET_MODE` (3 ページの転調基準ピッカー: -12..-1 / -5..+6 / +1..+12)。数字は `XPOSE/DIRECT` と同じ大きさ (FONT_HUGE) で **黄色**で表示し、選択中の値は緑バックの黒文字。`+6` → R / `-5` → L / `+1` → M / `-1` → M / `PAGE` で巡回 / `EXIT` で戻る。
- 旧来の長押しジェスチャ (BT ステータスラベル長押し → CONF、AOFF 長押し → BASE) も引き続き動作するフォールバック。
- いずれのオーバーレイ表示中も、ヘッダのアプリタブ / `MIX` / `AOFF` / `BASE` / `CONF` をタップすると一旦オーバーレイをキャンセルしてから当該操作を実行するので、ナビゲーションが詰まらない。

シリアルからは `MODE CONFIG` (`MODE CONFIG_EDIT`) / `MODE BASE` (`MODE BASE_SET`) でも入れる。

## USB シリアルコマンド

PC から本体を遠隔操作できる (115200 bps、改行 LF / CRLF)。

| コマンド | 動作 |
|---|---|
| `HELP` | コマンド一覧 |
| `STATUS` | アプリ / モード / 転調値 / FILTER・MAPPER 状態 / `midi_in`・`midi_out` (生バイト数) / `midi_in_real`・`midi_out_real` (ハートビート除外) / BT を 1 行で返す |
| `REDRAW` | 画面再描画 |
| `INFO SCREEN` | 画面サイズを返す |
| `BUTTON A\|B\|C [LONG]` | 物理ボタン疑似操作 (Tab5 はハードボタンが無いので疑似) |
| `TOUCH x y` | 画面の (x,y) をタップ (現状は座標エコーのみ) |
| `MODE DIRECT\|KEY\|INSTANT\|SEQUENCE\|FILTER\|MAPPER\|MIDI\|SRC\|SMF\|MP3\|CONFIG\|BASE` | 指定モードへジャンプ (`CONFIG`/`BASE` でオーバーレイ起動) |
| `GROUP TRANSPOSE\|MIDI` | アプリ切替 |
| `SET TRANSPOSE n` | 転調値を直接設定 (-12..12) |
| `SET FILTER BYPASS\|ACTIVE` | FILTER 全体の有効化 |
| `SET FILTER BYPASS 0\|1` | FILTER バイパス (0=ACTIVE, 1=BYPASS) |
| `SET FILTER ENABLED <n> 0\|1` | FILTER ルール n を個別 EN/DIS |
| `SET MAPPER BYPASS\|ACTIVE` | MAPPER 全体の有効化 |
| `SET MAPPER BYPASS 0\|1` | MAPPER バイパス (0=ACTIVE, 1=BYPASS) |
| `SET MAPPER ENABLED <n> 0\|1` | MAPPER ルール n を個別 EN/DIS |
| `LOAD TESTRULES` | リグレッション用ルール (PB/CC ブロック + Note Ch1→Ch2 / Ch3 vel スケール) を流し込む |

## 自動リグレッションテスト

`scripts/test_sequence.py` は本機 USB-CDC + UM-ONE 0/1 を使った 6 フェーズの自動回帰テストオーケストレータです。`LOAD TESTRULES` で既知ルールを投入し、`midi_capture_in.py` で MIDI OUT を捕まえて、`STATUS` の MIDI in/out カウンタと `[mem]` (`-DM5TAB_DIAG` ビルド時) のヒープ推移を CSV に書き出します。リーク検出と FILTER/MAPPER のスループット確認を 1 コマンドで回せます。

短いケーブル疎通チェック用に `scripts/check_out_path.py` も同梱: UM-ONE 1 → 本機 MIDI IN → 本機 MIDI OUT → UM-ONE 0 の片道経路を 4 秒のバーストで検証し、`PASS: N bytes captured` か `FAIL: no bytes received on UM-ONE 0` を返します。Unit MIDI モジュールに **ハードウェア MIDI THRU スイッチ ON** が残っているとケーブル直結で false PASS になるので、本テストの前に THRU は OFF にしておくこと。

### 動作確認済み (2026-05-07)

`-DM5TAB_DIAG` ビルドで `scripts/test_sequence.py` を 6 フェーズ × 60 秒 (passthrough / transpose +5 / filter PB / filter PB+CC / mapper Ch1→Ch2 / mapper Ch1→Ch2 + Ch3 vel halve) 通過、Tab5 (COM7, Unit MIDI on PortA) で全フェーズ PASS。`midi_out` カウンタとホスト側キャプチャは 99.98% 一致 (例: phase 1 で 48,633 vs 48,622)。`[mem] all_min` は phase 1 の暖機 21 KB ドロップと phase 3 の filter init 4.9 KB ドロップ後、**phase 4–6 は 31,735,484 で固定** (リーク無し)。`stack_hw_min=30,248`。

ベンチ用ハードウェア構成は memory ファイル参照 (Roland UM-ONE / `mido` ポート名 `UM-ONE 0` `UM-ONE 1` / `pyserial`)。

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
| [`sketch.yaml`](./sketch.yaml) | Arduino-CLI プロファイル |
| [`build.cmd`](./build.cmd) | ビルドラッパー (PSRAM/POWER_CHANNEL workaround 入り) |
| [`src/gm_instruments.h`](./src/gm_instruments.h) | GM 128 音色名 + Drum Kit テーブル (SRC モード用) |
| [`scripts/chord_diag.py`](./scripts/chord_diag.py) | 和音 Note Off 落ち診断 (STATUS 差分で入出力カウンタを照合) |

---

## 設計メモ

- **MIDI 処理パイプライン** は `processMIDIByte` でバイトを溜めて `MidiMessage` 構造体を組み立て、`handleParsedMidiMessage` 経由で FILTER → MAPPER → Transpose → `Serial2` の順に流す。SysEx もペイロード単位で FILTER 対象。
- **Tab5 用の MAPPER 編集 UI** は M5Core2 と違って PG1/PG2 を切り替えず、**SOURCE 側と DESTINATION 側を左右並列**で同時表示する。1280 px 幅を生かしている。
- **大メニュー / サブメニュー** は 2 段構成: 大メニュー (XPOSE/MSG/PLAY) はヘッダ右側に常駐、サブモードタブ + 機能ボタンはその下のツールバーに展開する。アプリを切り替えるとツールバーの内容が DIR./KEY/INST./SEQ. (XPOSE) → FILTER/MAPPER/BYPASS (MSG) → SMF/MP3 + トランスポート (PLAY) と差し替わる。
- 転調実行中の Note On / Note Off は `currentNoteStates[]` / `savedNoteStates[]` でペアリング保持しているので、転調値変更時も「同じ鍵が止まらない」現象が出ない。
- **SRC モードの MIDI 経路**: SRC は転調パイプラインの上にかぶせる UI レイヤであり、入力 MIDI は通常通り FILTER → MAPPER → Transpose → Serial2 を流れる。Program Change / CC / Pitch Bend は SRC の操作に応じて UI 側から `Serial2.write()` で直接 Unit MIDI へ送出する。チャンネル別に GM Program / Volume / PB / Sustain をキャッシュしているので、Ch を切替えるたびに状態が再送される。
- **専用 MIDI I/O タスク (2026-05-08)**: 旧構成は `processMidiInput()` をメインループから呼ぶ作りで、SRC モードのピアノロール再描画や `drawInterface()` の重い `fillRect` が走るたびに MIDI 入力ドレインが数 ms 〜 数十 ms 止まり、演奏中の和音バイトが Serial2 RX 内で詰まっていた。専用タスク `midi_io_task` (FreeRTOS, **core 1**, priority 10) に切り出して 1 ms 周期で動くようにし、UI 側の重い処理に関係なく MIDI 入出力を維持。タスクは UI と同じ core 1 で動くが優先度が高いので必要時に loopTask をプリエンプトする。core 0 は USB ホストに専用で残す (USB-MIDI バイトのプリエンプトを避ける)。
- **Serial2 TX ミューテックス**: 専用 MIDI タスクと UI 側からの送信 (`sendGSReset` / `sendGMReset` / `sendUnitProgramChange` / `sendAllNotesOff` / SMF 再生時の `smfMidiEventHandler`) が同じ Serial2 TX FIFO を共有するため、再帰ミューテックス `g_serial2TxMux` で各メッセージ送信を直列化している。バイト列のインターリーブで Note Off が破損して音がスタックする問題を防ぐ。
- **USB ホストの並列 IN 転送 (2026-05-08)**: 旧構成は USB MIDI IN 転送を 1 つだけ `usb_host_transfer_submit` していた。完了コールバック処理中は次の転送がキューにないので、その隙にキーボードが送ったバイトが USB ホストドライバ層で取りこぼされていた (我々のリングカウンタ `usb_drop` には乗らないので不可視のドロップ)。**16 並列**で常時 in-flight にし、コールバックは個別の転送だけ再投入する設計に変更。再投入失敗にも 3 回までリトライを入れて、キュー深度が永続的に下がらないようにしている。USB MIDI 入力リングは 1 KB → 8 KB に拡大。
- **STATUS の診断カウンタ**: `midi_in` / `midi_in_real` / `midi_out` / `midi_out_real` (heartbeat 除外) に加えて、`usb_drop` (USB MIDI リング溢れバイト数) と `usb_resub_fail` (USB IN 再投入失敗回数) を追加。`scripts/chord_diag.py` で chord on+off の前後に STATUS を撮って差分を見れば、入力で落ちているか出力で落ちているかが切り分けられる。

## 関連プロジェクト

- [`M5Core2-MIDIXposeFilBT`](../M5Core2-MIDIXposeFilBT/) — 元の M5Core2 + 古典 BT HID 版。同じ FILTER/MAPPER/SEQUENCE 機能のオリジナル。
