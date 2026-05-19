// M5Tab-MIDITransposer
// M5Stack Tab5 MIDI live transposer (port of M5Core2-MIDITransposerBT).
// Board : M5Stack Tab5 (ESP32-P4 + ESP32-C6 via ESP-Hosted)
// Libs  : M5Unified, M5GFX, SdFat, ESP8266Audio
//
// The original live transposer is extended with:
//   - SMF player imported from ../../M5Core2-SMF-Player
//   - MP3 player imported from ../../M5Core2-MP3Player
//   - Common app menu and large-format Tab5 UI
//
// MIDI input can come from either:
//   - Tab5 PortA repurposed as UART for M5 Unit MIDI (SAM2695)
//     RX = G54, TX = G53
//   - Tab5 USB-A host port for class-compliant USB-MIDI instruments
// The on-screen selector in the upper-right corner switches the input source.
//
// Differences vs the M5Core2 original:
//   - Full 1280x720 layout, FreeSans proportional fonts, larger tap targets.
//   - No hardware buttons on Tab5, so A/B/C are replaced by on-screen toolbar
//     buttons (ALL OFF, RANGE, MODE).
//   - SD access is unified on the Tab5 SPI-wired microSD slot so the SMF
//     library, MP3 decoder and transposer sequence storage share one card.

#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <math.h>
#include <new>
#include <SdFat.h>
#include <M5Unified.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <driver/uart.h>
#include <AudioFileSourceFS.h>
#include <AudioFileSourceID3.h>
#include <AudioGeneratorMP3.h>
#include <usb/usb_host.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>
#include <freertos/semphr.h>

// =================================================================
//  Serial2 TX mutex — coordinates writes between the dedicated MIDI
//  task (core 0) and the UI thread (core 1). Without it, a chord
//  Note Off coming from the MIDI task can interleave with a UI-driven
//  send (GS Reset / Program Change / All Notes Off) and produce
//  garbled bytes the synth cannot interpret as a Note Off — leaving
//  voices stuck on. Recursive so a function that already holds the
//  lock can call another locked helper safely.
// =================================================================
static SemaphoreHandle_t g_serial2TxMux = nullptr;

class Serial2TxLock {
  bool taken;
 public:
  Serial2TxLock() {
    taken = (g_serial2TxMux != nullptr) &&
            (xSemaphoreTakeRecursive(g_serial2TxMux, portMAX_DELAY) == pdTRUE);
  }
  ~Serial2TxLock() {
    if (taken && g_serial2TxMux) xSemaphoreGiveRecursive(g_serial2TxMux);
  }
  Serial2TxLock(const Serial2TxLock&) = delete;
  Serial2TxLock& operator=(const Serial2TxLock&) = delete;
};

// Define M5TAB_DIAG (e.g. via -DM5TAB_DIAG in build.cmd or the IDE) to
// enable the lightweight `[mem]` heap / stack monitor printed every 5 s.
// Off by default so production builds carry zero diagnostic overhead.
#ifdef M5TAB_DIAG
#include <esp_system.h>
#include <esp_heap_caps.h>
#endif

#define SD_FAT_TYPE 3
#include "src/MD_MIDIFile.h"
#include "src/AudioOutputM5Speaker.h"
#include "src/gm_instruments.h"

// ==== Tab5 PortA repurposed as UART for M5 Unit MIDI (SAM2695) ====
#define RXD2 54
#define TXD2 53
#define MIDI_BAUD 31250

// ==== Tab5 microSD SPI pinout ====
#define SD_SPI_SCK  43
#define SD_SPI_MOSI 44
#define SD_SPI_MISO 39
#define SD_SPI_CS   42

// ==== Screen ====
static int SCREEN_W = 1280;
static int SCREEN_H = 720;

// ==== Fonts (proportional, anti-aliased in M5GFX) ====
#define FONT_TITLE   (&fonts::FreeSansBold24pt7b)
#define FONT_HUGE    (&fonts::FreeSansBold24pt7b)
#define FONT_LARGE   (&fonts::FreeSansBold18pt7b)
#define FONT_MED     (&fonts::FreeSans18pt7b)
#define FONT_SMALL   (&fonts::FreeSans12pt7b)
#define FONT_TINY    (&fonts::FreeSans9pt7b)

// Colors (use the TFT_* names the LGFX driver exposes).
static constexpr uint16_t COL_BG       = TFT_BLACK;
static constexpr uint16_t COL_PANEL    = 0x10A2; // dark slate
static constexpr uint16_t COL_BTN      = 0x2945; // slate
static constexpr uint16_t COL_BTN_HI   = TFT_GREEN;
static constexpr uint16_t COL_BTN_HI2  = TFT_ORANGE;
static constexpr uint16_t COL_BTN_BDR  = TFT_LIGHTGREY;
static constexpr uint16_t COL_BTN_TXT  = TFT_WHITE;
static constexpr uint16_t COL_BTN_TXT_HI = TFT_BLACK;
static constexpr uint16_t COL_TITLE    = TFT_WHITE;
static constexpr uint16_t COL_MUTED    = 0x7BEF; // grey
static constexpr uint16_t COL_VALUE    = TFT_YELLOW;
static constexpr uint16_t COL_ACCENT   = TFT_CYAN;
static constexpr uint16_t COL_DANGER   = TFT_RED;
static constexpr uint16_t COL_NAVY     = TFT_NAVY;
static constexpr uint16_t COL_WHITEKEY = TFT_WHITE;
static constexpr uint16_t COL_BLACKKEY = 0x2104;

// ==== Modes ====
// 大メニュー (アプリ): XPOSE / MIDI / PLAY
enum AppMode { APP_TRANSPOSE, APP_MIDI, APP_PLAY };
// XPOSE 内のサブモード (4 種) — overlay 用に CONFIG_EDIT / BASE_SET を末尾に追加。
// 既存の `(DisplayMode)i` ループ (modeTab[]) は i=0..3 しか回さないので enum 末尾に
// 値を増やしても影響しない。drawInterface() / handleTouch() 側で overlay 値を
// 先に判定して通常のモード dispatch をスキップする。
enum DisplayMode { DIRECT_MODE, KEY_MODE, INSTANT_MODE, SEQUENCE_MODE,
                   CONFIG_EDIT_MODE, BASE_SET_MODE };
// PLAY 内のサブモード (2 種)
enum PlayMode { PLAY_SRC, PLAY_SMF, PLAY_MP3 };
enum TransposeRange { RANGE_0_TO_12, RANGE_MINUS12_TO_0, RANGE_MINUS5_TO_6 };
enum MidiInputSource { MIDI_INPUT_UNIT, MIDI_INPUT_USB, MIDI_INPUT_MIX };

// ==== MIDI Manager (FILTER + MAPPER) ====
enum MidiManagePage     { MIDI_PAGE_FILTER, MIDI_PAGE_MAPPER };
enum MidiMapperEditPage { MAPPER_PAGE_SOURCE, MAPPER_PAGE_DEST };
enum MidiMessageKind {
  MIDI_KIND_NOTE_OFF,
  MIDI_KIND_NOTE_ON,
  MIDI_KIND_KEY_PRESSURE,
  MIDI_KIND_PROGRAM_CHANGE,
  MIDI_KIND_CONTROL_CHANGE,
  MIDI_KIND_CHANNEL_PRESSURE,
  MIDI_KIND_PITCH_BEND,
  MIDI_KIND_SYSTEM_EXCLUSIVE,
  MIDI_KIND_MIDI_TIME_CODE,
  MIDI_KIND_SONG_POSITION,
  MIDI_KIND_SONG_SELECT,
  MIDI_KIND_TUNE_REQUEST,
  MIDI_KIND_CLOCK,
  MIDI_KIND_START,
  MIDI_KIND_CONTINUE,
  MIDI_KIND_STOP,
  MIDI_KIND_ACTIVE_SENSE,
  MIDI_KIND_SYSTEM_RESET,
  MIDI_KIND_COUNT
};
struct MidiFilterRule {
  bool enabled;
  MidiMessageKind kind;
  int8_t channel;   // -1 = ALL, 0..15 = Ch1..Ch16
};
struct MidiMapperRule {
  bool enabled;
  MidiMessageKind srcKind;
  int8_t  srcChannel;   // -1 = ALL
  int16_t srcData1;     // -1 = ANY
  int16_t srcMin;
  int16_t srcMax;
  MidiMessageKind dstKind;
  int8_t  dstChannel;   // -1 = KEEP
  int16_t dstData1;     // -1 = KEEP
  int16_t dstMin;
  int16_t dstMax;
};
struct MidiMessage {
  uint8_t bytes[3];
  uint8_t length;
  MidiMessageKind kind;
  bool hasChannel;
  int8_t channel;
};
static const int MAX_FILTER_RULES = 8;
static const int MAX_MAPPER_RULES = 8;

struct Rect { int x, y, w, h; };
struct ValueBtn { Rect r; int8_t value; char label[5]; };
struct KeyBtn   { Rect r; int8_t keyValue; const char* keyName; bool isBlackKey; };
struct SeqSlot  { Rect up, slot, down; };
struct PianoKeyGeom { Rect r; uint8_t note; bool isBlackKey; };
struct SmfKeyGeom { Rect r; bool isBlackKey; };

// ==== State ====
static AppMode currentApp = APP_TRANSPOSE;
static DisplayMode currentMode = DIRECT_MODE;
static PlayMode currentPlay = PLAY_SRC;
static TransposeRange transposeRange = RANGE_MINUS5_TO_6;
static volatile int8_t transposeValue = 0;
static bool transposeButtonsOn[12] = {false};

static int8_t selectedMajorKey = -1;
static int8_t selectedMinorKey = -1;
static bool   majorUpperTranspose = false;
static bool   minorUpperTranspose = false;

static bool allNotesOffEnabled = false;
static unsigned long midiInCount = 0;
static unsigned long midiOutCount = 0;
// Counts that exclude heartbeat traffic (0xFE Active Sense, 0xF8 Clock) —
// used to drive the top-right flash stripes so they only blink for real
// musical activity, not for the constant ticking of clock / active-sense.
// (Macro instead of static inline so the Arduino auto-prototype generator
// keeps its forward-decl insertion below the existing typedef block.)
static unsigned long midiInRealCount  = 0;
static unsigned long midiOutRealCount = 0;
#define isHeartbeatByte(b) ((b) == 0xF8 || (b) == 0xFE)
static MidiInputSource midiInputSource = MIDI_INPUT_MIX;

// MIDI Manager state
static MidiManagePage     midiManagePage      = MIDI_PAGE_FILTER;
static MidiMapperEditPage midiMapperEditPage  = MAPPER_PAGE_SOURCE;
static bool          midiFilterBypass     = true;
static bool          midiMapperBypass     = true;
static MidiFilterRule midiFilterRules[MAX_FILTER_RULES];
static MidiMapperRule midiMapperRules[MAX_MAPPER_RULES];
static int  midiFilterRuleCount   = 1;
static int  midiMapperRuleCount   = 1;
static int  midiSelectedFilterRule = 0;
static int  midiSelectedMapperRule = 0;
static DisplayMode lastTransposeMode = DIRECT_MODE;

// ==== Storage ====
static bool storageReady = false;
static bool midiFsReady = false;
static SdFs midiSd;

// ==== 88-key tracking (A0=21 .. C8=108) ====
#define PIANO_LOWEST_NOTE 21
#define PIANO_HIGHEST_NOTE 108
#define PIANO_KEY_COUNT   88
#define MIDI_CHANNEL_COUNT 16
#define TRACKED_NOTE_STATE_COUNT (PIANO_KEY_COUNT * MIDI_CHANNEL_COUNT)
struct NoteState {
  bool isActive;
  int8_t originalTranspose;
  uint8_t channel;
  uint8_t velocity;
};
static NoteState currentNoteStates[TRACKED_NOTE_STATE_COUNT];
static NoteState savedNoteStates  [TRACKED_NOTE_STATE_COUNT];
static PianoKeyGeom pianoKeys[PIANO_KEY_COUNT];

// ==== Sequence mode ====
#define SEQ_PATTERN_COUNT 16
#define SEQ_STEP_COUNT     6
#define SEQ_FILE          "/seq_data.dat"
static int8_t seqPatterns[SEQ_PATTERN_COUNT][SEQ_STEP_COUNT];
static int seqCurrentPattern = 0;
static int seqCurrentStep    = 0;
enum SeqSaveUiState { SEQ_SAVE_UI_IDLE, SEQ_SAVE_UI_PENDING, SEQ_SAVE_UI_OK, SEQ_SAVE_UI_ERR };
static bool g_seqSavePending = false;
static SeqSaveUiState g_seqSaveUiState = SEQ_SAVE_UI_IDLE;
static uint32_t g_seqSaveUiUntil = 0;

// ==== SMF Player ====
static constexpr const char* SMF_FOLDER = "/smf";
// Playlists are kept in fixed-size, statically-allocated char arrays so the
// heap is never touched once boot finishes. Using `String` / `vector<String>`
// here was a slow heap-fragmentation trap: every rescan freed the old string
// blocks and allocated new ones at fresh addresses, eventually leaving only
// holes too small for a contiguous allocation, which manifested as a crash
// hours into a session.
static const int   SMF_MAX_FILES     = 1024;
static const int   PLAYLIST_PATH_MAX = 128;  // "/smf/" + filename + NUL
static MD_MIDIFile smf;
// Allocated lazily in PSRAM on the first scan (1024 × 128 = 128 KB per
// playlist would not fit in internal DRAM). Holds only the currently
// browsed directory; re-filled on every folder navigation.
static char        (*smfPlaylist)[PLAYLIST_PATH_MAX] = nullptr;
static int         smfPlaylistCount = 0;
static int smfCurrentTrack = 0;
static int smfListScroll = 0;
// Subfolder navigation. smfCurrentDir is the directory currently shown in the
// list (e.g. "/smf" at the root, "/smf/album1" inside a subfolder). Files are
// stored as full paths; directory entries carry a trailing "/"; the synthetic
// parent entry is the literal string "..".
static char smfCurrentDir[PLAYLIST_PATH_MAX] = "";
static bool smfLoaded = false;
static bool smfPlaying = false;
static bool smfLoop = false;
static uint32_t smfPlaybackStartMs = 0;
static uint32_t smfPausedElapsedMs = 0;
static uint8_t smfActiveVelocity[128] = {};
static uint8_t smfChannelVelocity[16][128] = {};
static SmfKeyGeom smfKeyGeom[16][128];
static bool smfMonitorGeometryReady = false;
static bool smfMonitorDirtyAll = false;
static bool smfMonitorKeyDirty[16][128] = {};
static char smfCurrentName[128] = {};

// ==== MP3 Player ====
static constexpr const char* MP3_FOLDER = "/mp3";
static const int MP3_MAX_FILES = 1024;
static char      (*mp3Playlist)[PLAYLIST_PATH_MAX] = nullptr;  // PSRAM, see smfPlaylist
static int       mp3PlaylistCount = 0;
static int mp3CurrentTrack = 0;
static int mp3ListScroll = 0;
static char mp3CurrentDir[PLAYLIST_PATH_MAX] = "";  // see smfCurrentDir comment
static bool mp3Playing = false;
static uint32_t mp3PlaybackStartMs = 0;
static int mp3Volume = 220;  // M5.Speaker range is 0..255
static float mp3CassetteAngle = 0.0f;
static uint32_t mp3LastAnimMs = 0;
static bool mp3StaticDirty = true;
static bool mp3VisualDirty = true;
static AudioOutputM5Speaker mp3Out(&M5.Speaker, 0);
static AudioGeneratorMP3 mp3Decoder;
static AudioFileSourceFS* mp3File = nullptr;
static AudioFileSourceID3* mp3Id3 = nullptr;
static char mp3Title[128] = {};
static char mp3Artist[128] = {};
static char mp3CurrentName[128] = {};

// ==== Device-wide configuration (persisted to /config.json on SD) ====
// The struct is defined here, near the other top-level state, so functions
// declared later in the .ino (which the Arduino preprocessor can't auto-
// forward) can still reference g_config at compile time. The save/load/apply
// helpers themselves live further down with the rest of the SD I/O code.
struct DeviceConfig {
  char defaultApp[16];
  char defaultTransposeMode[16];
  int  initialTranspose;
  int  transposeBase;
  bool initialAllNotesOff;
  bool initialFilterBypass;
  bool initialMapperBypass;
  char transposeRange[12];
  char midiInputSource[12];
  bool majorUpperTranspose;
  bool showSplash;
  // Reset behaviour: SAM2695 retains its program/CC state across reboots, so
  // unconditionally blasting GS Reset at boot can stomp the user's last setup.
  // Both flags default such that startup is silent (off) but SMF playback
  // still resets the synth before each track (existing behaviour).
  bool startupGSReset;       // boot: send Roland GS Reset SysEx
  bool smfStartGSReset;      // SMF play start: send Roland GS Reset SysEx
  // SRC playback mode: per-channel synth state restored at boot.
  uint8_t srcInitChannel;    // 1..16 (UI is 1-based)
  uint8_t srcInitProgram;    // 0..127 (GM program number for melodic chs)
  uint8_t srcInitVolume;     // 0..127 (CC #7)
  bool    srcAutoChannel;    // follow incoming MIDI channel automatically
};
static DeviceConfig g_config;

// ==== SRC (Sound source / instrument selection) ====
// Per-channel state for the M5 Unit MIDI (SAM2695) GM synth. Mirrors the
// state model used by M5Core2-MIDIXposeFilBTUM but extends it to 16 channels
// so the user can drive any channel from the Tab UI. Channel 10 (index 9)
// uses the drum-kit catalog; everything else uses the GM melodic catalog.
static const int SRC_CH_COUNT = 16;
static const int SRC_DRUM_CHANNEL = 9;     // 0-based MIDI channel 10
static uint8_t  srcChannel = 0;            // active UI channel (0..15)
static uint8_t  srcProgram[SRC_CH_COUNT];  // last sent program per channel
static uint8_t  srcVolume[SRC_CH_COUNT];   // CC #7 cached value
static uint16_t srcPitchBend[SRC_CH_COUNT];// 14-bit, 8192 = center
static bool     srcSustain[SRC_CH_COUNT];  // CC #64 cached value
static bool     srcAutoFollow = true;      // follow incoming MIDI channel
static int      srcListScroll = 0;         // first item index in the picker
static bool     srcDirtyAll = true;        // force a full SRC redraw next tick

// ==== Layout (computed in computeLayout()) ====
static Rect headerArea;       // top banner
static Rect toolbarArea;      // mode tabs + aux
static Rect contentArea;      // main area
static Rect navArea;          // bottom PREV / NEXT

static Rect appTab[3];
static const char* appName[3] = { "XPOSE", "MSG", "PLAY" };
// XPOSE app sub-mode tabs (4)
static Rect modeTab[4];
static const char* modeName[4] = { "DIR.", "KEY", "INST.", "SEQ." };
// MIDI app sub-tab buttons (FILTER / MAPPER / BYPASS|ACTIVE)
static Rect midiAppTabFilter;
static Rect midiAppTabMapper;
static Rect midiAppBypassBtn;
static Rect midiInputSourceBtn;
static Rect baseEntryBtn;     // header right: enter BASE_SET_MODE
static Rect cfgEntryBtn;      // header right: enter CONFIG_EDIT_MODE
// PLAY app sub-mode tabs (SRC / SMF / MP3)
static Rect playTab[3];
static const char* playName[3] = { "SRC", "SMF", "MP3" };
static Rect btnAllOff;        // aux
static Rect btnRange;         // aux (DIRECT range cycle / KEY upper/lower swap)
static Rect btnPrev, btnNext; // bottom-nav transpose prev/next
static Rect smfBtnPrev, smfBtnPlay, smfBtnNext, smfBtnLoop;
static Rect mp3BtnPrev, mp3BtnPlay, mp3BtnNext, mp3BtnVolDown, mp3BtnVolUp;
static Rect srcBtnGm, srcBtnGs, srcBtnInit, srcBtnAuto;

static ValueBtn directBtns[12];
static KeyBtn   majorKeys[12];
static KeyBtn   minorKeys[12];
static ValueBtn instantBtns[8];
static Rect     instantZero;

static SeqSlot  seqSteps[SEQ_STEP_COUNT];
static Rect     seqPatLeft, seqPatRight, seqSave;
static Rect     smfListArea, smfInfoArea, smfPianoArea;
static Rect     smfListPageUpBtn, smfListUpBtn, smfListDownBtn, smfListPageDownBtn;
static Rect     mp3ListArea, mp3InfoArea, mp3VisualArea;
static Rect     mp3ListPageUpBtn, mp3ListUpBtn, mp3ListDownBtn, mp3ListPageDownBtn;

// SRC layout rectangles (computed in computeLayout()).
static Rect srcChannelRow;       // strip showing CH 01..16 cells
static Rect srcChannelCells[16];
static Rect srcProgramHeader;    // "PRG:nnn  Name"  — also tappable to open picker
static Rect srcPrgDownBtn, srcPrgUpBtn;
static Rect srcListArea;         // outer container for label / piano roll / keyboard
static Rect srcLabelArea;        // top-of-srcListArea band hosting the channel label
static Rect srcRollArea;         // piano-roll history strip above the keyboard
static Rect srcKeyboardArea;     // 88-key live monitor at the bottom of srcListArea
static Rect srcListUpBtn, srcListDownBtn;  // (legacy, zeroed; kept to avoid dangling refs)
static Rect srcVolDownBtn, srcVolUpBtn, srcVolLabel;
static Rect srcPbDownBtn,  srcPbUpBtn,  srcPbLabel;
static Rect srcSusBtn;

// Per-channel live note state for the SRC active-channel keyboard. Updated
// from handleParsedMidiMessage so any MIDI passing through the pipeline
// shows up on the keyboard while in SRC mode. Channel is 0..15.
static uint8_t srcKeyVel[16][128]   = {};
static bool    srcKeyDirty[16][128] = {};
static bool    srcKeyAllDirty       = true;
struct SrcKeyGeom { Rect r; bool isBlack; };
static SrcKeyGeom srcKeyGeom[128];   // pre-computed rectangles for notes 21..108 (88 keys)
static bool       srcKeyGeomReady = false;
// Notes outside the 88-key range (21..108) get drawn into the leftmost white
// key as a tint for visibility, but we mostly ignore them.
static const int  SRC_KEY_LO = 21;   // A0
static const int  SRC_KEY_HI = 108;  // C8

// Piano-roll history. Captures recent note events on the active channel so
// we can render scrolling vertical bars above the keyboard. The roll spans
// SRC_ROLL_T_MS milliseconds of wall time, with the present moment at the
// bottom edge and older events scrolling upward off the top.
static const uint32_t SRC_ROLL_T_MS = 3000;   // 3 s window (faster scroll)
static const int      SRC_ROLL_HISTORY = 96;  // ring buffer size for finished notes
struct SrcRollEvent { uint8_t note; uint8_t vel; uint8_t channel; uint32_t startMs; uint32_t endMs; };
static SrcRollEvent srcRollHist[SRC_ROLL_HISTORY];
static int srcRollHistCount = 0;     // total writes (mod SRC_ROLL_HISTORY for ring)
static uint32_t srcNoteStartMs[16][128];  // 0 = note not currently held

// SRC fullscreen instrument picker overlay state. Opens when the user taps
// the PRG name banner; covers the entire screen with a paged grid of all
// 128 GM instruments (or the 9 drum kits when active channel is Ch10).
static bool g_srcPickerOpen = false;
static int  g_srcPickerPage = 0;
static const int SRC_PICKER_COLS    = 4;
static const int SRC_PICKER_ROWS    = 8;
static const int SRC_PICKER_PERPAGE = SRC_PICKER_COLS * SRC_PICKER_ROWS;  // 32
static Rect srcPickerCells[SRC_PICKER_PERPAGE];
static Rect srcPickerPrevBtn, srcPickerNextBtn, srcPickerCloseBtn;

// Touch input uses M5Unified's edge-triggered wasPressed(), so no manual latch.

// ==== Redraw flags ====
static bool needFullRedraw    = true;
static bool needPartialUpdate = false;
static bool midiManageDirty   = false;
static uint32_t g_lastMidiInputAt = 0;
// ESP-IDF USB-host based MIDI input. Uses the same libusb.a that ships with
// arduino-esp32 3.3.x for ESP32-P4. The PHY is brought up automatically by
// usb_host_install(). See USB MIDI handling below.
static volatile bool   g_usbHostReady     = false;  // host stack installed
static volatile bool   g_usbMidiMounted   = false;  // MIDI interface claimed and IN xfer running
static volatile uint8_t g_usbMidiActiveIndex = 0xFF; // legacy field, kept for status text
static volatile uint8_t g_usbMidiCableCount  = 0;    // virtual cables on IN endpoint
static volatile uint8_t g_usbMidiInEpAddr    = 0;    // address of the IN endpoint we polled
static volatile uint8_t g_usbMidiClaimedItf  = 0xFF; // interface number we claimed
static usb_host_client_handle_t g_usbClient = nullptr;
static usb_device_handle_t      g_usbDevice = nullptr;
// Multiple IN transfers in flight at once. Single transfer left a window
// while we processed a completed buffer where no transfer was queued; the
// keyboard's data on the next USB poll had nowhere to land and got dropped
// at the USB host driver layer (never reached our ring, hence usb_drop=0
// even though notes were lost). Aggressive chord trills can saturate
// short-lived bursts; 16 × 64-byte buffers ≈ 1 KB of always-queued receive
// capacity gives the IDF host driver plenty of room while a callback is
// processing one of them.
static const int                USB_MIDI_IN_XFERS = 16;
static usb_transfer_t*          g_usbInXfers[USB_MIDI_IN_XFERS] = {};
// Drop counter for transfers we couldn't re-submit (usually transient, but
// without a counter we can't tell). Surfaced via STATUS too.
static volatile uint32_t        g_usbInResubmitFails = 0;
static TaskHandle_t             g_usbTask   = nullptr;

// Lock-free-ish ring buffer the USB task uses to hand raw MIDI bytes to the
// main loop. processMIDIByte() touches Serial2 and global UI state, so it
// must run on the loop task only.
// Generous USB MIDI ring. 1024 was too small once a heavy chord burst
// arrived faster than the consumer could drain (USB Full-Speed bursts are
// far quicker than 31.25 kbaud MIDI OUT). 8 KB ≈ 2.6 s of headroom.
static constexpr size_t USB_MIDI_RING_SIZE = 8192;
static uint8_t  g_usbMidiRing[USB_MIDI_RING_SIZE];
static volatile uint32_t g_usbMidiRingHead = 0;  // written by USB task
static volatile uint32_t g_usbMidiRingTail = 0;  // read by loop
static volatile uint32_t g_usbMidiRingDropCount = 0;  // bytes dropped on full-ring
static portMUX_TYPE g_usbMidiRingMux = portMUX_INITIALIZER_UNLOCKED;

enum CyclerKind : uint8_t;

// ==== Forward decls ====
static void computeLayout();
static void initDirect();
static void initKeys();
static void initInstant();
static void initSequence();
static void initPianoKeys(const Rect& area);
static void drawInterface();
static void drawSplash();
static void drawHeader();
static void drawAppTabs();
static void drawToolbar();
static void drawNav();
static void drawDirect();
static void drawKey();
static void drawInstant();
static void drawSequence();
static void drawMidiManage();
static void processMidiManageTouch(const m5::Touch_Class::touch_detail_t& td);
static void drawSmf();
static void drawMp3();
static void drawSrc();
static void handleSrcTouch(int x, int y);
static void drawSrcPicker();
static void handleSrcPickerTouch(int x, int y);
static void srcPickerOpen();
static void srcPickerClose();
static void drawHeaderStatusApp();
static void drawHeaderRightButtons();
static void drawMidiActivityLines();
static void tickMidiActivityLines();
static void invalidateMidiActivityStripes();
static void drawToolbarApp();
static void drawNavApp();
static void updateStatusArea();
static void drawMidiInputSourceBtn();
static const char* getHeaderTitle();
static void handleTouch();
static void setCurrentApp(AppMode app);
static bool setMidiInputSource(MidiInputSource source);
static bool ensureStorage();
static void scanSmfFiles();
static bool loadSmfTrack(int index);
static void playSmf(bool sendReset = true);
static void stopSmf();
static void closeSmf();
static void processSmf();
static void resetSmfKeyboard();
static void smfMidiEventHandler(midi_event* pev);
static void smfSysexEventHandler(sysex_event* pev);
static void smfMetaEventHandler(const meta_event* mev);
static void resetMidiInputParser();
static bool startUsbHost();
static void serviceUsbHost(uint32_t now);
static void processMidiInput();
static void processUsbMidiInput();
static size_t getMidiInputAvailable();
static void scanMp3Files();
static bool startMp3Track(int index);
static void stopMp3();
static void processMp3();
static void mp3MetadataCallback(void* cbData, const char* type, bool isUnicode, const char* str);
static void drawPianoKeyboard(const Rect& area, const uint8_t* velocityMap,
                              const char* title, const char* subtitle);
static void drawSmfChannelKeyboard(const Rect& area);
static void initSmfMonitorGeometry(const Rect& area);
static void invalidateSmfMonitorAll();
static void invalidateSmfMonitorKey(uint8_t channel, uint8_t note);
static void flushSmfMonitorDirty(const Rect& area);
static void drawSmfMonitorBase(const Rect& area);
static void drawSmfMonitorKey(uint8_t channel, uint8_t note);
static void handleSmfTouch(int x, int y);
static void handleMp3Touch(int x, int y);
static void processMidiInput();
static void sendAllNotesOff();
static void sendGSReset();
static int8_t clampTranspose(int8_t v);
static int getTrackedNoteStateIndex(uint8_t midiNote, uint8_t channel);
static void clearTrackedNoteStates();
static bool isMidiInputIdle(uint32_t now);
static void processDeferredStorageTasks(uint32_t now);
static void refreshSmfAggregateNote(uint8_t note);
static void clearSmfChannelNotes(uint8_t channel);
static void handleTransposeChange(int8_t newValue);
static bool saveSequencesToSD();
static bool loadSequencesFromSD();
static void setCurrentTransposeButton();
static void shiftTransposeBy(int dir);
static void updateDirectButtonLabels();

// Device configuration / overlay-mode forward decls.
static void setDefaultConfig();
static bool loadDeviceConfigFromSD();
static bool saveDeviceConfigToSD();
static void applyDeviceConfig();
static void enterConfigEditMode();
static void exitConfigEditMode(bool save, bool apply);
static void drawConfigEditMode();
static void processConfigEditTouch(int tx, int ty);
static void enterBaseSetMode();
static void exitBaseSetMode();
static void drawBaseSetMode();
static void processBaseSetTouch(int tx, int ty);
static void setTransposeBase(int newBase);

// =================================================================
//  Button drawing helpers
// =================================================================
static void drawRectBtn(const Rect& r, uint16_t bg, uint16_t border,
                        const char* label, uint16_t txt, const lgfx::IFont* font)
{
  M5.Display.fillRoundRect(r.x, r.y, r.w, r.h, 10, bg);
  M5.Display.drawRoundRect(r.x, r.y, r.w, r.h, 10, border);
  if (label) {
    M5.Display.setFont(font);
    M5.Display.setTextColor(txt, bg);
    M5.Display.setTextDatum(middle_center);
    M5.Display.drawString(label, r.x + r.w / 2, r.y + r.h / 2);
  }
}

static void drawTriangleBtn(const Rect& r, uint16_t bg, uint16_t border,
                            bool up, uint16_t triColor)
{
  M5.Display.fillRoundRect(r.x, r.y, r.w, r.h, 10, bg);
  M5.Display.drawRoundRect(r.x, r.y, r.w, r.h, 10, border);
  int cx = r.x + r.w / 2;
  int cy = r.y + r.h / 2;
  // Use ~40% of the shorter side so the triangle visually fills the button
  // and reads as a tap target. The old 1/3 ratio left too much whitespace.
  int s = (r.w < r.h ? r.w : r.h) * 2 / 5;
  if (up) {
    M5.Display.fillTriangle(cx, cy - s, cx - s, cy + s, cx + s, cy + s, triColor);
  } else {
    M5.Display.fillTriangle(cx - s, cy - s, cx + s, cy - s, cx, cy + s, triColor);
  }
}

// Two triangles side-by-side horizontally (was vertically stacked).
// Designed for buttons noticeably wider than the single-arrow buttons so the
// "fast" affordance is visually obvious. The triangles use h/3 so they read
// at finger-distance.
static void drawDoubleTriangleBtn(const Rect& r, uint16_t bg, uint16_t border,
                                  bool up, uint16_t triColor)
{
  M5.Display.fillRoundRect(r.x, r.y, r.w, r.h, 10, bg);
  M5.Display.drawRoundRect(r.x, r.y, r.w, r.h, 10, border);
  int cy = r.y + r.h / 2;
  int s = r.h / 3;                      // half-edge of each triangle
  int innerGap = 10;                    // gap between the two triangles
  int pairW = 4 * s + innerGap;         // (2*s)+gap+(2*s)
  int leftCx  = r.x + (r.w - pairW) / 2 + s;
  int rightCx = leftCx + 2 * s + innerGap;
  if (up) {
    M5.Display.fillTriangle(leftCx,  cy - s, leftCx  - s, cy + s, leftCx  + s, cy + s, triColor);
    M5.Display.fillTriangle(rightCx, cy - s, rightCx - s, cy + s, rightCx + s, cy + s, triColor);
  } else {
    M5.Display.fillTriangle(leftCx  - s, cy - s, leftCx  + s, cy - s, leftCx,  cy + s, triColor);
    M5.Display.fillTriangle(rightCx - s, cy - s, rightCx + s, cy - s, rightCx, cy + s, triColor);
  }
}

static void drawTextFit(const Rect& r, const char* text, const lgfx::IFont* font,
                        uint16_t txt, uint16_t bg, int padX = 8)
{
  if (!text) text = "";
  char buf[192];
  strlcpy(buf, text, sizeof(buf));
  M5.Display.setFont(font);
  int maxW = r.w - padX * 2;
  if (maxW < 0) maxW = 0;
  while ((int)M5.Display.textWidth(buf) > maxW && strlen(buf) > 3) {
    size_t len = strlen(buf);
    buf[len - 1] = '\0';
    if (len > 4) {
      buf[len - 4] = '.';
      buf[len - 3] = '.';
      buf[len - 2] = '.';
      buf[len - 1] = '\0';
    }
  }
  M5.Display.setTextColor(txt, bg);
  M5.Display.setTextDatum(middle_left);
  M5.Display.drawString(buf, r.x + padX, r.y + r.h / 2);
}

static bool hit(const Rect& r, int x, int y) {
  return x >= r.x && x < r.x + r.w && y >= r.y && y < r.y + r.h;
}

// =================================================================
//  Layout
// =================================================================
static void computeLayout() {
  SCREEN_W = M5.Display.width();
  SCREEN_H = M5.Display.height();

  headerArea  = { 0,   0,               SCREEN_W, 100 };
  toolbarArea = { 0,   headerArea.h,    SCREEN_W,  96 };
  navArea     = { 0,   SCREEN_H - 100,  SCREEN_W, 100 };
  contentArea = { 0,   headerArea.h + toolbarArea.h,
                  SCREEN_W,
                  SCREEN_H - (headerArea.h + toolbarArea.h + navArea.h) };

  int tabX = 20;
  int tabY = toolbarArea.y + 6;
  int tabH = 62;

  // App tabs (XPOSE / MIDI / PLAY) live in the header now, to the right of
  // the title text. This frees the toolbar entirely for sub-mode tabs and
  // transport buttons.
  int appGap = 8;
  int appW = 120;
  int appTabHX0 = 336;                                       // shifted left to make room
  int appTabHY  = headerArea.y + (headerArea.h - tabH) / 2;  // centred vertically
  for (int i = 0; i < 3; ++i) {
    appTab[i] = { appTabHX0 + i * (appW + appGap), appTabHY, appW, tabH };
  }

  // The toolbar can now start from the left edge.
  int toolStartX = tabX;
  int auxW = 138;
  int auxGap = 8;
  int toolRightX = SCREEN_W - 20 - (auxW * 2 + auxGap);
  // XPOSE: 4 mode tabs DIR./KEY/INST./SEQ.
  int transTabGap = 12;
  int transTabW = (toolRightX - toolStartX - transTabGap * 3) / 4;
  for (int i = 0; i < 4; i++) {
    modeTab[i] = { toolStartX + i * (transTabW + transTabGap), tabY, transTabW, tabH };
  }
  // MIDI: FILTER / MAPPER / BYPASS|ACTIVE buttons across same area
  int midiTabGap = 12;
  int midiTabW = (toolRightX - toolStartX - midiTabGap * 2) / 3;
  midiAppTabFilter = { toolStartX,                              tabY, midiTabW, tabH };
  midiAppTabMapper = { toolStartX + 1 * (midiTabW + midiTabGap), tabY, midiTabW, tabH };
  midiAppBypassBtn = { toolStartX + 2 * (midiTabW + midiTabGap), tabY, midiTabW, tabH };
  // PLAY: 3 mode tabs SRC/SMF/MP3 share the toolbar with transport buttons.
  int playTabW = 130;
  int playTabGap = 8;
  playTab[0] = { toolStartX,                                  tabY, playTabW, tabH };
  playTab[1] = { toolStartX + 1 * (playTabW + playTabGap),    tabY, playTabW, tabH };
  playTab[2] = { toolStartX + 2 * (playTabW + playTabGap),    tabY, playTabW, tabH };
  // Transport buttons start to the right of PLAY tabs (3 tab widths over).
  int transStartX = toolStartX + 3 * (playTabW + playTabGap);

  btnRange  = { SCREEN_W - (auxW * 2 + auxGap + 20), tabY, auxW, tabH };
  btnAllOff = { SCREEN_W - (auxW + 20),              tabY, auxW, tabH };

  int playerGap = 12;
  // Transport / synth-control buttons share the right portion next to the
  // SRC/SMF/MP3 mode tabs and before the AOFF button. Compute available width
  // from transStartX. SMF (4 btns), MP3 (5 btns) and SRC (4 btns) all live in
  // this band — only the active sub-mode is drawn at any moment, so the
  // rectangles can overlap cleanly.
  int transRightX = btnAllOff.x - 12;  // leave some space before AOFF
  int playerAvail = transRightX - transStartX;

  // Common main-button width: PREV/PLAY/NEXT are the same width across SMF
  // and MP3 so the eye doesn't track between sub-modes. SMF's 4 buttons
  // exactly tile the available width; MP3's PREV/PLAY/NEXT match and the
  // VOL-/VOL+ pair splits the remaining slot.
  int mainW = (playerAvail - playerGap * 3) / 4;
  if (mainW < 120) mainW = 120;

  smfBtnPrev = { transStartX,                                tabY, mainW, tabH };
  smfBtnPlay = { smfBtnPrev.x + smfBtnPrev.w + playerGap,     tabY, mainW, tabH };
  smfBtnNext = { smfBtnPlay.x + smfBtnPlay.w + playerGap,     tabY, mainW, tabH };
  smfBtnLoop = { smfBtnNext.x + smfBtnNext.w + playerGap,     tabY, mainW, tabH };

  // MP3: 3 matching main buttons + 2 narrower VOL buttons (4 inter-button gaps).
  int mp3VolW = (playerAvail - mainW * 3 - playerGap * 4) / 2;
  if (mp3VolW < 60) mp3VolW = 60;
  mp3BtnPrev    = { transStartX,                                  tabY, mainW,   tabH };
  mp3BtnPlay    = { mp3BtnPrev.x + mp3BtnPrev.w + playerGap,       tabY, mainW,   tabH };
  mp3BtnNext    = { mp3BtnPlay.x + mp3BtnPlay.w + playerGap,       tabY, mainW,   tabH };
  mp3BtnVolDown = { mp3BtnNext.x + mp3BtnNext.w + playerGap,       tabY, mp3VolW, tabH };
  mp3BtnVolUp   = { mp3BtnVolDown.x + mp3BtnVolDown.w + playerGap, tabY, mp3VolW, tabH };

  // SRC: 4 matching buttons (GMRST / GSRST / INIT / AUTO), same width as SMF
  // main buttons so the toolbar feels consistent when switching sub-modes.
  srcBtnGm   = { transStartX,                              tabY, mainW, tabH };
  srcBtnGs   = { srcBtnGm.x   + srcBtnGm.w   + playerGap,  tabY, mainW, tabH };
  srcBtnInit = { srcBtnGs.x   + srcBtnGs.w   + playerGap,  tabY, mainW, tabH };
  srcBtnAuto = { srcBtnInit.x + srcBtnInit.w + playerGap,  tabY, mainW, tabH };

  // Header right side, left → right:
  //   appTabs (XPOSE / MSG / PLAY) | BASE | CONF | MIX | <status text>
  // BASE / CONF sit immediately right of the PLAY tab (auxW=138 each), MIX
  // sits right of CONF as a narrower selector, and the dynamic status text
  // (K +n / PLAY MM:SS) is right-aligned to SCREEN_W so the most-glanced
  // value lands at the natural reading endpoint.
  baseEntryBtn        = { appTab[2].x + appTab[2].w + 12,                  appTabHY, auxW, tabH };
  cfgEntryBtn         = { baseEntryBtn.x + baseEntryBtn.w + auxGap,        appTabHY, auxW, tabH };
  int mixW = 110;
  midiInputSourceBtn  = { cfgEntryBtn.x + cfgEntryBtn.w + auxGap,          appTabHY, mixW, tabH };

  // Bottom nav: large PREV / NEXT.
  int navPad = 20;
  int navBtnW = (SCREEN_W - navPad * 3) / 2;
  int navBtnH = navArea.h - 20;
  btnPrev = { navArea.x + navPad,                              navArea.y + 10, navBtnW, navBtnH };
  btnNext = { navArea.x + navPad + navBtnW + navPad,           navArea.y + 10, navBtnW, navBtnH };

  int margin = 20;
  int splitGap = 16;
  int rightW = 240;
  int listW = contentArea.w - margin * 2 - splitGap - rightW;
  int rightX = contentArea.x + margin + listW + splitGap;

  // Lists span the full area down to navArea bottom (matches the SMF select UX).
  int listBottom = navArea.y + navArea.h - margin;
  smfListArea  = { contentArea.x + margin, contentArea.y + margin,
                   contentArea.w - margin * 2, listBottom - (contentArea.y + margin) };
  {
    // Single arrows are 90 wide; the double-arrow (page-jump) buttons are
    // 140 wide so they read as distinct, "fast" buttons. Heights are 64 to
    // give a generous tap target.
    const int singleW = 90, doubleW = 140, btnH = 64, btnGap = 8;
    int btnY = smfListArea.y + 8;
    int rightX = smfListArea.x + smfListArea.w - 12;
    // Right-anchored: [▲▲][▲][▼][▼▼]
    smfListPageDownBtn = { rightX - doubleW,                                    btnY, doubleW, btnH };
    smfListDownBtn     = { smfListPageDownBtn.x - btnGap - singleW,             btnY, singleW, btnH };
    smfListUpBtn       = { smfListDownBtn.x     - btnGap - singleW,             btnY, singleW, btnH };
    smfListPageUpBtn   = { smfListUpBtn.x       - btnGap - doubleW,             btnY, doubleW, btnH };
  }
  smfInfoArea  = { rightX, contentArea.y + margin, rightW, 140 };
  int smfKeyboardTop = smfInfoArea.y + smfInfoArea.h + 16;
  int smfKeyboardBottom = navArea.y + navArea.h - margin;
  smfPianoArea = { rightX, smfKeyboardTop,
                   rightW, smfKeyboardBottom - smfKeyboardTop };

  mp3ListArea   = { contentArea.x + margin, contentArea.y + margin,
                    listW, listBottom - (contentArea.y + margin) };
  {
    const int singleW = 90, doubleW = 140, btnH = 64, btnGap = 8;
    int btnY = mp3ListArea.y + 8;
    int rightX = mp3ListArea.x + mp3ListArea.w - 12;
    mp3ListPageDownBtn = { rightX - doubleW,                                    btnY, doubleW, btnH };
    mp3ListDownBtn     = { mp3ListPageDownBtn.x - btnGap - singleW,             btnY, singleW, btnH };
    mp3ListUpBtn       = { mp3ListDownBtn.x     - btnGap - singleW,             btnY, singleW, btnH };
    mp3ListPageUpBtn   = { mp3ListUpBtn.x       - btnGap - doubleW,             btnY, doubleW, btnH };
  }
  mp3InfoArea   = { rightX, contentArea.y + margin, rightW, 118 };
  mp3VisualArea = { rightX, mp3InfoArea.y + mp3InfoArea.h + 12,
                    rightW, listBottom - (mp3InfoArea.y + mp3InfoArea.h + 12) };

  // SRC layout: channel strip on top, program header + ±buttons, then a
  // wide piano roll + a thin live-monitor keyboard. VOL / PB / SUS controls
  // live in navArea below. Margins are kept tight so the piano roll gets the
  // bulk of the vertical space.
  {
    const int srcSideMargin = 20;   // left/right
    const int srcTopMargin  = 6;    // toolbar bottom -> channel strip
    const int srcGap        = 6;    // between rows in this band
    const int srcBottomGap  = 4;    // keyboard bottom -> navArea top
    int sxL = contentArea.x + srcSideMargin;
    int sxR = contentArea.x + contentArea.w - srcSideMargin;
    int sxW = sxR - sxL;
    int sy  = contentArea.y + srcTopMargin;

    // Channel strip: 16 cells × ~50 px wide × 56 px tall
    int chCellH = 56;
    int chCellGap = 4;
    int chCellW = (sxW - chCellGap * 15) / 16;
    srcChannelRow = { sxL, sy, sxW, chCellH };
    for (int i = 0; i < 16; ++i) {
      srcChannelCells[i] = { sxL + i * (chCellW + chCellGap), sy, chCellW, chCellH };
    }
    sy += chCellH + srcGap;

    // Program header row: PRG- (160) | name banner (flex) | PRG+ (160), 80 tall
    int prgBtnW = 160;
    int prgRowH = 80;
    int prgGap = 12;
    srcPrgDownBtn   = { sxL, sy, prgBtnW, prgRowH };
    srcPrgUpBtn     = { sxR - prgBtnW, sy, prgBtnW, prgRowH };
    srcProgramHeader = { srcPrgDownBtn.x + srcPrgDownBtn.w + prgGap, sy,
                         srcPrgUpBtn.x - prgGap - (srcPrgDownBtn.x + srcPrgDownBtn.w + prgGap),
                         prgRowH };
    sy += prgRowH + srcGap;

    // Middle band: piano roll history + 88-key keyboard.
    // Keyboard is intentionally short (≈1/4 of the band) so the piano roll
    // above gets the bulk of the vertical space — that's where the per-note
    // history actually conveys playing dynamics. The "Ch nn live monitor"
    // label is dropped: the channel strip already shows which ch is active,
    // and the colour key on every bar makes the panel self-explanatory.
    int listBottomPx = contentArea.y + contentArea.h - srcBottomGap;
    srcListArea = { sxL, sy, sxW, listBottomPx - sy };
    int kbH       = max(80, srcListArea.h / 4);
    srcLabelArea    = { 0, 0, 0, 0 };  // unused
    srcRollArea     = { srcListArea.x, srcListArea.y,
                        srcListArea.w, srcListArea.h - kbH - 2 };
    srcKeyboardArea = { srcListArea.x, srcListArea.y + srcListArea.h - kbH,
                        srcListArea.w, kbH };
    // Up/down arrow rectangles are kept (zeroed) only so the legacy decls
    // don't dangle; nothing draws or hit-tests them anymore.
    srcListUpBtn   = { 0, 0, 0, 0 };
    srcListDownBtn = { 0, 0, 0, 0 };

    // Bottom row inside navArea: VOL- VOL VOL+ | PB- PB PB+ | SUS — each
    // cluster 1/3 of width.
    int botRowH = navArea.h - 20;
    int yBot = navArea.y + (navArea.h - botRowH) / 2;
    int clusterW = (sxW - 16 * 2) / 3;     // 16 px gaps between clusters
    int subBtnW = 110;
    int volX = sxL;
    int pbX  = sxL + clusterW + 16;
    int susX = pbX + clusterW + 16;
    srcVolDownBtn = { volX, yBot, subBtnW, botRowH };
    srcVolUpBtn   = { volX + clusterW - subBtnW, yBot, subBtnW, botRowH };
    srcVolLabel   = { srcVolDownBtn.x + srcVolDownBtn.w, yBot,
                      srcVolUpBtn.x - (srcVolDownBtn.x + srcVolDownBtn.w), botRowH };
    srcPbDownBtn  = { pbX, yBot, subBtnW, botRowH };
    srcPbUpBtn    = { pbX + clusterW - subBtnW, yBot, subBtnW, botRowH };
    srcPbLabel    = { srcPbDownBtn.x + srcPbDownBtn.w, yBot,
                      srcPbUpBtn.x - (srcPbDownBtn.x + srcPbDownBtn.w), botRowH };
    srcSusBtn     = { susX, yBot, clusterW, botRowH };
  }

  // ---- SRC fullscreen instrument picker layout ----
  // Cover the entire screen with: title strip, paged grid (4×8 = 32 cells),
  // footer with [< PAGE] [PAGE >] [CLOSE]. Cells are big enough to read GM
  // names at FONT_MED.
  {
    int pickerMargin = 24;
    int titleH = 60;
    int footerH = 92;
    int gridX = pickerMargin;
    int gridY = pickerMargin + titleH;
    int gridW = SCREEN_W - pickerMargin * 2;
    int gridH = SCREEN_H - pickerMargin * 2 - titleH - footerH - 16;
    int gap = 10;
    int cellW = (gridW - gap * (SRC_PICKER_COLS - 1)) / SRC_PICKER_COLS;
    int cellH = (gridH - gap * (SRC_PICKER_ROWS - 1)) / SRC_PICKER_ROWS;
    for (int row = 0; row < SRC_PICKER_ROWS; ++row) {
      for (int col = 0; col < SRC_PICKER_COLS; ++col) {
        int idx = row * SRC_PICKER_COLS + col;
        srcPickerCells[idx] = {
          gridX + col * (cellW + gap),
          gridY + row * (cellH + gap),
          cellW,
          cellH
        };
      }
    }
    int footerY = SCREEN_H - pickerMargin - footerH;
    int footerW = (SCREEN_W - pickerMargin * 2 - gap * 2) / 3;
    srcPickerPrevBtn  = { pickerMargin,                                   footerY, footerW, footerH };
    srcPickerNextBtn  = { pickerMargin + footerW + gap,                   footerY, footerW, footerH };
    srcPickerCloseBtn = { pickerMargin + (footerW + gap) * 2,             footerY, footerW, footerH };
  }

  initPianoKeys(smfPianoArea);
  // SRC keyboard geometry depends on srcListArea; force a recompute next draw.
  srcKeyGeomReady = false;
}

// =================================================================
//  Mode-specific init
// =================================================================
static void updateDirectButtonLabels() {
  for (int i = 0; i < 12; i++) {
    int v;
    if (transposeRange == RANGE_0_TO_12)          v = i;
    else if (transposeRange == RANGE_MINUS12_TO_0) v = i - 11;
    else /* RANGE_MINUS5_TO_6 */                   v = i - 5;

    directBtns[i].value = (int8_t)v;
    if (v > 0)      snprintf(directBtns[i].label, sizeof(directBtns[i].label), "+%d", v);
    else            snprintf(directBtns[i].label, sizeof(directBtns[i].label), "%d",  v);
  }
}

static void initDirect() {
  // 4 x 3 grid inside contentArea, with generous margin.
  int cols = 4, rows = 3;
  int margin = 30;
  int gap    = 20;
  int gridW  = contentArea.w - margin * 2;
  int gridH  = contentArea.h - margin * 2;
  int bw = (gridW - gap * (cols - 1)) / cols;
  int bh = (gridH - gap * (rows - 1)) / rows;

  for (int i = 0; i < 12; i++) {
    int r = i / cols, c = i % cols;
    directBtns[i].r = {
      contentArea.x + margin + c * (bw + gap),
      contentArea.y + margin + r * (bh + gap),
      bw, bh
    };
  }
  updateDirectButtonLabels();
}

static void initKeys() {
  static const char* keyNames[] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
  // Piano: 7 white keys per octave.
  int margin = 30;
  int pianoW = contentArea.w - margin * 2;
  int whiteW = pianoW / 7;
  int blackW = (whiteW * 6) / 10;
  int majorY = contentArea.y + 40;
  int whiteH = (contentArea.h - 60) / 2 - 20;
  int blackH = (whiteH * 65) / 100;
  int minorY = majorY + whiteH + 40;

  int baseX = contentArea.x + margin;
  auto layout = [&](KeyBtn* keys, int y) {
    int whiteIdx = 0;
    for (int i = 0; i < 12; i++) {
      keys[i].keyValue = (int8_t)i;
      keys[i].keyName  = keyNames[i];
      keys[i].isBlackKey = (i == 1 || i == 3 || i == 6 || i == 8 || i == 10);
      if (keys[i].isBlackKey) {
        keys[i].r = { baseX + whiteIdx * whiteW - blackW / 2, y, blackW, blackH };
      } else {
        keys[i].r = { baseX + whiteIdx * whiteW,              y, whiteW, whiteH };
        whiteIdx++;
      }
    }
  };
  layout(majorKeys, majorY);
  layout(minorKeys, minorY);
}

static void initInstant() {
  static const int8_t values[8] = { +1, +2, +3, +5, -1, -2, -3, -5 };

  int margin = 40;
  int gap    = 20;
  int gridW  = contentArea.w - margin * 2;
  int bw = (gridW - gap * 3) / 4;

  // Layout is tuned for a ~430 px content area.  Stack = 0-button (100 px)
  // + gap + two rows of 130 px each.
  int zeroH = 100;
  int bh    = 130;

  int zeroW = bw * 2 + gap;
  int zeroX = contentArea.x + (contentArea.w - zeroW) / 2;
  int zeroY = contentArea.y + 15;
  instantZero = { zeroX, zeroY, zeroW, zeroH };

  int rowY = zeroY + zeroH + 20;
  int rowGap = 15;
  for (int i = 0; i < 8; i++) {
    int row = i / 4;
    int col = i % 4;
    instantBtns[i].value = values[i];
    if (values[i] > 0) snprintf(instantBtns[i].label, sizeof(instantBtns[i].label), "+%d", values[i]);
    else               snprintf(instantBtns[i].label, sizeof(instantBtns[i].label), "%d",  values[i]);
    instantBtns[i].r = {
      contentArea.x + margin + col * (bw + gap),
      rowY + row * (bh + rowGap),
      bw, bh
    };
  }
}

static void initSequence() {
  int margin = 20;
  int topY = contentArea.y + 15;
  int topRowH = 74;

  // Pattern left / label / right / save along the top.
  int leftW  = 96;
  int rightW = 96;
  int saveW  = 160;
  int labelW = contentArea.w - (leftW + rightW + saveW + margin * 2 + 30);
  seqPatLeft  = { contentArea.x + margin,                        topY, leftW,  topRowH };
  seqPatRight = { seqPatLeft.x + leftW + 10 + labelW + 10,       topY, rightW, topRowH };
  seqSave     = { seqPatRight.x + rightW + 10,                   topY, saveW,  topRowH };

  // 6 step slots with UP / value / DOWN stacked.  Layout is tuned for a
  // 430 px content area (720 - header - toolbar - nav).
  int slotGap = 15;
  int slotW  = (contentArea.w - margin * 2 - slotGap * 5) / SEQ_STEP_COUNT;
  int stackY = topY + topRowH + 18;
  int upH    = 54;
  int downH  = 54;
  int vGap   = 8;
  int availableH = (contentArea.y + contentArea.h) - stackY - margin;
  int valH   = availableH - upH - downH - vGap * 2;
  if (valH < 120) valH = 120;
  for (int i = 0; i < SEQ_STEP_COUNT; i++) {
    int x = contentArea.x + margin + i * (slotW + slotGap);
    seqSteps[i].up   = { x, stackY,                                    slotW, upH  };
    seqSteps[i].slot = { x, stackY + upH + vGap,                       slotW, valH };
    seqSteps[i].down = { x, stackY + upH + vGap + valH + vGap,         slotW, downH };
  }

  memset(seqPatterns, 0, sizeof(seqPatterns));
}

static void initPianoKeys(const Rect& area) {
  static const uint8_t blackOffsets[] = {1, 3, 6, 8, 10};
  auto isBlack = [&](uint8_t note) {
    uint8_t pitch = note % 12;
    for (uint8_t v : blackOffsets) if (pitch == v) return true;
    return false;
  };

  int whiteCount = 0;
  for (uint8_t note = PIANO_LOWEST_NOTE; note <= PIANO_HIGHEST_NOTE; ++note) {
    if (!isBlack(note)) ++whiteCount;
  }

  int whiteW = area.w / whiteCount;
  int blackW = (whiteW * 65) / 100;
  int blackH = (area.h * 60) / 100;
  int whiteIdx = 0;

  for (int i = 0; i < PIANO_KEY_COUNT; ++i) {
    uint8_t note = PIANO_LOWEST_NOTE + i;
    pianoKeys[i].note = note;
    pianoKeys[i].isBlackKey = isBlack(note);
    if (!pianoKeys[i].isBlackKey) {
      pianoKeys[i].r = { area.x + whiteIdx * whiteW, area.y, whiteW + 1, area.h };
      ++whiteIdx;
    } else {
      pianoKeys[i].r = { area.x + whiteIdx * whiteW - blackW / 2, area.y, blackW, blackH };
    }
  }
}

// =================================================================
//  Draw
// =================================================================
static void drawSplash() {
  SCREEN_W = M5.Display.width();
  SCREEN_H = M5.Display.height();

  M5.Display.fillScreen(TFT_BLACK);

  const int cx = SCREEN_W / 2;
  const int cy = SCREEN_H / 2;

  // Double frame for product feel.
  M5.Display.drawRect(50, 50, SCREEN_W - 100, SCREEN_H - 100, 0x2104);
  M5.Display.drawRect(54, 54, SCREEN_W - 108, SCREEN_H - 108, 0x18C3);

  // Decorative lines flanking the title with accent dots at the corners.
  M5.Display.drawFastHLine(cx - 320, cy - 90, 640, 0x39E7);
  M5.Display.drawFastHLine(cx - 320, cy + 90, 640, 0x39E7);
  M5.Display.fillCircle(cx - 320, cy - 90, 3, COL_ACCENT);
  M5.Display.fillCircle(cx + 320, cy - 90, 3, COL_ACCENT);
  M5.Display.fillCircle(cx - 320, cy + 90, 3, COL_ACCENT);
  M5.Display.fillCircle(cx + 320, cy + 90, 3, COL_ACCENT);

  // Subtitle under the brand line.
  M5.Display.setFont(FONT_MED);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextColor(0x8C71, TFT_BLACK);
  M5.Display.drawString("MIDI Transposer  -  Player", cx, cy + 40);

  // Footer.
  M5.Display.setFont(FONT_TINY);
  M5.Display.setTextColor(0x6B4D, TFT_BLACK);
  M5.Display.drawString("for M5Stack Tab5", cx, cy + 170);

  // Progress bar frame.
  const int pbW = 520;
  const int pbH = 4;
  const int pbX = cx - pbW / 2;
  const int pbY = cy + 140;
  M5.Display.drawRoundRect(pbX, pbY, pbW, pbH, 2, 0x4208);

  const uint32_t totalMs = 3000;
  const uint32_t startMs = millis();

  M5.Display.setFont(FONT_TITLE);
  M5.Display.setTextDatum(middle_center);

  int lastFill = -1;
  uint16_t lastTitleCol = 0xFFFE;  // unlikely value, forces first draw

  while (true) {
    uint32_t e = millis() - startMs;
    if (e >= totalMs) break;

    // Title fades in over the first 700ms, holds, then fades out the last 300ms.
    uint8_t lum;
    if (e < 700)                lum = (uint8_t)((uint32_t)255 * e / 700);
    else if (e > totalMs - 300) lum = (uint8_t)((uint32_t)255 * (totalMs - e) / 300);
    else                        lum = 255;

    uint16_t tcol = M5.Display.color565(lum, lum, lum);
    if (tcol != lastTitleCol) {
      M5.Display.setTextColor(tcol, TFT_BLACK);
      M5.Display.drawString("OWAMIDICON-Tab", cx, cy - 30);
      lastTitleCol = tcol;
    }

    int fill = (int)((uint64_t)(pbW - 4) * e / totalMs);
    if (fill != lastFill) {
      M5.Display.fillRoundRect(pbX + 2, pbY + 2, fill, pbH - 4, 1, COL_ACCENT);
      lastFill = fill;
    }
    delay(16);
  }
}

static void drawHeader() {
  M5.Display.fillRect(headerArea.x, headerArea.y, headerArea.w, headerArea.h, COL_PANEL);

  M5.Display.setFont(FONT_TITLE);
  M5.Display.setTextColor(COL_TITLE, COL_PANEL);
  M5.Display.setTextDatum(middle_left);
  M5.Display.drawString(getHeaderTitle(), 30, headerArea.y + headerArea.h / 2);

  drawAppTabs();
  drawHeaderRightButtons();
  drawHeaderStatusApp();
  // Full header was just cleared — force the stripes to repaint regardless
  // of the colour cache.
  invalidateMidiActivityStripes();
  drawMidiActivityLines();
}

// BASE / CONF / MIX live in the right strip but only change state on
// overlay enter/exit (BASE_SET_MODE / CONFIG_EDIT_MODE) or USB MIDI
// connect/disconnect — all of which already trigger needFullRedraw. So we
// only paint these from the full-redraw path; partial updates leave them
// alone, which prevents the ~5 Hz SMF-playback refresh from making them
// pulse.
static void drawHeaderRightButtons() {
  bool baseOn = (currentMode == BASE_SET_MODE);
  bool cfgOn  = (currentMode == CONFIG_EDIT_MODE);
  drawRectBtn(baseEntryBtn,
              baseOn ? COL_BTN_HI2 : COL_BTN, COL_BTN_BDR, "BASE",
              baseOn ? COL_BTN_TXT_HI : COL_BTN_TXT, FONT_MED);
  drawRectBtn(cfgEntryBtn,
              cfgOn ? COL_BTN_HI2 : COL_BTN, COL_BTN_BDR, "CONF",
              cfgOn ? COL_BTN_TXT_HI : COL_BTN_TXT, FONT_MED);
  drawMidiInputSourceBtn();
}

static void updateStatusArea() {
  // Order in the right strip: [BASE] [CONF] [MIX] [K +n text right-aligned].
  // Partial updates only need to repaint the text strip (right of MIX) —
  // BASE / CONF / MIX themselves only change on overlay enter/exit, which
  // takes the needFullRedraw path. Clearing only the text strip keeps the
  // ~5 Hz partial refreshes from making BASE/CONF/MIX pulse and from
  // touching the y=2..7 MIDI activity stripes at all.
  int textRight = SCREEN_W - 24;
  int textX = midiInputSourceBtn.x + midiInputSourceBtn.w + 4;
  int textW = SCREEN_W - textX;
  M5.Display.fillRect(textX, headerArea.y + 10, textW, headerArea.h - 10, COL_PANEL);

  // "K %+d" readout — right-aligned all the way to SCREEN_W so the most-
  // glanced value lives at the natural reading endpoint. IN/OUT numeric
  // counters are not drawn here (the top-edge flash stripes show activity
  // and STATUS USB-serial reports the numbers).
  char buf[16];
  snprintf(buf, sizeof(buf), "K %+d", (int)transposeValue);
  M5.Display.setFont(FONT_LARGE);
  M5.Display.setTextColor(COL_VALUE, COL_PANEL);
  M5.Display.setTextDatum(middle_right);
  M5.Display.drawString(buf, textRight, headerArea.y + headerArea.h / 2);
}

static const char* getHeaderTitle() {
  if (currentApp == APP_MIDI) {
    return (midiManagePage == MIDI_PAGE_FILTER) ? "Filter" : "Mapper";
  } else if ((currentApp == APP_PLAY && currentPlay == PLAY_SRC)) {
    return "SND Source";
  } else if ((currentApp == APP_PLAY && currentPlay == PLAY_SMF)) {
    return "SMF Player";
  } else if ((currentApp == APP_PLAY && currentPlay == PLAY_MP3)) {
    return "MP3 Player";
  }
  return "Transposer";
}

static const char* getMidiInputSourceLabel() {
  switch (midiInputSource) {
    case MIDI_INPUT_USB: return "USBIN";
    case MIDI_INPUT_MIX: return "MIX";
    default:             return "MIDIIN";
  }
}

static void drawMidiInputSourceBtn() {
  const bool usbSelected = (midiInputSource == MIDI_INPUT_USB);
  const bool mixSelected = (midiInputSource == MIDI_INPUT_MIX);
  uint16_t bg = COL_BTN;
  uint16_t txt = COL_BTN_TXT;
  if (mixSelected) {
    bg = COL_ACCENT;
    txt = COL_BTN_TXT_HI;
  } else if (usbSelected) {
    bg = g_usbMidiMounted ? COL_BTN_HI : COL_BTN_HI2;
    txt = g_usbMidiMounted ? COL_BTN_TXT_HI : COL_BTN_TXT;
  }
  drawRectBtn(midiInputSourceBtn, bg, COL_BTN_BDR, getMidiInputSourceLabel(), txt, FONT_MED);
}

static void drawToolbar() {
  M5.Display.fillRect(toolbarArea.x, toolbarArea.y, toolbarArea.w, toolbarArea.h, COL_BG);

  // XPOSE sub-mode tabs (DIR./KEY/INST./SEQ.)
  for (int i = 0; i < 4; i++) {
    bool on = (currentMode == (DisplayMode)i);
    uint16_t bg  = on ? COL_BTN_HI : COL_BTN;
    uint16_t txt = on ? COL_BTN_TXT_HI : COL_BTN_TXT;
    drawRectBtn(modeTab[i], bg, COL_BTN_BDR, modeName[i], txt, FONT_MED);
  }

  // Range button — label depends on mode.
  const char* rangeLabel = "RANGE";
  if (currentMode == DIRECT_MODE) {
    if      (transposeRange == RANGE_0_TO_12)      rangeLabel = "0 .. +11";
    else if (transposeRange == RANGE_MINUS12_TO_0) rangeLabel = "-11 .. 0";
    else                                            rangeLabel = "-5 .. +6";
  } else if (currentMode == KEY_MODE) {
    rangeLabel = majorUpperTranspose ? "KEY: UPPER" : "KEY: NORMAL";
  } else {
    rangeLabel = "--";
  }
  bool rangeEnabled = (currentMode == DIRECT_MODE || currentMode == KEY_MODE);
  drawRectBtn(btnRange,
              rangeEnabled ? COL_BTN : COL_PANEL,
              COL_BTN_BDR,
              rangeLabel,
              rangeEnabled ? COL_BTN_TXT : COL_MUTED,
              FONT_MED);

  // All-notes-off toggle.
  drawRectBtn(btnAllOff,
              allNotesOffEnabled ? COL_BTN_HI : COL_DANGER,
              COL_BTN_BDR,
              "AOFF",
              allNotesOffEnabled ? COL_BTN_TXT_HI : COL_BTN_TXT,
              FONT_MED);
}

static void drawNav() {
  M5.Display.fillRect(navArea.x, navArea.y, navArea.w, navArea.h, COL_BG);
  drawRectBtn(btnPrev, TFT_BLUE, COL_BTN_BDR, "< PREV", COL_BTN_TXT, FONT_LARGE);
  drawRectBtn(btnNext, TFT_BLUE, COL_BTN_BDR, "NEXT >", COL_BTN_TXT, FONT_LARGE);
}

static void drawHeaderStatusApp() {
  if (currentApp == APP_TRANSPOSE) {
    updateStatusArea();
    return;
  }

  // PLAY app's elapsed-time readout lives at the right edge — same slot the
  // XPOSE app's "K +n" uses — so the most-glanced value is always rightmost.
  int valueX = SCREEN_W - 24;
  int y  = headerArea.y + 10;
  int h  = headerArea.h - 20;
  // Partial refresh only repaints the text strip right of MIX (see
  // updateStatusArea() comment). BASE/CONF/MIX state changes flow through
  // needFullRedraw, so they don't need to be repainted here.
  int textX = midiInputSourceBtn.x + midiInputSourceBtn.w + 4;
  int textW = SCREEN_W - textX;
  M5.Display.fillRect(textX, headerArea.y + 10, textW, headerArea.h - 10, COL_PANEL);

  // Two-line PLAY/STOP readout: small label on top, larger MM:SS below.
  // Single-line "PLAY 00:27" overflowed left into the MIX button on the
  // narrow ~150 px strip available right of MIX, so we stack instead.
  int labelY = headerArea.y + headerArea.h / 2 - 18;
  int timeY  = headerArea.y + headerArea.h / 2 + 18;
  if (currentApp == APP_PLAY && currentPlay == PLAY_SRC) {
    char chBuf[16];
    snprintf(chBuf, sizeof(chBuf), "CH:%02u", (unsigned)(srcChannel + 1));
    char prgBuf[16];
    if (srcChannel == SRC_DRUM_CHANNEL) {
      snprintf(prgBuf, sizeof(prgBuf), "DRUM:%03u", (unsigned)srcProgram[srcChannel]);
    } else {
      snprintf(prgBuf, sizeof(prgBuf), "PRG:%03u", (unsigned)(srcProgram[srcChannel] + 1));
    }
    uint16_t col = srcAutoFollow ? COL_BTN_HI : COL_ACCENT;
    M5.Display.setTextColor(col, COL_PANEL);
    M5.Display.setTextDatum(middle_right);
    // Both lines use FONT_SMALL so the wider "PRG:nnn" doesn't overflow the
    // narrow strip between the MIX button and the right edge.
    M5.Display.setFont(FONT_SMALL);
    M5.Display.drawString(chBuf,  valueX, labelY);
    M5.Display.drawString(prgBuf, valueX, timeY);
  } else if (currentApp == APP_PLAY && currentPlay == PLAY_SMF) {
    uint32_t elapsed = smfPlaying
                     ? smfPausedElapsedMs + (millis() - smfPlaybackStartMs)
                     : smfPausedElapsedMs;
    char timebuf[16];
    snprintf(timebuf, sizeof(timebuf), "%02lu:%02lu",
             (unsigned long)(elapsed / 60000UL),
             (unsigned long)((elapsed / 1000UL) % 60UL));
    uint16_t col = smfPlaying ? COL_BTN_HI : COL_MUTED;
    M5.Display.setTextColor(col, COL_PANEL);
    M5.Display.setTextDatum(middle_right);
    M5.Display.setFont(FONT_SMALL);
    M5.Display.drawString(smfPlaying ? "PLAY" : "STOP", valueX, labelY);
    M5.Display.setFont(FONT_MED);
    M5.Display.drawString(timebuf, valueX, timeY);
  } else if (currentApp == APP_PLAY && currentPlay == PLAY_MP3) {
    uint32_t elapsed = (mp3Playing && mp3PlaybackStartMs)
                     ? (millis() - mp3PlaybackStartMs) : 0;
    char timebuf[16];
    snprintf(timebuf, sizeof(timebuf), "%02lu:%02lu",
             (unsigned long)(elapsed / 60000UL),
             (unsigned long)((elapsed / 1000UL) % 60UL));
    uint16_t col = mp3Playing ? COL_ACCENT : COL_MUTED;
    M5.Display.setTextColor(col, COL_PANEL);
    M5.Display.setTextDatum(middle_right);
    M5.Display.setFont(FONT_SMALL);
    M5.Display.drawString(mp3Playing ? "PLAY" : "STOP", valueX, labelY);
    M5.Display.setFont(FONT_MED);
    M5.Display.drawString(timebuf, valueX, timeY);
  }
  // currentApp == APP_MIDI: no extra header-right text.
  (void)y; (void)h;
}

static void drawAllOffBtn() {
  drawRectBtn(btnAllOff,
              allNotesOffEnabled ? COL_BTN_HI : COL_DANGER,
              COL_BTN_BDR, "AOFF",
              allNotesOffEnabled ? COL_BTN_TXT_HI : COL_BTN_TXT, FONT_MED);
}

static void drawAppTabs() {
  // While a header overlay (BASE_SET / CONFIG_EDIT) is active we are not
  // really "in" any of the three apps — the saved app is just a return
  // target — so suppress the app-tab highlight to make the BASE / CONF
  // button highlight unambiguous.
  bool inOverlay = (currentMode == BASE_SET_MODE || currentMode == CONFIG_EDIT_MODE);
  for (int i = 0; i < 3; ++i) {
    bool on = !inOverlay && (currentApp == (AppMode)i);
    drawRectBtn(appTab[i], on ? COL_BTN_HI2 : COL_BTN, COL_BTN_BDR, appName[i],
                on ? COL_BTN_TXT_HI : COL_BTN_TXT, FONT_MED);
  }
}

// =================================================================
//  MIDI activity indicator — replaces the old IN/OUT counter text.
// =================================================================
// Two thin horizontal stripes at the very top edge of the screen:
//   y=0,1  blue   — flashes when midiInCount advances (incoming MIDI)
//   y=2,3  red    — flashes when midiOutCount advances (outgoing MIDI)
// The numeric counters are still maintained internally and queryable
// via the STATUS USB-serial command; they just don't get drawn here.
// `tickMidiActivityLines()` is called from loop() so the flashes refresh
// independently of full header redraws.
static unsigned long g_lastMidiInDrawn  = 0;
static unsigned long g_lastMidiOutDrawn = 0;
static unsigned long g_midiInFlashUntil  = 0;
static unsigned long g_midiOutFlashUntil = 0;
// Cache of the last colours actually pushed to the LCD for each stripe.
// 0xFFFF is used as an "invalid" sentinel so the next call repaints
// unconditionally — see invalidateMidiActivityStripes() below.
static uint16_t g_lastInColDrawn  = 0xFFFF;
static uint16_t g_lastOutColDrawn = 0xFFFF;

static void tickMidiActivityLines() {
  unsigned long now = millis();
  // Compare against the heartbeat-filtered counters so the stripes don't
  // ticker on idle traffic (Active Sense at ~3 Hz, Clock at 24 ppq).
  if (midiInRealCount != g_lastMidiInDrawn) {
    g_lastMidiInDrawn  = midiInRealCount;
    g_midiInFlashUntil = now + 120;
  }
  if (midiOutRealCount != g_lastMidiOutDrawn) {
    g_lastMidiOutDrawn  = midiOutRealCount;
    g_midiOutFlashUntil = now + 120;
  }
}

// Reset the cache so the next drawMidiActivityLines() call repaints the
// stripes regardless of colour. Call this whenever code clears or paints
// over the y=0..7 strip in the top-right corner (full header redraw).
static void invalidateMidiActivityStripes() {
  g_lastInColDrawn  = 0xFFFF;
  g_lastOutColDrawn = 0xFFFF;
}

static void drawMidiActivityLines() {
  tickMidiActivityLines();
  unsigned long now = millis();
  bool inActive  = now < g_midiInFlashUntil;
  bool outActive = now < g_midiOutFlashUntil;
  uint16_t inCol  = inActive  ? TFT_BLUE : 0x0008;  // bright vs dim
  uint16_t outCol = outActive ? TFT_RED  : 0x0800;
  // Skip the LCD write entirely when the colour hasn't changed since the
  // last paint — this is called at 50 Hz from loop() and most ticks have
  // no transition, so unconditional fillRects were just wasted DSI traffic
  // (and, on Tab5, perceptible flicker).
  if (inCol == g_lastInColDrawn && outCol == g_lastOutColDrawn) return;
  // Two ~5 mm stripes side-by-side in the top-right corner. Tab5 panel is
  // ~213 dpi so 5 mm ≈ 42 px.
  const int stripeW = 42;
  const int stripeH = 6;
  const int gap = 8;
  const int rightMargin = 6;
  int xOut = SCREEN_W - rightMargin - stripeW;
  int xIn  = xOut - gap - stripeW;
  if (inCol != g_lastInColDrawn) {
    M5.Display.fillRect(xIn, 2, stripeW, stripeH, inCol);
    g_lastInColDrawn = inCol;
  }
  if (outCol != g_lastOutColDrawn) {
    M5.Display.fillRect(xOut, 2, stripeW, stripeH, outCol);
    g_lastOutColDrawn = outCol;
  }
}

static void drawToolbarApp() {
  M5.Display.fillRect(toolbarArea.x, toolbarArea.y, toolbarArea.w, toolbarArea.h, COL_BG);

  if (currentApp == APP_TRANSPOSE) {
    drawToolbar();   // XPOSE: 4 mode tabs + RANGE + AOFF (existing)
    return;
  }

  drawAllOffBtn();

  if (currentApp == APP_MIDI) {
    // MIDI: FILTER / MAPPER / BYPASS|ACTIVE in the same toolbar slot as XPOSE mode tabs.
    bool isFilter = (midiManagePage == MIDI_PAGE_FILTER);
    bool isMapper = (midiManagePage == MIDI_PAGE_MAPPER);
    bool bypass   = isFilter ? midiFilterBypass : midiMapperBypass;

    drawRectBtn(midiAppTabFilter, isFilter ? COL_BTN_HI : COL_BTN, COL_BTN_BDR, "FILTER",
                isFilter ? COL_BTN_TXT_HI : COL_BTN_TXT, FONT_MED);
    drawRectBtn(midiAppTabMapper, isMapper ? COL_BTN_HI : COL_BTN, COL_BTN_BDR, "MAPPER",
                isMapper ? COL_BTN_TXT_HI : COL_BTN_TXT, FONT_MED);
    drawRectBtn(midiAppBypassBtn, bypass ? COL_BTN_HI2 : TFT_BLUE, COL_BTN_BDR,
                bypass ? "BYPASS" : "ACTIVE",
                bypass ? COL_BTN_TXT_HI : COL_BTN_TXT, FONT_MED);
    return;
  }

  // APP_PLAY: SRC/SMF/MP3 mode tabs + transport/aux buttons of the active sub-mode.
  for (int i = 0; i < 3; i++) {
    bool on = ((PlayMode)i == currentPlay);
    drawRectBtn(playTab[i], on ? COL_BTN_HI : COL_BTN, COL_BTN_BDR, playName[i],
                on ? COL_BTN_TXT_HI : COL_BTN_TXT, FONT_MED);
  }
  if (currentPlay == PLAY_SRC) {
    // SRC: 4 narrow buttons — GMRST / GSRST / INIT / AUTO. Labels shortened
    // so they fit comfortably in the same toolbar slots as SMF transport.
    // AUTO's state is shown by highlight colour (no "ON"/"OFF" suffix needed).
    drawRectBtn(srcBtnGm,   TFT_BLUE,   COL_BTN_BDR, "GMRST", COL_BTN_TXT, FONT_MED);
    drawRectBtn(srcBtnGs,   TFT_BLUE,   COL_BTN_BDR, "GSRST", COL_BTN_TXT, FONT_MED);
    drawRectBtn(srcBtnInit, COL_DANGER, COL_BTN_BDR, "INIT",  COL_BTN_TXT, FONT_MED);
    drawRectBtn(srcBtnAuto,
                srcAutoFollow ? COL_BTN_HI : COL_BTN, COL_BTN_BDR, "AUTO",
                srcAutoFollow ? COL_BTN_TXT_HI : COL_BTN_TXT, FONT_MED);
  } else if (currentPlay == PLAY_SMF) {
    drawRectBtn(smfBtnPrev, TFT_BLUE, COL_BTN_BDR, "PREV", COL_BTN_TXT, FONT_MED);
    drawRectBtn(smfBtnPlay, smfPlaying ? COL_DANGER : COL_BTN_HI, COL_BTN_BDR,
                smfPlaying ? "STOP" : "PLAY",
                smfPlaying ? COL_BTN_TXT : COL_BTN_TXT_HI, FONT_MED);
    drawRectBtn(smfBtnNext, TFT_BLUE, COL_BTN_BDR, "NEXT", COL_BTN_TXT, FONT_MED);
    drawRectBtn(smfBtnLoop, smfLoop ? COL_BTN_HI : COL_BTN, COL_BTN_BDR,
                smfLoop ? "RPTON" : "RPTOFF",
                smfLoop ? COL_BTN_TXT_HI : COL_BTN_TXT, FONT_MED);
  } else {
    drawRectBtn(mp3BtnPrev, TFT_BLUE, COL_BTN_BDR, "PREV", COL_BTN_TXT, FONT_MED);
    drawRectBtn(mp3BtnPlay, mp3Playing ? COL_DANGER : COL_BTN_HI, COL_BTN_BDR,
                mp3Playing ? "STOP" : "PLAY",
                mp3Playing ? COL_BTN_TXT : COL_BTN_TXT_HI, FONT_MED);
    drawRectBtn(mp3BtnNext, TFT_BLUE, COL_BTN_BDR, "NEXT", COL_BTN_TXT, FONT_MED);
    drawRectBtn(mp3BtnVolDown, COL_BTN, COL_BTN_BDR, "V-", COL_BTN_TXT, FONT_MED);
    drawRectBtn(mp3BtnVolUp,   COL_BTN, COL_BTN_BDR, "V+", COL_BTN_TXT, FONT_MED);
  }
}

static void drawNavApp() {
  if (currentApp == APP_TRANSPOSE) {
    drawNav();
    return;
  }
  M5.Display.fillRect(navArea.x, navArea.y, navArea.w, navArea.h, COL_BG);
}

static void drawDirect() {
  M5.Display.fillRect(contentArea.x, contentArea.y, contentArea.w, contentArea.h, COL_BG);
  for (int i = 0; i < 12; i++) {
    bool on = transposeButtonsOn[i];
    uint16_t bg  = on ? COL_BTN_HI : COL_BTN;
    uint16_t txt = on ? COL_BTN_TXT_HI : COL_BTN_TXT;
    drawRectBtn(directBtns[i].r, bg, COL_BTN_BDR, directBtns[i].label, txt, FONT_HUGE);
  }
}

static void drawKey() {
  M5.Display.fillRect(contentArea.x, contentArea.y, contentArea.w, contentArea.h, COL_BG);

  // Section labels.
  M5.Display.setFont(FONT_SMALL);
  M5.Display.setTextColor(COL_ACCENT, COL_BG);
  M5.Display.setTextDatum(top_left);
  M5.Display.drawString(majorUpperTranspose ? "Major Keys (Upper +)" : "Major Keys (to C)",
                        30, contentArea.y + 10);
  int midLabelY = majorKeys[0].r.y + majorKeys[0].r.h + 10;
  M5.Display.drawString(minorUpperTranspose ? "Minor Keys (Lower -)" : "Minor Keys (to Am)",
                        30, midLabelY);

  int correspondingMajor = -1;
  int correspondingMinor = -1;
  if (majorUpperTranspose) {
    if (transposeValue >= 1 && transposeValue <= 12) correspondingMajor = 12 - transposeValue;
  } else {
    if (transposeValue <= 0 && transposeValue >= -11) correspondingMajor = -transposeValue;
  }
  if (minorUpperTranspose) {
    if (transposeValue <= -1 && transposeValue >= -9) correspondingMinor = 9 + transposeValue;
  } else {
    if (transposeValue >= -2 && transposeValue <= 9) correspondingMinor = 9 - transposeValue;
  }

  auto drawKeySet = [&](KeyBtn* keys, int selected, int corresponding,
                        uint16_t selColor, uint16_t corrColor) {
    // white keys first, then black so black overlays.
    for (int pass = 0; pass < 2; pass++) {
      for (int i = 0; i < 12; i++) {
        if (keys[i].isBlackKey != (pass == 1)) continue;
        uint16_t bg, txt;
        if (selected == i)          { bg = selColor;  txt = TFT_BLACK; }
        else if (corresponding == i) { bg = corrColor; txt = TFT_BLACK; }
        else if (keys[i].isBlackKey) { bg = COL_BLACKKEY; txt = COL_BTN_TXT; }
        else                         { bg = COL_WHITEKEY; txt = TFT_BLACK; }
        M5.Display.fillRoundRect(keys[i].r.x, keys[i].r.y, keys[i].r.w, keys[i].r.h, 6, bg);
        M5.Display.drawRoundRect(keys[i].r.x, keys[i].r.y, keys[i].r.w, keys[i].r.h, 6, TFT_DARKGREY);
        M5.Display.setFont(FONT_MED);
        M5.Display.setTextColor(txt, bg);
        M5.Display.setTextDatum(bottom_center);
        M5.Display.drawString(keys[i].keyName,
                              keys[i].r.x + keys[i].r.w / 2,
                              keys[i].r.y + keys[i].r.h - 10);
      }
    }
  };
  drawKeySet(majorKeys, selectedMajorKey, correspondingMajor, TFT_GREEN, 0x07E0);
  drawKeySet(minorKeys, selectedMinorKey, correspondingMinor, TFT_ORANGE, 0xFD20);
}

static void drawInstant() {
  M5.Display.fillRect(contentArea.x, contentArea.y, contentArea.w, contentArea.h, COL_BG);

  M5.Display.setFont(FONT_SMALL);
  M5.Display.setTextColor(COL_ACCENT, COL_BG);
  M5.Display.setTextDatum(top_left);
  M5.Display.drawString("Instant: tap to jump to that value",
                        30, contentArea.y + 6);

  // 0 button (wide).
  bool zeroOn = (transposeValue == 0);
  drawRectBtn(instantZero,
              zeroOn ? COL_BTN_HI : COL_BTN,
              COL_BTN_BDR, "0",
              zeroOn ? COL_BTN_TXT_HI : COL_BTN_TXT,
              FONT_HUGE);

  for (int i = 0; i < 8; i++) {
    bool on = (instantBtns[i].value == transposeValue);
    uint16_t bg  = on ? COL_BTN_HI : COL_BTN;
    uint16_t txt = on ? COL_BTN_TXT_HI : COL_BTN_TXT;
    drawRectBtn(instantBtns[i].r, bg, COL_BTN_BDR, instantBtns[i].label, txt, FONT_HUGE);
  }
}

static void drawSequence() {
  M5.Display.fillRect(contentArea.x, contentArea.y, contentArea.w, contentArea.h, COL_BG);

  // Pattern nav row.
  drawRectBtn(seqPatLeft,  TFT_BLUE, COL_BTN_BDR, "<", COL_BTN_TXT, FONT_HUGE);
  drawRectBtn(seqPatRight, TFT_BLUE, COL_BTN_BDR, ">", COL_BTN_TXT, FONT_HUGE);

  // Pattern label between the arrows.
  Rect patLabel = {
    seqPatLeft.x + seqPatLeft.w + 10,
    seqPatLeft.y,
    seqPatRight.x - (seqPatLeft.x + seqPatLeft.w) - 20,
    seqPatLeft.h
  };
  M5.Display.fillRoundRect(patLabel.x, patLabel.y, patLabel.w, patLabel.h, 10, COL_NAVY);
  M5.Display.drawRoundRect(patLabel.x, patLabel.y, patLabel.w, patLabel.h, 10, COL_BTN_BDR);
  char patStr[24];
  snprintf(patStr, sizeof(patStr), "Pattern %02d / %02d",
           seqCurrentPattern + 1, SEQ_PATTERN_COUNT);
  M5.Display.setFont(FONT_HUGE);
  M5.Display.setTextColor(COL_VALUE, COL_NAVY);
  M5.Display.setTextDatum(middle_center);
  M5.Display.drawString(patStr, patLabel.x + patLabel.w / 2, patLabel.y + patLabel.h / 2);

  // SAVE button.
  const char* saveLabel = "SAVE";
  uint16_t saveBg = TFT_RED;
  uint16_t saveTxt = COL_BTN_TXT;
  if (g_seqSaveUiState == SEQ_SAVE_UI_PENDING) {
    saveLabel = "QUEUED";
    saveBg = TFT_BLUE;
  } else if (g_seqSaveUiState == SEQ_SAVE_UI_OK) {
    saveLabel = "SAVED";
    saveBg = TFT_GREEN;
    saveTxt = TFT_BLACK;
  } else if (g_seqSaveUiState == SEQ_SAVE_UI_ERR) {
    saveLabel = "SD ERR";
    saveBg = TFT_ORANGE;
    saveTxt = TFT_BLACK;
  }
  drawRectBtn(seqSave, saveBg, COL_BTN_BDR, saveLabel, saveTxt, FONT_HUGE);

  // Step slots.
  for (int i = 0; i < SEQ_STEP_COUNT; i++) {
    int8_t value = seqPatterns[seqCurrentPattern][i];
    bool isCurrent = (i == seqCurrentStep);

    // UP button with a triangle.
    drawRectBtn(seqSteps[i].up, COL_BTN, COL_BTN_BDR, nullptr, COL_BTN_TXT, FONT_MED);
    {
      int cx = seqSteps[i].up.x + seqSteps[i].up.w / 2;
      int cy = seqSteps[i].up.y + seqSteps[i].up.h / 2;
      int tri = 18;
      M5.Display.fillTriangle(cx, cy - tri, cx - tri, cy + tri, cx + tri, cy + tri, COL_BTN_TXT);
    }

    // Value slot.
    char vstr[6];
    if (value > 0) snprintf(vstr, sizeof(vstr), "+%d", value);
    else           snprintf(vstr, sizeof(vstr), "%d",  value);
    drawRectBtn(seqSteps[i].slot,
                isCurrent ? COL_BTN_HI : COL_BTN,
                COL_BTN_BDR, vstr,
                isCurrent ? COL_BTN_TXT_HI : COL_BTN_TXT,
                FONT_HUGE);

    // DOWN button with a triangle.
    drawRectBtn(seqSteps[i].down, COL_BTN, COL_BTN_BDR, nullptr, COL_BTN_TXT, FONT_MED);
    {
      int cx = seqSteps[i].down.x + seqSteps[i].down.w / 2;
      int cy = seqSteps[i].down.y + seqSteps[i].down.h / 2;
      int tri = 18;
      M5.Display.fillTriangle(cx, cy + tri, cx - tri, cy - tri, cx + tri, cy - tri, COL_BTN_TXT);
    }
  }

}

// ==== MIDI Manager (FILTER / MAPPER) UI ====
// Tab5 1280×720 用フルエディタ。FILTER/MAPPER/BYPASS の上部タブ、ルール一覧、
// 編集行 (FILTER は Type+Ch、MAPPER は SOURCE/DESTINATION 並列)、操作ボタン。

enum CyclerKind : uint8_t {
  CY_F_TYPE, CY_F_CH,
  CY_M_SRC_TYPE, CY_M_SRC_CH, CY_M_SRC_DATA1, CY_M_SRC_MIN, CY_M_SRC_MAX,
  CY_M_DST_TYPE, CY_M_DST_CH, CY_M_DST_DATA1, CY_M_DST_MIN, CY_M_DST_MAX,
  CY_COUNT
};
enum ActionKind { ACT_ENDIS, ACT_ADD, ACT_DEL, ACT_UP, ACT_DOWN, ACT_COUNT };

static Rect midiTabFilter;
static Rect midiTabMapper;
static Rect midiBypassBtn;
static Rect midiRuleRows[MAX_FILTER_RULES];
static Rect midiRuleListPanel;
static Rect midiCyclerArea[CY_COUNT];
static Rect midiCyclerLeft[CY_COUNT];
static Rect midiCyclerRight[CY_COUNT];
static bool midiCyclerActive[CY_COUNT];
static Rect midiActionBtn[ACT_COUNT];
static int  midiListFirstRow = 0;   // 直近の draw でのスクロール位置 (touch 用)
static int  midiListVisible  = 0;   // 直近の draw で見せた行数

enum MidiKeypadButtonIndex {
  MKP_BTN_1, MKP_BTN_2, MKP_BTN_3,
  MKP_BTN_4, MKP_BTN_5, MKP_BTN_6,
  MKP_BTN_7, MKP_BTN_8, MKP_BTN_9,
  MKP_BTN_SPECIAL, MKP_BTN_0, MKP_BTN_BS,
  MKP_BTN_CANCEL, MKP_BTN_OK,
  MKP_BTN_COUNT
};

static bool g_midiKeypadActive = false;
static bool g_midiKeypadUseSpecial = false;
static CyclerKind g_midiKeypadTarget = CY_COUNT;
static int g_midiKeypadMinValue = 0;
static int g_midiKeypadMaxValue = 0;
static bool g_midiKeypadOneBased = false;
static char g_midiKeypadSpecialLabel[8] = {};
static char g_midiKeypadDigits[8] = {};
static Rect g_midiKeypadPanel;
static Rect g_midiKeypadDisplay;
static Rect g_midiKeypadButtons[MKP_BTN_COUNT];

static void requestMidiManageRefresh() {
  midiManageDirty = true;
}

static void cycleMidiKind(MidiMessageKind& kind, int delta) {
  int v = ((int)kind + delta + MIDI_KIND_COUNT) % MIDI_KIND_COUNT;
  kind = (MidiMessageKind)v;
}
static void cycleChannelValue(int8_t& ch, int delta) {
  // -1 (ALL/KEEP), 0..15 (Ch1..Ch16) → 17 値
  int v = ((int)ch + 1 + delta + 17) % 17;
  ch = (int8_t)(v - 1);
}
static void cycleData1Value(int16_t& d, int delta) {
  // -1 (ANY/KEEP), 0..127 → 129 値
  int v = ((int)d + 1 + delta + 129) % 129;
  d = (int16_t)(v - 1);
}
static void cycleClampedValue(int16_t& v, int delta, int maxVal) {
  int x = (int)v + delta;
  if (x < 0)      x = 0;
  if (x > maxVal) x = maxVal;
  v = (int16_t)x;
}

static bool midiCyclerSupportsKeypad(CyclerKind k) {
  switch (k) {
    case CY_F_CH:
    case CY_M_SRC_CH:
    case CY_M_SRC_DATA1:
    case CY_M_SRC_MIN:
    case CY_M_SRC_MAX:
    case CY_M_DST_CH:
    case CY_M_DST_DATA1:
    case CY_M_DST_MIN:
    case CY_M_DST_MAX:
      return true;
    default:
      return false;
  }
}

static int parseMidiKeypadDigits(void) {
  if (g_midiKeypadDigits[0] == '\0') return 0;
  return atoi(g_midiKeypadDigits);
}

static void setMidiKeypadDigitsFromValue(int value) {
  snprintf(g_midiKeypadDigits, sizeof(g_midiKeypadDigits), "%d", value);
}

static bool prepareMidiKeypad(CyclerKind k) {
  g_midiKeypadTarget = k;
  g_midiKeypadUseSpecial = false;
  g_midiKeypadOneBased = false;
  g_midiKeypadSpecialLabel[0] = '\0';
  g_midiKeypadMinValue = 0;
  g_midiKeypadMaxValue = 127;

  if (midiManagePage == MIDI_PAGE_FILTER) {
    MidiFilterRule& r = midiFilterRules[midiSelectedFilterRule];
    if (k == CY_F_CH) {
      if (!midiKindHasChannel(r.kind)) return false;
      g_midiKeypadOneBased = true;
      g_midiKeypadMinValue = 1;
      g_midiKeypadMaxValue = 16;
      strcpy(g_midiKeypadSpecialLabel, "ALL");
      if (r.channel < 0) g_midiKeypadUseSpecial = true;
      else setMidiKeypadDigitsFromValue(r.channel + 1);
      return true;
    }
    return false;
  }

  MidiMapperRule& r = midiMapperRules[midiSelectedMapperRule];
  switch (k) {
    case CY_M_SRC_CH:
      if (!midiKindHasChannel(r.srcKind)) return false;
      g_midiKeypadOneBased = true;
      g_midiKeypadMinValue = 1;
      g_midiKeypadMaxValue = 16;
      strcpy(g_midiKeypadSpecialLabel, "ALL");
      if (r.srcChannel < 0) g_midiKeypadUseSpecial = true;
      else setMidiKeypadDigitsFromValue(r.srcChannel + 1);
      return true;
    case CY_M_DST_CH:
      if (!midiKindHasChannel(r.dstKind)) return false;
      g_midiKeypadOneBased = true;
      g_midiKeypadMinValue = 1;
      g_midiKeypadMaxValue = 16;
      strcpy(g_midiKeypadSpecialLabel, "KEEP");
      if (r.dstChannel < 0) g_midiKeypadUseSpecial = true;
      else setMidiKeypadDigitsFromValue(r.dstChannel + 1);
      return true;
    case CY_M_SRC_DATA1:
      if (!midiKindSupportsData1(r.srcKind)) return false;
      g_midiKeypadMinValue = 0;
      g_midiKeypadMaxValue = 127;
      strcpy(g_midiKeypadSpecialLabel, "ANY");
      if (r.srcData1 < 0) g_midiKeypadUseSpecial = true;
      else setMidiKeypadDigitsFromValue(r.srcData1);
      return true;
    case CY_M_DST_DATA1:
      if (!midiKindSupportsData1(r.dstKind)) return false;
      g_midiKeypadMinValue = 0;
      g_midiKeypadMaxValue = 127;
      strcpy(g_midiKeypadSpecialLabel, "KEEP");
      if (r.dstData1 < 0) g_midiKeypadUseSpecial = true;
      else setMidiKeypadDigitsFromValue(r.dstData1);
      return true;
    case CY_M_SRC_MIN:
      g_midiKeypadMinValue = 0;
      g_midiKeypadMaxValue = getMidiValueMax(r.srcKind);
      setMidiKeypadDigitsFromValue(r.srcMin);
      return true;
    case CY_M_SRC_MAX:
      g_midiKeypadMinValue = 0;
      g_midiKeypadMaxValue = getMidiValueMax(r.srcKind);
      setMidiKeypadDigitsFromValue(r.srcMax);
      return true;
    case CY_M_DST_MIN:
      g_midiKeypadMinValue = 0;
      g_midiKeypadMaxValue = getMidiValueMax(r.dstKind);
      setMidiKeypadDigitsFromValue(r.dstMin);
      return true;
    case CY_M_DST_MAX:
      g_midiKeypadMinValue = 0;
      g_midiKeypadMaxValue = getMidiValueMax(r.dstKind);
      setMidiKeypadDigitsFromValue(r.dstMax);
      return true;
    default:
      return false;
  }
}

static void openMidiKeypad(CyclerKind k) {
  if (!prepareMidiKeypad(k)) return;
  g_midiKeypadActive = true;
  requestMidiManageRefresh();
}

static void closeMidiKeypad(void) {
  g_midiKeypadActive = false;
  g_midiKeypadTarget = CY_COUNT;
  requestMidiManageRefresh();
}

static void applyMidiKeypadValue(void) {
  if (g_midiKeypadTarget == CY_COUNT) return;

  int rawValue = g_midiKeypadUseSpecial ? -1 : parseMidiKeypadDigits();
  if (!g_midiKeypadUseSpecial) {
    if (rawValue < g_midiKeypadMinValue) rawValue = g_midiKeypadMinValue;
    if (rawValue > g_midiKeypadMaxValue) rawValue = g_midiKeypadMaxValue;
  }

  if (midiManagePage == MIDI_PAGE_FILTER) {
    MidiFilterRule& r = midiFilterRules[midiSelectedFilterRule];
    if (g_midiKeypadTarget == CY_F_CH) {
      r.channel = g_midiKeypadUseSpecial ? -1 : (int8_t)(rawValue - 1);
    }
    closeMidiKeypad();
    return;
  }

  MidiMapperRule& r = midiMapperRules[midiSelectedMapperRule];
  switch (g_midiKeypadTarget) {
    case CY_M_SRC_CH:    r.srcChannel = g_midiKeypadUseSpecial ? -1 : (int8_t)(rawValue - 1); break;
    case CY_M_DST_CH:    r.dstChannel = g_midiKeypadUseSpecial ? -1 : (int8_t)(rawValue - 1); break;
    case CY_M_SRC_DATA1: r.srcData1   = g_midiKeypadUseSpecial ? -1 : (int16_t)rawValue; break;
    case CY_M_DST_DATA1: r.dstData1   = g_midiKeypadUseSpecial ? -1 : (int16_t)rawValue; break;
    case CY_M_SRC_MIN:   r.srcMin     = (int16_t)rawValue; if (r.srcMax < r.srcMin) r.srcMax = r.srcMin; break;
    case CY_M_SRC_MAX:   r.srcMax     = (int16_t)rawValue; if (r.srcMin > r.srcMax) r.srcMin = r.srcMax; break;
    case CY_M_DST_MIN:   r.dstMin     = (int16_t)rawValue; if (r.dstMax < r.dstMin) r.dstMax = r.dstMin; break;
    case CY_M_DST_MAX:   r.dstMax     = (int16_t)rawValue; if (r.dstMin > r.dstMax) r.dstMin = r.dstMax; break;
    default: break;
  }
  closeMidiKeypad();
}

static void drawCycler(int idx, const Rect& area, const char* label, const char* value, bool enabled) {
  const int btnW = 48;
  Rect lbtn = { area.x,                area.y, btnW, area.h };
  Rect mid  = { area.x + btnW,         area.y, area.w - btnW * 2, area.h };
  Rect rbtn = { area.x + area.w - btnW, area.y, btnW, area.h };
  midiCyclerArea[idx]   = area;
  midiCyclerLeft[idx]   = lbtn;
  midiCyclerRight[idx]  = rbtn;
  midiCyclerActive[idx] = enabled;

  uint16_t bg     = enabled ? COL_BTN : COL_PANEL;
  uint16_t accent = enabled ? COL_NAVY : COL_PANEL;
  M5.Display.fillRoundRect(area.x, area.y, area.w, area.h, 8, bg);
  M5.Display.drawRoundRect(area.x, area.y, area.w, area.h, 8, COL_BTN_BDR);
  M5.Display.fillRoundRect(lbtn.x, lbtn.y, lbtn.w, lbtn.h, 8, accent);
  M5.Display.fillRoundRect(rbtn.x, rbtn.y, rbtn.w, rbtn.h, 8, accent);
  M5.Display.drawRoundRect(lbtn.x, lbtn.y, lbtn.w, lbtn.h, 8, COL_BTN_BDR);
  M5.Display.drawRoundRect(rbtn.x, rbtn.y, rbtn.w, rbtn.h, 8, COL_BTN_BDR);

  M5.Display.setTextDatum(middle_center);
  M5.Display.setFont(FONT_MED);
  M5.Display.setTextColor(enabled ? COL_BTN_TXT : COL_MUTED);
  M5.Display.drawString("<", lbtn.x + lbtn.w / 2, lbtn.y + lbtn.h / 2);
  M5.Display.drawString(">", rbtn.x + rbtn.w / 2, rbtn.y + rbtn.h / 2);

  // Label + value
  M5.Display.setTextDatum(top_left);
  M5.Display.setFont(FONT_TINY);
  M5.Display.setTextColor(COL_MUTED);
  M5.Display.drawString(label, mid.x + 10, mid.y + 6);

  const lgfx::IFont* valueFont = FONT_SMALL;
  M5.Display.setFont(valueFont);
  if ((int)M5.Display.textWidth(value) > (mid.w - 16)) {
    valueFont = FONT_TINY;
  }
  M5.Display.setTextDatum(middle_center);
  M5.Display.setFont(valueFont);
  M5.Display.setTextColor(enabled ? COL_VALUE : COL_MUTED);
  M5.Display.drawString(value, mid.x + mid.w / 2, mid.y + area.h - 18);
}

static void drawMidiKeypadOverlay(void) {
  Rect host = midiRuleListPanel;
  int panelW = host.w - 20;
  int panelH = host.h - 20;
  int px = host.x + 10;
  int py = host.y + 10;
  g_midiKeypadPanel = { px, py, panelW, panelH };
  g_midiKeypadDisplay = { px + 18, py + 48, panelW - 36, 72 };

  M5.Display.fillRoundRect(g_midiKeypadPanel.x, g_midiKeypadPanel.y, g_midiKeypadPanel.w, g_midiKeypadPanel.h, 16, COL_PANEL);
  M5.Display.drawRoundRect(g_midiKeypadPanel.x, g_midiKeypadPanel.y, g_midiKeypadPanel.w, g_midiKeypadPanel.h, 16, COL_BTN_BDR);
  M5.Display.setTextDatum(top_left);
  M5.Display.setFont(FONT_SMALL);
  M5.Display.setTextColor(COL_ACCENT, COL_PANEL);
  M5.Display.drawString("DIRECT INPUT", px + 20, py + 16);

  M5.Display.fillRoundRect(g_midiKeypadDisplay.x, g_midiKeypadDisplay.y, g_midiKeypadDisplay.w, g_midiKeypadDisplay.h, 10, COL_BG);
  M5.Display.drawRoundRect(g_midiKeypadDisplay.x, g_midiKeypadDisplay.y, g_midiKeypadDisplay.w, g_midiKeypadDisplay.h, 10, COL_BTN_BDR);
  const char* displayText = g_midiKeypadUseSpecial
                              ? g_midiKeypadSpecialLabel
                              : (g_midiKeypadDigits[0] ? g_midiKeypadDigits : "0");
  M5.Display.setTextDatum(middle_right);
  M5.Display.setFont(FONT_HUGE);
  M5.Display.setTextColor(COL_VALUE, COL_BG);
  M5.Display.drawString(displayText, g_midiKeypadDisplay.x + g_midiKeypadDisplay.w - 18,
                        g_midiKeypadDisplay.y + g_midiKeypadDisplay.h / 2);

  M5.Display.setTextDatum(top_left);
  M5.Display.setFont(FONT_TINY);
  M5.Display.setTextColor(COL_MUTED, COL_PANEL);
  char hint[64];
  if (g_midiKeypadSpecialLabel[0]) {
    snprintf(hint, sizeof(hint), "Range %d-%d or %s", g_midiKeypadMinValue, g_midiKeypadMaxValue, g_midiKeypadSpecialLabel);
  } else {
    snprintf(hint, sizeof(hint), "Range %d-%d", g_midiKeypadMinValue, g_midiKeypadMaxValue);
  }
  M5.Display.drawString(hint, px + 20, py + 130);

  const char* labels[MKP_BTN_COUNT] = {
    "1","2","3","4","5","6","7","8","9",
    g_midiKeypadSpecialLabel[0] ? g_midiKeypadSpecialLabel : "-",
    "0","BS","CANCEL","OK"
  };
  const int btnGap = 10;
  const int gridX = px + 18;
  const int gridY = py + 156;
  const int gridW = panelW - 36;
  const int topBtnW = (gridW - btnGap * 2) / 3;
  const int topBtnH = 52;
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 3; col++) {
      int idx = row * 3 + col;
      g_midiKeypadButtons[idx] = { gridX + col * (topBtnW + btnGap), gridY + row * (topBtnH + btnGap), topBtnW, topBtnH };
    }
  }
  int bottomY = gridY + 4 * (topBtnH + btnGap);
  int bottomW = (gridW - btnGap) / 2;
  int bottomH = panelH - (bottomY - py) - 18;
  if (bottomH < 54) bottomH = 54;
  g_midiKeypadButtons[MKP_BTN_CANCEL] = { gridX, bottomY, bottomW, bottomH };
  g_midiKeypadButtons[MKP_BTN_OK]     = { gridX + bottomW + btnGap, bottomY, bottomW, bottomH };

  for (int i = 0; i < MKP_BTN_COUNT; i++) {
    uint16_t bg = COL_BTN;
    uint16_t txt = COL_BTN_TXT;
    if (i == MKP_BTN_CANCEL) bg = COL_DANGER;
    else if (i == MKP_BTN_OK) { bg = COL_BTN_HI; txt = COL_BTN_TXT_HI; }
    else if (i == MKP_BTN_SPECIAL && g_midiKeypadUseSpecial) { bg = COL_BTN_HI2; txt = COL_BTN_TXT_HI; }
    drawRectBtn(g_midiKeypadButtons[i], bg, COL_BTN_BDR, labels[i], txt, FONT_MED);
  }
}

static void drawMidiManage() {
  // FILTER/MAPPER/BYPASS のタブは toolbar 側に出している。
  // ここでは content + nav を一体のワークスペースとして使う。
  Rect manageArea = {
    contentArea.x,
    contentArea.y,
    contentArea.w,
    contentArea.h + navArea.h
  };
  M5.Display.fillRect(manageArea.x, manageArea.y, manageArea.w, manageArea.h, COL_BG);

  const bool isFilter = (midiManagePage == MIDI_PAGE_FILTER);
  const int margin = 18;
  const int gap = 18;
  const int x0 = manageArea.x + margin;
  const int y0 = manageArea.y + margin;
  const int w = manageArea.w - margin * 2;
  const int h = manageArea.h - margin * 2;
  const int listW = 480;
  const int sideX = x0 + listW + gap;
  const int sideW = w - listW - gap;
  const int topCardH = 54;
  const int actionPanelH = 92;
  const int editorY = y0 + topCardH + gap;
  const int editorH = h - topCardH - actionPanelH - gap * 2;
  const int actionY = editorY + editorH + gap;
  const int listY = y0;
  const int listH = h;
  const int rowH = 52;

  // ── Rule list ──
  midiRuleListPanel = { x0, listY, listW, listH };
  M5.Display.fillRoundRect(x0, listY, listW, listH, 12, COL_PANEL);
  M5.Display.drawRoundRect(x0, listY, listW, listH, 12, COL_BTN_BDR);
  M5.Display.setTextDatum(top_left);
  M5.Display.setFont(FONT_SMALL);
  M5.Display.setTextColor(COL_ACCENT, COL_PANEL);
  M5.Display.drawString(isFilter ? "FILTER RULES" : "MAPPER RULES", x0 + 14, listY + 10);

  int ruleCount = isFilter ? midiFilterRuleCount : midiMapperRuleCount;
  int selected  = isFilter ? midiSelectedFilterRule : midiSelectedMapperRule;
  int rowsVisible = (listH - 68) / rowH;
  if (rowsVisible > MAX_FILTER_RULES) rowsVisible = MAX_FILTER_RULES;
  if (rowsVisible < 1) rowsVisible = 1;
  int firstRow = 0;
  if (selected >= rowsVisible) firstRow = selected - rowsVisible + 1;
  if (firstRow + rowsVisible > ruleCount) firstRow = std::max(0, ruleCount - rowsVisible);
  midiListFirstRow = firstRow;
  midiListVisible  = rowsVisible;

  for (int slot = 0; slot < rowsVisible; slot++) {
    int i = firstRow + slot;
    if (i >= ruleCount) {
      midiRuleRows[slot] = {0, 0, 0, 0};
      continue;
    }
    Rect row = { x0 + 10, listY + 44 + slot * rowH, listW - 20, rowH - 6 };
    midiRuleRows[slot] = row;
    bool sel = (i == selected);
    uint16_t bg = sel ? COL_BTN_HI : COL_BTN;
    M5.Display.fillRoundRect(row.x, row.y, row.w, row.h, 6, bg);

    char line[96];
    if (isFilter) formatMidiFilterRuleSummary(midiFilterRules[i], i, line, sizeof(line));
    else          formatMidiMapperRuleSummary(midiMapperRules[i], i, line, sizeof(line));
    Rect textR = { row.x + 12, row.y, row.w - 72, row.h };
    drawTextFit(textR, line, FONT_SMALL,
                sel ? COL_BTN_TXT_HI : COL_BTN_TXT, bg, 0);

    bool en = isFilter ? midiFilterRules[i].enabled : midiMapperRules[i].enabled;
    M5.Display.setTextDatum(middle_right);
    M5.Display.setFont(FONT_SMALL);
    M5.Display.setTextColor(en ? TFT_GREEN : COL_DANGER);
    M5.Display.drawString(en ? "EN" : "DIS", row.x + row.w - 14, row.y + row.h / 2);
  }
  for (int slot = rowsVisible; slot < MAX_FILTER_RULES; slot++) midiRuleRows[slot] = {0, 0, 0, 0};

  // ── Summary / context card ──
  M5.Display.fillRoundRect(sideX, y0, sideW, topCardH, 12, COL_PANEL);
  M5.Display.drawRoundRect(sideX, y0, sideW, topCardH, 12, COL_BTN_BDR);
  {
    char summary[128];
    snprintf(summary, sizeof(summary), "%s   Rule %d / %d   %s",
             isFilter ? "FILTER EDITOR" : "MAPPER EDITOR",
             ruleCount > 0 ? (selected + 1) : 0,
             ruleCount,
             (isFilter ? (midiFilterBypass ? "BYPASS" : "ACTIVE")
                       : (midiMapperBypass ? "BYPASS" : "ACTIVE")));
    M5.Display.setTextDatum(top_left);
    M5.Display.setFont(FONT_SMALL);
    M5.Display.setTextColor(COL_ACCENT, COL_PANEL);
    M5.Display.drawString(summary, sideX + 14, y0 + 10);
    M5.Display.setTextColor(COL_MUTED, COL_PANEL);
    M5.Display.drawString(isFilter
                            ? "Select a rule on the left, then adjust Type and Channel here."
                            : "Edit the source condition and destination rewrite side-by-side.",
                          sideX + 14, y0 + 30);
  }

  // ── Edit area ──
  M5.Display.fillRoundRect(sideX, editorY, sideW, editorH, 12, COL_PANEL);
  M5.Display.drawRoundRect(sideX, editorY, sideW, editorH, 12, COL_BTN_BDR);

  // 全 cycler を非アクティブ初期化 → 描いたものだけアクティブになる
  for (int k = 0; k < CY_COUNT; k++) midiCyclerActive[k] = false;

  if (isFilter) {
    MidiFilterRule& r = midiFilterRules[midiSelectedFilterRule];
    M5.Display.setTextDatum(top_left);
    M5.Display.setFont(FONT_SMALL);
    M5.Display.setTextColor(COL_ACCENT, COL_PANEL);
    M5.Display.drawString("MATCH CONDITION", sideX + 14, editorY + 10);
    int cw = (sideW - 54) / 2;
    int cy = editorY + 38;
    int ch = editorH - 54;
    Rect cycType = { sideX + 18,            cy, cw, ch };
    Rect cycCh   = { sideX + 36 + cw,       cy, cw, ch };
    drawCycler(CY_F_TYPE, cycType, "Type", getMidiKindLabel(r.kind), true);
    bool chEditable = midiKindHasChannel(r.kind);
    drawCycler(CY_F_CH,   cycCh,   "Ch",   getChannelLabel(r.channel, false), chEditable);
  } else {
    MidiMapperRule& r = midiMapperRules[midiSelectedMapperRule];
    int colW = (sideW - 56) / 2;
    int colY = editorY + 14;
    int colH = editorH - 28;
    Rect colSrc = { sideX + 14,           colY, colW, colH };
    Rect colDst = { sideX + 14 + colW + 28, colY, colW, colH };

    auto drawCol = [&](const Rect& col, const char* head, MidiMessageKind kind,
                       int8_t channel, int16_t data1, int16_t minV, int16_t maxV,
                       CyclerKind cyType, CyclerKind cyCh, CyclerKind cyD1, CyclerKind cyMn, CyclerKind cyMx,
                       bool isDst) {
      M5.Display.fillRoundRect(col.x, col.y, col.w, col.h, 10, COL_BG);
      M5.Display.drawRoundRect(col.x, col.y, col.w, col.h, 10, COL_BTN_BDR);

      // ヘッダ
      M5.Display.setTextDatum(top_left);
      M5.Display.setFont(FONT_SMALL);
      M5.Display.setTextColor(COL_ACCENT);
      M5.Display.drawString(head, col.x + 10, col.y + 8);
      M5.Display.setFont(FONT_TINY);
      M5.Display.setTextColor(COL_MUTED);
      M5.Display.drawString(isDst ? "rewrite before MIDI out" : "match incoming MIDI", col.x + 10, col.y + 26);

      const int rowGap = 14;
      const int innerPad = 12;
      const int innerY = col.y + 46;
      const int innerW = col.w - innerPad * 2;
      const int row1H = 60;
      const int pairH = (col.h - 46 - row1H - rowGap * 2 - innerPad) / 2;
      const int pairGap = 14;
      const int pairW = (innerW - pairGap) / 2;

      Rect typeR = { col.x + innerPad, innerY, innerW, row1H };
      Rect chR   = { col.x + innerPad, innerY + row1H + rowGap, pairW, pairH };
      Rect d1R   = { chR.x + pairW + pairGap, chR.y, pairW, pairH };
      Rect minR  = { col.x + innerPad, chR.y + pairH + rowGap, pairW, pairH };
      Rect maxR  = { minR.x + pairW + pairGap, minR.y, pairW, pairH };

      drawCycler(cyType, typeR, "Type",  getMidiKindLabel(kind), true);
      bool chEditable = midiKindHasChannel(kind);
      drawCycler(cyCh,   chR,   "Ch",    getChannelLabel(channel, isDst), chEditable);
      bool d1Editable = midiKindSupportsData1(kind);
      drawCycler(cyD1,   d1R,   "Data1", getData1Label(data1, isDst), d1Editable);
      char mn[8], mx[8];
      snprintf(mn, sizeof(mn), "%d", minV);
      snprintf(mx, sizeof(mx), "%d", maxV);
      drawCycler(cyMn,   minR,  "Min",   mn, true);
      drawCycler(cyMx,   maxR,  "Max",   mx, true);
    };
    drawCol(colSrc, "FROM (Source)", r.srcKind, r.srcChannel, r.srcData1, r.srcMin, r.srcMax,
            CY_M_SRC_TYPE, CY_M_SRC_CH, CY_M_SRC_DATA1, CY_M_SRC_MIN, CY_M_SRC_MAX, false);
    drawCol(colDst, "TO (Destination)", r.dstKind, r.dstChannel, r.dstData1, r.dstMin, r.dstMax,
            CY_M_DST_TYPE, CY_M_DST_CH, CY_M_DST_DATA1, CY_M_DST_MIN, CY_M_DST_MAX, true);
  }

  // ── Action buttons ──
  const char* actLabels[ACT_COUNT] = { "EN/DIS", "ADD", "DEL", "UP", "DOWN" };
  uint16_t    actColors[ACT_COUNT] = { COL_BTN, COL_NAVY, COL_DANGER, COL_NAVY, COL_NAVY };
  M5.Display.fillRoundRect(sideX, actionY, sideW, actionPanelH, 12, COL_PANEL);
  M5.Display.drawRoundRect(sideX, actionY, sideW, actionPanelH, 12, COL_BTN_BDR);
  M5.Display.setTextDatum(top_left);
  M5.Display.setFont(FONT_SMALL);
  M5.Display.setTextColor(COL_ACCENT, COL_PANEL);
  M5.Display.drawString("RULE ACTIONS", sideX + 14, actionY + 10);

  int innerX = sideX + 14;
  int innerY = actionY + 36;
  int innerW = sideW - 28;
  int btnGap = 10;
  int actW = (innerW - btnGap * (ACT_COUNT - 1)) / ACT_COUNT;
  int actH = actionPanelH - 46;

  for (int i = 0; i < ACT_COUNT; i++) {
    midiActionBtn[i] = { innerX + i * (actW + btnGap), innerY, actW, actH };
  }

  for (int i = 0; i < ACT_COUNT; i++) {
    drawRectBtn(midiActionBtn[i], actColors[i], COL_BTN_BDR, actLabels[i], COL_BTN_TXT, FONT_LARGE);
  }

  if (g_midiKeypadActive) {
    drawMidiKeypadOverlay();
  }
}

static void processMidiManageTouch(const m5::Touch_Class::touch_detail_t& td) {
  int tx = td.x;
  int ty = td.y;
  auto inR = [](const Rect& r, int x, int y) {
    return r.w > 0 && r.h > 0 && x >= r.x && x < r.x + r.w && y >= r.y && y < r.y + r.h;
  };
  // FILTER/MAPPER/BYPASS のタブは toolbar 側で処理しているのでここには来ない。

  if (g_midiKeypadActive) {
    for (int i = 0; i < MKP_BTN_COUNT; i++) {
      if (!inR(g_midiKeypadButtons[i], tx, ty)) continue;
      switch (i) {
        case MKP_BTN_1: case MKP_BTN_2: case MKP_BTN_3:
        case MKP_BTN_4: case MKP_BTN_5: case MKP_BTN_6:
        case MKP_BTN_7: case MKP_BTN_8: case MKP_BTN_9:
        case MKP_BTN_0: {
          int digit = (i == MKP_BTN_0) ? 0 : (i + 1);
          size_t len = strlen(g_midiKeypadDigits);
          if (len < sizeof(g_midiKeypadDigits) - 1) {
            int current = g_midiKeypadUseSpecial ? 0 : parseMidiKeypadDigits();
            int candidate = (len == 0 || (len == 1 && g_midiKeypadDigits[0] == '0'))
                              ? digit
                              : current * 10 + digit;
            if (candidate <= g_midiKeypadMaxValue) {
              if (len == 1 && g_midiKeypadDigits[0] == '0') {
                g_midiKeypadDigits[0] = '0' + digit;
                g_midiKeypadDigits[1] = '\0';
              } else {
                g_midiKeypadDigits[len] = '0' + digit;
                g_midiKeypadDigits[len + 1] = '\0';
              }
              g_midiKeypadUseSpecial = false;
            }
          }
          requestMidiManageRefresh();
          return;
        }
        case MKP_BTN_SPECIAL:
          if (g_midiKeypadSpecialLabel[0]) {
            g_midiKeypadUseSpecial = true;
            requestMidiManageRefresh();
          }
          return;
        case MKP_BTN_BS: {
          if (!g_midiKeypadUseSpecial) {
            size_t len = strlen(g_midiKeypadDigits);
            if (len > 0) g_midiKeypadDigits[len - 1] = '\0';
            requestMidiManageRefresh();
          }
          return;
        }
        case MKP_BTN_CANCEL:
          closeMidiKeypad();
          return;
        case MKP_BTN_OK:
          applyMidiKeypadValue();
          return;
        default:
          return;
      }
    }
    return;
  }

  // ── Rule list ──
  for (int slot = 0; slot < midiListVisible && slot < MAX_FILTER_RULES; slot++) {
    if (inR(midiRuleRows[slot], tx, ty)) {
      int i = midiListFirstRow + slot;
      int ruleCount = (midiManagePage == MIDI_PAGE_FILTER) ? midiFilterRuleCount : midiMapperRuleCount;
      if (i >= 0 && i < ruleCount) {
        if (midiManagePage == MIDI_PAGE_FILTER) midiSelectedFilterRule = i;
        else                                     midiSelectedMapperRule = i;
        requestMidiManageRefresh();
      }
      return;
    }
  }

  if (td.wasHold()) {
    for (int k = 0; k < CY_COUNT; k++) {
      if (!midiCyclerActive[k] || !midiCyclerSupportsKeypad((CyclerKind)k)) continue;
      if (inR(midiCyclerArea[k], tx, ty)) {
        openMidiKeypad((CyclerKind)k);
        return;
      }
    }
  }

  // ── Cyclers ──
  for (int k = 0; k < CY_COUNT; k++) {
    if (!midiCyclerActive[k]) continue;
    int delta = 0;
    if (inR(midiCyclerLeft[k], tx, ty))  delta = -1;
    else if (inR(midiCyclerRight[k], tx, ty)) delta = +1;
    if (delta == 0) continue;

    if (midiManagePage == MIDI_PAGE_FILTER) {
      MidiFilterRule& r = midiFilterRules[midiSelectedFilterRule];
      switch (k) {
        case CY_F_TYPE: cycleMidiKind(r.kind, delta);
                        if (!midiKindHasChannel(r.kind)) r.channel = -1;
                        break;
        case CY_F_CH:   cycleChannelValue(r.channel, delta); break;
        default: break;
      }
    } else {
      MidiMapperRule& r = midiMapperRules[midiSelectedMapperRule];
      int maxV = getMidiValueMax(r.srcKind);
      int maxD = getMidiValueMax(r.dstKind);
      switch (k) {
        case CY_M_SRC_TYPE:  cycleMidiKind(r.srcKind, delta);
                             if (!midiKindHasChannel(r.srcKind)) r.srcChannel = -1;
                             if (!midiKindSupportsData1(r.srcKind)) r.srcData1 = -1;
                             r.srcMax = std::min((int)r.srcMax, getMidiValueMax(r.srcKind));
                             break;
        case CY_M_SRC_CH:    cycleChannelValue(r.srcChannel, delta); break;
        case CY_M_SRC_DATA1: cycleData1Value(r.srcData1, delta); break;
        case CY_M_SRC_MIN:   cycleClampedValue(r.srcMin, delta, maxV);
                             if (r.srcMax < r.srcMin) r.srcMax = r.srcMin;
                             break;
        case CY_M_SRC_MAX:   cycleClampedValue(r.srcMax, delta, maxV);
                             if (r.srcMin > r.srcMax) r.srcMin = r.srcMax;
                             break;
        case CY_M_DST_TYPE:  cycleMidiKind(r.dstKind, delta);
                             if (!midiKindHasChannel(r.dstKind)) r.dstChannel = -1;
                             if (!midiKindSupportsData1(r.dstKind)) r.dstData1 = -1;
                             r.dstMax = std::min((int)r.dstMax, getMidiValueMax(r.dstKind));
                             break;
        case CY_M_DST_CH:    cycleChannelValue(r.dstChannel, delta); break;
        case CY_M_DST_DATA1: cycleData1Value(r.dstData1, delta); break;
        case CY_M_DST_MIN:   cycleClampedValue(r.dstMin, delta, maxD);
                             if (r.dstMax < r.dstMin) r.dstMax = r.dstMin;
                             break;
        case CY_M_DST_MAX:   cycleClampedValue(r.dstMax, delta, maxD);
                             if (r.dstMin > r.dstMax) r.dstMin = r.dstMax;
                             break;
        default: break;
      }
    }
    requestMidiManageRefresh();
    return;
  }

  // ── Action buttons ──
  for (int i = 0; i < ACT_COUNT; i++) {
    if (!inR(midiActionBtn[i], tx, ty)) continue;
    bool isFilter = (midiManagePage == MIDI_PAGE_FILTER);
    int& ruleCount = isFilter ? midiFilterRuleCount : midiMapperRuleCount;
    int& selected  = isFilter ? midiSelectedFilterRule : midiSelectedMapperRule;
    switch ((ActionKind)i) {
      case ACT_ENDIS:
        if (isFilter) midiFilterRules[selected].enabled = !midiFilterRules[selected].enabled;
        else          midiMapperRules[selected].enabled = !midiMapperRules[selected].enabled;
        break;
      case ACT_ADD:
        if (isFilter) addDefaultFilterRule();
        else          addDefaultMapperRule();
        break;
      case ACT_DEL:
        if (isFilter) deleteSelectedFilterRule();
        else          deleteSelectedMapperRule();
        break;
      case ACT_UP:
        if (selected > 0) {
          if (isFilter) std::swap(midiFilterRules[selected], midiFilterRules[selected - 1]);
          else          std::swap(midiMapperRules[selected], midiMapperRules[selected - 1]);
          selected--;
        }
        break;
      case ACT_DOWN:
        if (selected < ruleCount - 1) {
          if (isFilter) std::swap(midiFilterRules[selected], midiFilterRules[selected + 1]);
          else          std::swap(midiMapperRules[selected], midiMapperRules[selected + 1]);
          selected++;
        }
        break;
      default: break;
    }
    requestMidiManageRefresh();
    return;
  }
}

static void drawPianoKeyboard(const Rect& area, const uint8_t* velocityMap,
                              const char* title, const char* subtitle) {
  M5.Display.fillRoundRect(area.x, area.y, area.w, area.h, 12, COL_PANEL);
  M5.Display.drawRoundRect(area.x, area.y, area.w, area.h, 12, COL_BTN_BDR);

  Rect keysArea = area;
  keysArea.y += 28;
  keysArea.h -= 28;

  M5.Display.setFont(FONT_TINY);
  M5.Display.setTextColor(COL_MUTED, COL_PANEL);
  M5.Display.setTextDatum(top_left);
  if (title) M5.Display.drawString(title, area.x + 12, area.y + 6);
  M5.Display.setTextDatum(top_right);
  if (subtitle) M5.Display.drawString(subtitle, area.x + area.w - 12, area.y + 6);

  auto activeColor = [&](uint8_t vel, bool blackKey) {
    uint8_t r = blackKey ? 200 : 90 + vel;
    uint8_t g = blackKey ? 110 + (vel / 2) : 40 + vel;
    uint8_t b = blackKey ? 40 : 10;
    return M5.Display.color565(r, g, b);
  };

  for (int pass = 0; pass < 2; ++pass) {
    bool wantBlack = (pass == 1);
    for (int i = 0; i < PIANO_KEY_COUNT; ++i) {
      if (pianoKeys[i].isBlackKey != wantBlack) continue;
      const Rect& r = pianoKeys[i].r;
      uint8_t note = pianoKeys[i].note;
      uint8_t vel = velocityMap[note];
      uint16_t fill = pianoKeys[i].isBlackKey ? COL_BLACKKEY : COL_WHITEKEY;
      if (vel) fill = activeColor(vel, pianoKeys[i].isBlackKey);
      M5.Display.fillRect(r.x, r.y, r.w, r.h, fill);
      M5.Display.drawRect(r.x, r.y, r.w, r.h,
                          pianoKeys[i].isBlackKey ? TFT_DARKGREY : TFT_BLACK);
      if (!pianoKeys[i].isBlackKey && (note % 12) == 0) {
        char name[4];
        snprintf(name, sizeof(name), "C%d", (note / 12) - 1);
        M5.Display.setFont(FONT_TINY);
        M5.Display.setTextColor(vel ? TFT_BLACK : COL_MUTED, fill);
        M5.Display.setTextDatum(bottom_center);
        M5.Display.drawString(name, r.x + (r.w / 2), r.y + r.h - 6);
      }
    }
  }
}

static void initSmfMonitorGeometry(const Rect& area) {
  int labelW = 34;
  int bodyX = area.x + labelW;
  int bodyW = area.w - labelW;
  int rowH = max(12, area.h / 16);
  int blackH = max(5, (rowH * 46) / 100);

  auto isBlack = [](uint8_t note) {
    switch (note % 12) {
      case 1: case 3: case 6: case 8: case 10: return true;
      default: return false;
    }
  };

  int whiteCount = 0;
  for (uint8_t note = 0; note < 128; ++note) {
    if (!isBlack(note)) ++whiteCount;
  }
  int whiteW = max(3, bodyW / whiteCount);
  int blackW = max(3, (whiteW * 76) / 100);

  int whiteIdx = 0;
  int noteX[128] = {};
  for (uint8_t note = 0; note < 128; ++note) {
    if (!isBlack(note)) {
      noteX[note] = bodyX + whiteIdx * whiteW;
      ++whiteIdx;
    } else {
      noteX[note] = bodyX + whiteIdx * whiteW - blackW / 2;
    }
  }

  for (uint8_t ch = 0; ch < 16; ++ch) {
    int rowY = area.y + ch * rowH;
    for (uint8_t note = 0; note < 128; ++note) {
      bool black = isBlack(note);
      smfKeyGeom[ch][note].isBlackKey = black;
      smfKeyGeom[ch][note].r = {
        noteX[note],
        rowY,
        black ? blackW : whiteW + 1,
        black ? blackH : rowH - 1
      };
    }
  }
  smfMonitorGeometryReady = true;
}

static void invalidateSmfMonitorAll() {
  smfMonitorDirtyAll = true;
}

static void invalidateSmfMonitorKey(uint8_t channel, uint8_t note) {
  if (channel >= 16 || note >= 128) return;
  smfMonitorKeyDirty[channel][note] = true;
}

static void drawSmfMonitorBase(const Rect& area) {
  M5.Display.fillRect(area.x, area.y, area.w, area.h, 0x4208);
  if (!smfMonitorGeometryReady) initSmfMonitorGeometry(area);

  M5.Display.setFont(FONT_TINY);
  M5.Display.setTextDatum(middle_center);

  int rowH = max(12, area.h / 16);
  for (uint8_t ch = 0; ch < 16; ++ch) {
    int rowY = area.y + ch * rowH;
    int cy = rowY + rowH / 2;
    Rect labelRect = { area.x, rowY, 34, rowH };
    M5.Display.fillRect(labelRect.x, labelRect.y, labelRect.w, labelRect.h, 0x18C3);
    M5.Display.setTextColor(TFT_WHITE, 0x18C3);
    char label[4];
    snprintf(label, sizeof(label), "%02d", ch + 1);
    M5.Display.drawString(label, labelRect.x + labelRect.w / 2, cy);

    for (uint8_t note = 0; note < 128; ++note) {
      if (smfKeyGeom[ch][note].isBlackKey) continue;
      const Rect& r = smfKeyGeom[ch][note].r;
      M5.Display.fillRect(r.x, r.y, r.w, r.h, TFT_WHITE);
      M5.Display.drawRect(r.x, r.y, r.w, r.h, 0x8410);
    }
    for (uint8_t note = 0; note < 128; ++note) {
      if (!smfKeyGeom[ch][note].isBlackKey) continue;
      const Rect& r = smfKeyGeom[ch][note].r;
      M5.Display.fillRect(r.x, r.y, r.w, r.h, TFT_BLACK);
      M5.Display.drawRect(r.x, r.y, r.w, r.h, 0x4208);
    }
  }
}

static void drawSmfMonitorKey(uint8_t channel, uint8_t note) {
  if (channel >= 16 || note >= 128) return;
  const Rect& r = smfKeyGeom[channel][note].r;
  bool black = smfKeyGeom[channel][note].isBlackKey;
  uint8_t vel = smfChannelVelocity[channel][note];
  if (black) {
    uint16_t fill = vel ? TFT_RED : TFT_BLACK;
    uint16_t border = vel ? TFT_WHITE : 0x4208;
    M5.Display.fillRect(r.x, r.y, r.w, r.h, fill);
    M5.Display.drawRect(r.x, r.y, r.w, r.h, border);
    return;
  }

  int activeTop = r.y + max(2, (r.h * 74) / 100);
  int activeH = max(3, (r.y + r.h) - activeTop - 1);
  uint16_t fill = vel ? TFT_RED : TFT_WHITE;
  int activeW = max(3, (r.w * 76) / 100);
  int activeX = r.x + (r.w - activeW) / 2;
  M5.Display.fillRect(activeX, activeTop, activeW, activeH, fill);
  M5.Display.drawLine(r.x, r.y + r.h - 1, r.x + r.w - 1, r.y + r.h - 1, 0x8410);
}

static void flushSmfMonitorDirty(const Rect& area) {
  if (!smfMonitorGeometryReady) initSmfMonitorGeometry(area);
  if (smfMonitorDirtyAll) {
    drawSmfMonitorBase(area);
    for (uint8_t ch = 0; ch < 16; ++ch) {
      for (uint8_t note = 0; note < 128; ++note) {
        if (smfChannelVelocity[ch][note]) drawSmfMonitorKey(ch, note);
        smfMonitorKeyDirty[ch][note] = false;
      }
    }
    smfMonitorDirtyAll = false;
    return;
  }

  for (uint8_t ch = 0; ch < 16; ++ch) {
    for (uint8_t note = 0; note < 128; ++note) {
      if (!smfMonitorKeyDirty[ch][note]) continue;
      drawSmfMonitorKey(ch, note);
      smfMonitorKeyDirty[ch][note] = false;
    }
  }
}

static void drawSmfChannelKeyboard(const Rect& area) {
  flushSmfMonitorDirty(area);
}

static void drawMp3Static() {
  M5.Display.fillRect(contentArea.x, contentArea.y, contentArea.w, contentArea.h, COL_BG);
  M5.Display.fillRect(navArea.x, navArea.y, navArea.w, navArea.h, COL_BG);

  M5.Display.fillRoundRect(mp3ListArea.x, mp3ListArea.y, mp3ListArea.w, mp3ListArea.h, 12, COL_PANEL);
  M5.Display.drawRoundRect(mp3ListArea.x, mp3ListArea.y, mp3ListArea.w, mp3ListArea.h, 12, COL_BTN_BDR);
  M5.Display.setFont(FONT_SMALL);
  M5.Display.setTextColor(COL_TITLE, COL_PANEL);
  M5.Display.setTextDatum(top_left);
  M5.Display.drawString("MP3 Playlist", mp3ListArea.x + 12, mp3ListArea.y + 14);
  drawDoubleTriangleBtn(mp3ListPageUpBtn,   TFT_BLUE, COL_BTN_BDR, true,  COL_BTN_TXT);
  drawTriangleBtn(mp3ListUpBtn,             TFT_BLUE, COL_BTN_BDR, true,  COL_BTN_TXT);
  drawTriangleBtn(mp3ListDownBtn,           TFT_BLUE, COL_BTN_BDR, false, COL_BTN_TXT);
  drawDoubleTriangleBtn(mp3ListPageDownBtn, TFT_BLUE, COL_BTN_BDR, false, COL_BTN_TXT);

  int lineH = 44;
  int top = mp3ListArea.y + 84;
  int listRight = mp3ListArea.x + mp3ListArea.w - 12;
  int visible = (mp3ListArea.h - 84) / lineH;
  // Only auto-follow the current track when there *is* one. After folder
  // navigation mp3CurrentTrack == -1 (no selection); applying the follow
  // logic in that state would clamp mp3ListScroll back to 0 on every
  // redraw and silently undo the user's page-jump taps.
  if (mp3CurrentTrack >= 0) {
    if (mp3CurrentTrack < mp3ListScroll) mp3ListScroll = mp3CurrentTrack;
    if (mp3CurrentTrack >= mp3ListScroll + visible) mp3ListScroll = mp3CurrentTrack - visible + 1;
  }
  if (mp3ListScroll < 0) mp3ListScroll = 0;
  int mp3MaxScroll = max(0, mp3PlaylistCount - visible);
  if (mp3ListScroll > mp3MaxScroll) mp3ListScroll = mp3MaxScroll;
  for (int row = 0; row < visible; ++row) {
    int idx = mp3ListScroll + row;
    if (idx >= mp3PlaylistCount) break;
    Rect rr = { mp3ListArea.x + 10, top + row * lineH, mp3ListArea.w - 20, lineH - 2 };
    rr.w = listRight - rr.x;
    bool on = (idx == mp3CurrentTrack);
    M5.Display.fillRoundRect(rr.x, rr.y, rr.w, rr.h, 6, on ? COL_BTN_HI2 : COL_PANEL);
    char labelBuf[PLAYLIST_PATH_MAX + 4];
    const char* name = formatPlaylistLabel(mp3Playlist[idx], labelBuf, sizeof(labelBuf));
    bool isNav = !entryIsFile(mp3Playlist[idx]);
    uint16_t txt = on ? COL_BTN_TXT_HI : (isNav ? COL_ACCENT : COL_BTN_TXT);
    drawTextFit(rr, name, FONT_SMALL, txt, on ? COL_BTN_HI2 : COL_PANEL);
  }

  M5.Display.fillRoundRect(mp3InfoArea.x, mp3InfoArea.y, mp3InfoArea.w, mp3InfoArea.h, 12, COL_PANEL);
  M5.Display.drawRoundRect(mp3InfoArea.x, mp3InfoArea.y, mp3InfoArea.w, mp3InfoArea.h, 12, COL_BTN_BDR);
  Rect nameRow   = { mp3InfoArea.x + 10, mp3InfoArea.y + 8,  mp3InfoArea.w - 20, 28 };
  Rect titleRow  = { mp3InfoArea.x + 10, mp3InfoArea.y + 38, mp3InfoArea.w - 20, 22 };
  Rect artistRow = { mp3InfoArea.x + 10, mp3InfoArea.y + 62, mp3InfoArea.w - 20, 22 };
  Rect volRow    = { mp3InfoArea.x + 10, mp3InfoArea.y + 86, mp3InfoArea.w - 20, 22 };
  drawTextFit(nameRow, mp3CurrentName[0] ? mp3CurrentName : "No MP3 file loaded",
              FONT_SMALL, COL_VALUE, COL_PANEL);
  drawTextFit(titleRow, mp3Title[0] ? mp3Title : "Title: -",
              FONT_TINY, COL_BTN_TXT, COL_PANEL);
  drawTextFit(artistRow, mp3Artist[0] ? mp3Artist : "Artist: -",
              FONT_TINY, COL_BTN_TXT, COL_PANEL);
  char volLine[32];
  snprintf(volLine, sizeof(volLine), "Volume %d%%", (mp3Volume * 100) / 255);
  drawTextFit(volRow, volLine, FONT_TINY, COL_BTN_TXT, COL_PANEL);

  M5.Display.fillRoundRect(mp3VisualArea.x, mp3VisualArea.y, mp3VisualArea.w, mp3VisualArea.h, 12, COL_PANEL);
  M5.Display.drawRoundRect(mp3VisualArea.x, mp3VisualArea.y, mp3VisualArea.w, mp3VisualArea.h, 12, COL_BTN_BDR);
}

static void drawMp3Visual() {
  // Body fills the panel beneath the "Cassette" title and stretches to the
  // panel edges so the cassette image uses the full width and height.
  int titleH = 28;
  Rect body = { mp3VisualArea.x + 8, mp3VisualArea.y + titleH,
                mp3VisualArea.w - 16, mp3VisualArea.h - titleH - 10 };
  if (!mp3StaticDirty) {
    M5.Display.fillRect(body.x, body.y, body.w, body.h, COL_PANEL);
  } else {
    M5.Display.fillRoundRect(mp3VisualArea.x, mp3VisualArea.y, mp3VisualArea.w, mp3VisualArea.h, 12, COL_PANEL);
    M5.Display.drawRoundRect(mp3VisualArea.x, mp3VisualArea.y, mp3VisualArea.w, mp3VisualArea.h, 12, COL_BTN_BDR);
  }

  M5.Display.setFont(FONT_SMALL);
  M5.Display.setTextColor(COL_ACCENT, COL_PANEL);
  M5.Display.setTextDatum(top_center);
  M5.Display.drawString("Cassette", mp3VisualArea.x + mp3VisualArea.w / 2, mp3VisualArea.y + 8);

  int caseX = body.x;
  int caseY = body.y;
  int caseW = body.w;
  int caseH = body.h;
  M5.Display.drawRoundRect(caseX,     caseY,     caseW,     caseH,     18, COL_BTN_BDR);
  M5.Display.drawRoundRect(caseX + 1, caseY + 1, caseW - 2, caseH - 2, 17, COL_BTN_BDR);

  int leftX  = caseX + caseW / 4;
  int rightX = caseX + caseW * 3 / 4;
  int cy     = caseY + caseH / 2;
  int reelR  = min(caseW * 22 / 100, caseH * 38 / 100);
  int spokeCount = 3;

  for (int i = 0; i < 2; ++i) {
    int ox = (i == 0) ? leftX : rightX;
    M5.Display.drawCircle(ox, cy, reelR,      TFT_WHITE);
    M5.Display.drawCircle(ox, cy, reelR - 10, TFT_LIGHTGREY);
    for (int s = 0; s < spokeCount; ++s) {
      float angle = mp3CassetteAngle + (2.0f * PI * s / spokeCount);
      int px = ox + (int)(cosf(angle) * (reelR - 10));
      int py = cy + (int)(sinf(angle) * (reelR - 10));
      M5.Display.drawLine(ox, cy, px, py, TFT_WHITE);
    }
  }
  M5.Display.drawLine(leftX, cy - reelR, rightX, cy - reelR, TFT_LIGHTGREY);

  mp3VisualDirty = false;
  mp3StaticDirty = false;
}

// =================================================================
//  SRC mode rendering
// =================================================================
static bool srcNoteIsBlack(uint8_t note) {
  switch (note % 12) {
    case 1: case 3: case 6: case 8: case 10: return true;
    default: return false;
  }
}

// Per-pitch-class colour: 12 distinct hues stepping through the colour wheel
// so neighbouring keys are easy to tell apart, while the same pitch in
// different octaves shares a colour. Computed once with HSV→RGB565 for the
// 12 chromatic classes (note % 12 indexes into the table).
static uint16_t srcPitchColor(uint8_t note) {
  static uint16_t cache[12] = {0};
  static bool inited = false;
  if (!inited) {
    for (int pc = 0; pc < 12; ++pc) {
      // hue 0..6 (HSV sextant), s=v=1 → bright saturated colours
      float h = (float)pc * 6.0f / 12.0f;
      int   i = (int)h;
      float f = h - i;
      float p = 0.0f;
      float q = 1.0f - f;
      float t = f;
      float r=0, g=0, b=0;
      switch (i) {
        case 0: r=1; g=t; b=p; break;
        case 1: r=q; g=1; b=p; break;
        case 2: r=p; g=1; b=t; break;
        case 3: r=p; g=q; b=1; break;
        case 4: r=t; g=p; b=1; break;
        default:r=1; g=p; b=q; break;
      }
      uint8_t r8 = (uint8_t)(r * 255);
      uint8_t g8 = (uint8_t)(g * 255);
      uint8_t b8 = (uint8_t)(b * 255);
      cache[pc] = M5.Display.color565(r8, g8, b8);
    }
    inited = true;
  }
  return cache[note % 12];
}

// Compute the 88-key piano geometry inside srcKeyboardArea. White keys tile
// the width evenly; black keys overlap on top, narrower and shorter.
static void srcInitKeyboardGeometry() {
  int x0 = srcKeyboardArea.x + 8;
  int y0 = srcKeyboardArea.y + 4;
  int w  = srcKeyboardArea.w - 16;
  int h  = srcKeyboardArea.h - 8;
  if (w < 88 || h < 30) { srcKeyGeomReady = false; return; }

  // 52 white keys in 88-key range (21..108).
  int whiteCount = 0;
  for (int n = SRC_KEY_LO; n <= SRC_KEY_HI; ++n) if (!srcNoteIsBlack((uint8_t)n)) ++whiteCount;
  int whiteW = w / whiteCount;
  int blackW = (whiteW * 60) / 100;
  if (blackW < 4) blackW = 4;
  int blackH = (h * 60) / 100;

  for (int n = 0; n < 128; ++n) {
    srcKeyGeom[n].isBlack = srcNoteIsBlack((uint8_t)n);
    srcKeyGeom[n].r = { 0, 0, 0, 0 };
  }
  int whiteIdx = 0;
  for (int n = SRC_KEY_LO; n <= SRC_KEY_HI; ++n) {
    bool black = srcNoteIsBlack((uint8_t)n);
    if (!black) {
      srcKeyGeom[n].r = { x0 + whiteIdx * whiteW, y0, whiteW + 1, h };
      ++whiteIdx;
    } else {
      // Black key sits in the gap to the right of the previous white key.
      int prevWhiteRightX = x0 + whiteIdx * whiteW;
      srcKeyGeom[n].r = { prevWhiteRightX - blackW / 2, y0, blackW, blackH };
    }
  }
  srcKeyGeomReady = true;
}

static void srcKeyboardClearVisualState() {
  // Forces a full redraw next time drawSrc runs. Call when the active channel
  // changes so the displayed keyboard reflects the new channel.
  srcKeyAllDirty = true;
}

static void srcDrawSingleKey(uint8_t note) {
  if (note < SRC_KEY_LO || note > SRC_KEY_HI) return;
  if (!srcKeyGeomReady) return;
  const Rect& r = srcKeyGeom[note].r;
  bool black = srcKeyGeom[note].isBlack;
  uint8_t vel = srcKeyVel[srcChannel][note];
  uint16_t fill;
  if (vel) {
    // Lit — colour by pitch class so each note in the chord stands out.
    fill = srcPitchColor(note);
  } else {
    fill = black ? TFT_BLACK : TFT_WHITE;
  }
  M5.Display.fillRect(r.x, r.y, r.w, r.h, fill);
  M5.Display.drawRect(r.x, r.y, r.w, r.h, black ? 0x4208 : 0x8410);
}

// Full redraw of the SRC keyboard panel (background + all keys).
static void srcDrawKeyboardFull() {
  if (!srcKeyGeomReady) srcInitKeyboardGeometry();
  if (!srcKeyGeomReady) return;
  M5.Display.fillRoundRect(srcKeyboardArea.x, srcKeyboardArea.y,
                           srcKeyboardArea.w, srcKeyboardArea.h, 10, 0x4208);
  M5.Display.drawRoundRect(srcKeyboardArea.x, srcKeyboardArea.y,
                           srcKeyboardArea.w, srcKeyboardArea.h, 10, COL_BTN_BDR);
  // White keys first so black keys overlap on top.
  for (int n = SRC_KEY_LO; n <= SRC_KEY_HI; ++n) {
    if (!srcKeyGeom[n].isBlack) srcDrawSingleKey((uint8_t)n);
  }
  for (int n = SRC_KEY_LO; n <= SRC_KEY_HI; ++n) {
    if (srcKeyGeom[n].isBlack) srcDrawSingleKey((uint8_t)n);
  }
  // Clear all dirty flags (full repaint already covered them).
  for (int n = 0; n < 128; ++n) srcKeyDirty[srcChannel][n] = false;
  srcKeyAllDirty = false;
}

// Incremental: redraw only keys flagged dirty for the current channel.
static void srcFlushKeyboardDirty() {
  if (!srcKeyGeomReady) return;
  // Always need the white-then-black ordering when a black-adjacent white
  // toggles, so redraw the white first then any dirty black on top.
  for (int n = SRC_KEY_LO; n <= SRC_KEY_HI; ++n) {
    if (!srcKeyGeom[n].isBlack && srcKeyDirty[srcChannel][n]) {
      srcDrawSingleKey((uint8_t)n);
      srcKeyDirty[srcChannel][n] = false;
      // Repaint adjacent black keys so their borders aren't clobbered.
      if (n > 0 && srcKeyGeom[n - 1].isBlack)   srcDrawSingleKey((uint8_t)(n - 1));
      if (n < 127 && srcKeyGeom[n + 1].isBlack) srcDrawSingleKey((uint8_t)(n + 1));
    }
  }
  for (int n = SRC_KEY_LO; n <= SRC_KEY_HI; ++n) {
    if (srcKeyGeom[n].isBlack && srcKeyDirty[srcChannel][n]) {
      srcDrawSingleKey((uint8_t)n);
      srcKeyDirty[srcChannel][n] = false;
    }
  }
}

// Append a finished note to the piano-roll history ring buffer.
static void srcRollAppend(uint8_t note, uint8_t vel, uint8_t channel, uint32_t startMs, uint32_t endMs) {
  int slot = srcRollHistCount % SRC_ROLL_HISTORY;
  srcRollHist[slot] = { note, vel, channel, startMs, endMs };
  ++srcRollHistCount;
}

// Update srcKeyVel from a parsed MIDI message. Called by handleParsedMidiMessage.
static void srcKeyboardOnNote(int channel, int note, int velocity, bool noteOn) {
  if (channel < 0 || channel >= 16) return;
  if (note < 0 || note >= 128) return;
  uint8_t v = (uint8_t)((noteOn && velocity > 0) ? velocity : 0);

  // Piano-roll bookkeeping: every Note On stamps a start-time; every Note Off
  // closes the interval and pushes it to the history ring.
  uint32_t now = millis();
  if (v > 0) {
    if (srcNoteStartMs[channel][note] == 0) {
      srcNoteStartMs[channel][note] = (now == 0) ? 1 : now;  // 0 means "not held"
    }
  } else {
    uint32_t s = srcNoteStartMs[channel][note];
    if (s != 0) {
      srcRollAppend((uint8_t)note, srcKeyVel[channel][note], (uint8_t)channel, s, now);
      srcNoteStartMs[channel][note] = 0;
    }
  }

  if (srcKeyVel[channel][note] == v) return;
  srcKeyVel[channel][note] = v;
  srcKeyDirty[channel][note] = true;
  if (currentApp == APP_PLAY && currentPlay == PLAY_SRC && (uint8_t)channel == srcChannel) {
    // The next UI tick's partial-redraw branch will flush keys for SRC mode.
    needPartialUpdate = true;
  }
}

// Map a millisecond age (now - eventMs) to a Y coordinate inside the piano-
// roll panel. Now is at the bottom; older events scroll upward and disappear
// off the top once they age past SRC_ROLL_T_MS.
static int srcRollYForAge(uint32_t ageMs) {
  if (ageMs > SRC_ROLL_T_MS) ageMs = SRC_ROLL_T_MS;
  int h = srcRollArea.h - 4;
  int yBottom = srcRollArea.y + srcRollArea.h - 2;
  return yBottom - (int)((uint64_t)ageMs * h / SRC_ROLL_T_MS);
}

// Draw one note bar inside the roll. xCol is the centre x of the matching
// keyboard column below; we use a narrow bar (whiteW or blackW) to align.
static void srcRollDrawBar(uint8_t note, uint32_t startMs, uint32_t endMs, uint32_t now) {
  if (note < SRC_KEY_LO || note > SRC_KEY_HI) return;
  if (!srcKeyGeomReady) return;
  if (endMs <= now - SRC_ROLL_T_MS) return;            // entirely off the top
  if (startMs > now) startMs = now;                    // future-clamp (shouldn't happen)
  uint32_t ageStart = (now > startMs) ? (now - startMs) : 0;
  uint32_t ageEnd   = (now > endMs)   ? (now - endMs)   : 0;
  int yStart = srcRollYForAge(ageEnd);                 // newer end → lower y → higher on screen
  int yEnd   = srcRollYForAge(ageStart);               // older start → higher y? Actually
  // age 0 → bottom, age SRC_ROLL_T_MS → top. The note's lifetime: youngest
  // edge is endMs (smallest age), oldest edge is startMs (largest age). So
  // yTop is at age=ageStart, yBottom is at age=ageEnd. Swap and clip:
  if (yStart > yEnd) { int t = yStart; yStart = yEnd; yEnd = t; }
  // Clip to the same interior region that the per-tick fillRect clears
  // (inset 4 px from the panel edges) so leftover bar pixels at the very top
  // and bottom of the panel can't survive between repaints.
  const int inset = 4;
  int areaTop = srcRollArea.y + inset;
  int areaBot = srcRollArea.y + srcRollArea.h - inset;
  if (yEnd   <= areaTop) return;
  if (yStart >= areaBot) return;
  if (yStart < areaTop) yStart = areaTop;
  if (yEnd   > areaBot) yEnd   = areaBot;
  if (yEnd <= yStart) yEnd = yStart + 1;

  const Rect& kr = srcKeyGeom[note].r;
  int colX = kr.x;
  int colW = kr.w;
  // Black-key roll bars are slightly inset so they don't smear over their
  // neighbouring white-key bars.
  if (srcKeyGeom[note].isBlack) { colX += 1; colW = max(2, colW - 2); }

  // Same per-pitch-class colour as the lit key, so a held note's bar matches
  // the key it's coming from below.
  uint16_t fill = srcPitchColor(note);
  M5.Display.fillRect(colX, yStart, colW, yEnd - yStart, fill);
}

// Draw the static panel border for the piano roll. Called once on full
// redraw (drawSrc) — NOT on every tick. Per-tick scrolling only repaints
// the interior so the rounded edge doesn't flicker.
static void srcDrawRollFrame() {
  M5.Display.fillRoundRect(srcRollArea.x, srcRollArea.y,
                           srcRollArea.w, srcRollArea.h, 8, 0x10A2);
  M5.Display.drawRoundRect(srcRollArea.x, srcRollArea.y,
                           srcRollArea.w, srcRollArea.h, 8, COL_BTN_BDR);
}

// Per-tick incremental repaint of the piano roll's interior. The panel border
// drawn by srcDrawRollFrame() is left untouched so it stays put while the
// notes inside scroll upward. Called every UI tick (~50 Hz).
static void srcDrawRoll() {
  // Inset by 4 px from each edge so we never write over the rounded border.
  const int inset = 4;
  M5.Display.fillRect(srcRollArea.x + inset, srcRollArea.y + inset,
                      srcRollArea.w - inset * 2, srcRollArea.h - inset * 2,
                      0x10A2);

  if (!srcKeyGeomReady) return;
  uint32_t now = millis();
  uint32_t cutoff = (now > SRC_ROLL_T_MS) ? (now - SRC_ROLL_T_MS) : 0;

  // Finished notes from the ring buffer — active channel only.
  int n = (srcRollHistCount < SRC_ROLL_HISTORY) ? srcRollHistCount : SRC_ROLL_HISTORY;
  for (int i = 0; i < n; ++i) {
    int slot = (srcRollHistCount < SRC_ROLL_HISTORY) ? i
              : (srcRollHistCount + i) % SRC_ROLL_HISTORY;
    const SrcRollEvent& e = srcRollHist[slot];
    if (e.channel != srcChannel) continue;
    if (e.endMs < cutoff) continue;
    srcRollDrawBar(e.note, e.startMs, e.endMs, now);
  }

  // Currently-held notes on the active channel — extend bar to "now"
  for (int note = SRC_KEY_LO; note <= SRC_KEY_HI; ++note) {
    uint32_t s = srcNoteStartMs[srcChannel][note];
    if (s == 0) continue;
    srcRollDrawBar((uint8_t)note, s, now, now);
  }
}

static int srcCatalogCount() {
  return (srcChannel == SRC_DRUM_CHANNEL) ? kGmDrumKitCount : 128;
}

static const char* srcCatalogNameAt(int index) {
  if (srcChannel == SRC_DRUM_CHANNEL) {
    if (index < 0 || index >= kGmDrumKitCount) return "";
    return kGmDrumKits[index].name;
  }
  if (index < 0 || index >= 128) return "";
  return kGmInstrumentNames[index];
}

static int srcCatalogProgramAt(int index) {
  if (srcChannel == SRC_DRUM_CHANNEL) {
    if (index < 0 || index >= kGmDrumKitCount) return 0;
    return kGmDrumKits[index].program;
  }
  return index;  // melodic GM: index == program number
}

static int srcCurrentCatalogIndex() {
  if (srcChannel == SRC_DRUM_CHANNEL) return gmDrumKitIndex(srcProgram[srcChannel]);
  return srcProgram[srcChannel];
}

static const char* srcCurrentProgramName() {
  if (srcChannel == SRC_DRUM_CHANNEL) return gmDrumKitName(srcProgram[srcChannel]);
  return gmInstrumentName(srcProgram[srcChannel]);
}

static void drawSrc() {
  M5.Display.fillRect(contentArea.x, contentArea.y, contentArea.w, contentArea.h, COL_BG);
  M5.Display.fillRect(navArea.x, navArea.y, navArea.w, navArea.h, COL_BG);
  srcDirtyAll = false;

  // ---- Channel strip ----
  for (int i = 0; i < 16; ++i) {
    bool sel = (i == srcChannel);
    bool drum = (i == SRC_DRUM_CHANNEL);
    uint16_t bg  = sel ? COL_BTN_HI : (drum ? COL_BTN_HI2 : COL_BTN);
    uint16_t txt = sel ? COL_BTN_TXT_HI : COL_BTN_TXT;
    char lab[4]; snprintf(lab, sizeof(lab), "%02d", i + 1);
    drawRectBtn(srcChannelCells[i], bg, COL_BTN_BDR, lab, txt, FONT_SMALL);
  }

  // ---- Program header (PRG- | name | PRG+) ----
  drawRectBtn(srcPrgDownBtn, TFT_BLUE, COL_BTN_BDR, "PRG-", COL_BTN_TXT, FONT_MED);
  drawRectBtn(srcPrgUpBtn,   TFT_BLUE, COL_BTN_BDR, "PRG+", COL_BTN_TXT, FONT_MED);
  M5.Display.fillRoundRect(srcProgramHeader.x, srcProgramHeader.y,
                           srcProgramHeader.w, srcProgramHeader.h, 12, COL_PANEL);
  M5.Display.drawRoundRect(srcProgramHeader.x, srcProgramHeader.y,
                           srcProgramHeader.w, srcProgramHeader.h, 12, COL_BTN_BDR);
  M5.Display.setFont(FONT_LARGE);
  M5.Display.setTextColor(COL_TITLE, COL_PANEL);
  M5.Display.setTextDatum(middle_left);
  char prgBuf[64];
  if (srcChannel == SRC_DRUM_CHANNEL) {
    snprintf(prgBuf, sizeof(prgBuf), "DRUM:%03u  %s",
             (unsigned)srcProgram[srcChannel], srcCurrentProgramName());
  } else {
    snprintf(prgBuf, sizeof(prgBuf), "PRG:%03u  %s",
             (unsigned)(srcProgram[srcChannel] + 1), srcCurrentProgramName());
  }
  M5.Display.drawString(prgBuf, srcProgramHeader.x + 16,
                        srcProgramHeader.y + srcProgramHeader.h / 2);

  // ---- Piano roll + 88-key live monitor ----
  // The "Ch nn live monitor" header is gone: the channel strip above already
  // shows which channel is active, and the colour-coded roll/keys are
  // self-explanatory.
  // Frame is drawn ONCE here; per-tick scrolling refills only the interior so
  // the rounded border doesn't flicker as bars move.
  if (!srcKeyGeomReady) srcInitKeyboardGeometry();
  srcDrawKeyboardFull();
  srcDrawRollFrame();
  srcDrawRoll();

  // ---- VOL / PB / SUS bottom row ----
  drawRectBtn(srcVolDownBtn, COL_BTN, COL_BTN_BDR, "VOL-", COL_BTN_TXT, FONT_MED);
  drawRectBtn(srcVolUpBtn,   COL_BTN, COL_BTN_BDR, "VOL+", COL_BTN_TXT, FONT_MED);
  M5.Display.fillRoundRect(srcVolLabel.x, srcVolLabel.y, srcVolLabel.w, srcVolLabel.h, 10, COL_PANEL);
  M5.Display.drawRoundRect(srcVolLabel.x, srcVolLabel.y, srcVolLabel.w, srcVolLabel.h, 10, COL_BTN_BDR);
  M5.Display.setFont(FONT_MED);
  M5.Display.setTextColor(COL_TITLE, COL_PANEL);
  M5.Display.setTextDatum(middle_center);
  char vbuf[16]; snprintf(vbuf, sizeof(vbuf), "VOL:%u", (unsigned)srcVolume[srcChannel]);
  M5.Display.drawString(vbuf, srcVolLabel.x + srcVolLabel.w / 2,
                        srcVolLabel.y + srcVolLabel.h / 2);

  drawRectBtn(srcPbDownBtn, COL_BTN, COL_BTN_BDR, "PB-",  COL_BTN_TXT, FONT_MED);
  drawRectBtn(srcPbUpBtn,   COL_BTN, COL_BTN_BDR, "PB+",  COL_BTN_TXT, FONT_MED);
  M5.Display.fillRoundRect(srcPbLabel.x, srcPbLabel.y, srcPbLabel.w, srcPbLabel.h, 10, COL_PANEL);
  M5.Display.drawRoundRect(srcPbLabel.x, srcPbLabel.y, srcPbLabel.w, srcPbLabel.h, 10, COL_BTN_BDR);
  char pbuf[24];
  int pbDelta = (int)srcPitchBend[srcChannel] - 8192;
  snprintf(pbuf, sizeof(pbuf), "PB:%+d", pbDelta);
  M5.Display.drawString(pbuf, srcPbLabel.x + srcPbLabel.w / 2,
                        srcPbLabel.y + srcPbLabel.h / 2);

  bool sus = srcSustain[srcChannel];
  drawRectBtn(srcSusBtn, sus ? COL_BTN_HI : COL_BTN, COL_BTN_BDR,
              sus ? "SUS ON" : "SUS OFF",
              sus ? COL_BTN_TXT_HI : COL_BTN_TXT, FONT_MED);
}

static void handleSrcTouch(int x, int y) {
  // Channel strip — manual select disables auto-follow so the user's pick
  // sticks even while live MIDI is flowing on a different channel.
  for (int i = 0; i < 16; ++i) {
    if (hit(srcChannelCells[i], x, y)) {
      if (srcChannel != (uint8_t)i) {
        srcChannel = (uint8_t)i;
        srcAutoFollow = false;
        // Re-send program so the synth reflects the cached selection on the
        // newly-active channel even if it was previously silent.
        srcSendCurrentProgram(srcChannel);
        sendUnitCC(srcChannel, 7, srcVolume[srcChannel]);
        srcKeyboardClearVisualState();
        srcDirtyAll = true;
        needFullRedraw = true;
      }
      return;
    }
  }

  // Program header ± buttons
  if (hit(srcPrgDownBtn, x, y)) {
    int total = srcCatalogCount();
    int cur = srcCurrentCatalogIndex();
    int next = (cur - 1 + total) % total;
    srcSetProgram(srcChannel, (uint8_t)srcCatalogProgramAt(next));
    needFullRedraw = true;
    return;
  }
  if (hit(srcPrgUpBtn, x, y)) {
    int total = srcCatalogCount();
    int cur = srcCurrentCatalogIndex();
    int next = (cur + 1) % total;
    srcSetProgram(srcChannel, (uint8_t)srcCatalogProgramAt(next));
    needFullRedraw = true;
    return;
  }

  // Tap on the program-name banner opens the fullscreen instrument picker.
  if (hit(srcProgramHeader, x, y)) {
    srcPickerOpen();
    return;
  }

  // VOL / PB / SUS buttons
  if (hit(srcVolDownBtn, x, y)) {
    int v = (int)srcVolume[srcChannel] - 4; if (v < 0) v = 0;
    srcSetVolume(srcChannel, (uint8_t)v); needPartialUpdate = true; needFullRedraw = true; return;
  }
  if (hit(srcVolUpBtn, x, y)) {
    int v = (int)srcVolume[srcChannel] + 4; if (v > 127) v = 127;
    srcSetVolume(srcChannel, (uint8_t)v); needFullRedraw = true; return;
  }
  if (hit(srcPbDownBtn, x, y)) {
    int v = (int)srcPitchBend[srcChannel] - 512; if (v < 0) v = 0;
    srcSetPitchBend(srcChannel, (uint16_t)v); needFullRedraw = true; return;
  }
  if (hit(srcPbUpBtn, x, y)) {
    int v = (int)srcPitchBend[srcChannel] + 512; if (v > 16383) v = 16383;
    srcSetPitchBend(srcChannel, (uint16_t)v); needFullRedraw = true; return;
  }
  if (hit(srcSusBtn, x, y)) {
    srcSetSustain(srcChannel, !srcSustain[srcChannel]);
    needFullRedraw = true;
    return;
  }
}

// =================================================================
//  SRC fullscreen instrument picker
// =================================================================
static void srcPickerOpen() {
  // Snap to the page that contains the current selection so the user sees
  // their pick highlighted on entry.
  int cur = srcCurrentCatalogIndex();
  g_srcPickerPage = cur / SRC_PICKER_PERPAGE;
  g_srcPickerOpen = true;
  needFullRedraw = true;
}

static void srcPickerClose() {
  g_srcPickerOpen = false;
  needFullRedraw = true;
}

static int srcPickerPageCount() {
  int total = srcCatalogCount();
  return (total + SRC_PICKER_PERPAGE - 1) / SRC_PICKER_PERPAGE;
}

static void drawSrcPicker() {
  M5.Display.fillScreen(COL_BG);

  // Title
  M5.Display.setFont(FONT_LARGE);
  M5.Display.setTextColor(COL_TITLE, COL_BG);
  M5.Display.setTextDatum(middle_left);
  char title[64];
  int pageCount = srcPickerPageCount();
  if (srcChannel == SRC_DRUM_CHANNEL) {
    snprintf(title, sizeof(title), "Drum Kits — Ch10  (page %d/%d)",
             g_srcPickerPage + 1, pageCount);
  } else {
    snprintf(title, sizeof(title), "GM Instruments — Ch%02u  (page %d/%d)",
             (unsigned)(srcChannel + 1), g_srcPickerPage + 1, pageCount);
  }
  M5.Display.drawString(title, 32, 24 + 30);

  // Cells
  int total = srcCatalogCount();
  int curIdx = srcCurrentCatalogIndex();
  int base = g_srcPickerPage * SRC_PICKER_PERPAGE;
  for (int i = 0; i < SRC_PICKER_PERPAGE; ++i) {
    int idx = base + i;
    Rect& r = srcPickerCells[i];
    if (idx >= total) {
      // Empty slot — paint a faint placeholder so the grid stays uniform.
      M5.Display.fillRoundRect(r.x, r.y, r.w, r.h, 12, COL_BG);
      M5.Display.drawRoundRect(r.x, r.y, r.w, r.h, 12, 0x18C3);
      continue;
    }
    bool sel = (idx == curIdx);
    uint16_t bg  = sel ? COL_BTN_HI2 : COL_PANEL;
    uint16_t txt = sel ? COL_BTN_TXT_HI : COL_BTN_TXT;
    M5.Display.fillRoundRect(r.x, r.y, r.w, r.h, 12, bg);
    M5.Display.drawRoundRect(r.x, r.y, r.w, r.h, 12, COL_BTN_BDR);
    int prgVal = srcCatalogProgramAt(idx);
    char line[80];
    if (srcChannel == SRC_DRUM_CHANNEL) {
      snprintf(line, sizeof(line), "%03d  %s", prgVal, srcCatalogNameAt(idx));
    } else {
      snprintf(line, sizeof(line), "%03d  %s", prgVal + 1, srcCatalogNameAt(idx));
    }
    drawTextFit(r, line, FONT_MED, txt, bg, 12);
  }

  // Footer
  drawRectBtn(srcPickerPrevBtn,  TFT_BLUE,    COL_BTN_BDR, "< PAGE", COL_BTN_TXT, FONT_LARGE);
  drawRectBtn(srcPickerNextBtn,  TFT_BLUE,    COL_BTN_BDR, "PAGE >", COL_BTN_TXT, FONT_LARGE);
  drawRectBtn(srcPickerCloseBtn, COL_DANGER,  COL_BTN_BDR, "CLOSE",  COL_BTN_TXT, FONT_LARGE);
}

static void handleSrcPickerTouch(int x, int y) {
  // Footer first
  if (hit(srcPickerCloseBtn, x, y)) { srcPickerClose(); return; }
  int pageCount = srcPickerPageCount();
  if (hit(srcPickerPrevBtn, x, y)) {
    g_srcPickerPage = (g_srcPickerPage + pageCount - 1) % pageCount;
    needFullRedraw = true;
    return;
  }
  if (hit(srcPickerNextBtn, x, y)) {
    g_srcPickerPage = (g_srcPickerPage + 1) % pageCount;
    needFullRedraw = true;
    return;
  }
  // Cells
  int total = srcCatalogCount();
  int base = g_srcPickerPage * SRC_PICKER_PERPAGE;
  for (int i = 0; i < SRC_PICKER_PERPAGE; ++i) {
    int idx = base + i;
    if (idx >= total) break;
    if (hit(srcPickerCells[i], x, y)) {
      srcSetProgram(srcChannel, (uint8_t)srcCatalogProgramAt(idx));
      srcPickerClose();
      return;
    }
  }
}

static void drawSmf() {
  M5.Display.fillRect(contentArea.x, contentArea.y, contentArea.w, contentArea.h, COL_BG);
  M5.Display.fillRect(navArea.x, navArea.y, navArea.w, navArea.h, COL_BG);

  if (smfPlaying) {
    Rect fullArea = {
      contentArea.x,
      contentArea.y,
      contentArea.w,
      contentArea.h + navArea.h
    };
    drawSmfChannelKeyboard(fullArea);
    return;
  }

  M5.Display.fillRoundRect(smfListArea.x, smfListArea.y, smfListArea.w, smfListArea.h, 12, COL_PANEL);
  M5.Display.drawRoundRect(smfListArea.x, smfListArea.y, smfListArea.w, smfListArea.h, 12, COL_BTN_BDR);
  M5.Display.setFont(FONT_SMALL);
  M5.Display.setTextColor(COL_TITLE, COL_PANEL);
  M5.Display.setTextDatum(top_left);
  M5.Display.drawString("SMF Playlist", smfListArea.x + 12, smfListArea.y + 14);
  drawDoubleTriangleBtn(smfListPageUpBtn,   TFT_BLUE, COL_BTN_BDR, true,  COL_BTN_TXT);
  drawTriangleBtn(smfListUpBtn,             TFT_BLUE, COL_BTN_BDR, true,  COL_BTN_TXT);
  drawTriangleBtn(smfListDownBtn,           TFT_BLUE, COL_BTN_BDR, false, COL_BTN_TXT);
  drawDoubleTriangleBtn(smfListPageDownBtn, TFT_BLUE, COL_BTN_BDR, false, COL_BTN_TXT);

  int lineH = 44;
  int top = smfListArea.y + 84;
  int visible = (smfListArea.h - 84) / lineH;
  // See drawMp3Static for the rationale: only auto-follow when a real
  // track is selected (>=0). With -1 (post-folder-nav) the follow logic
  // would yank smfListScroll back to 0 every redraw and eat the page-jump
  // taps.
  if (smfCurrentTrack >= 0) {
    if (smfCurrentTrack < smfListScroll) smfListScroll = smfCurrentTrack;
    if (smfCurrentTrack >= smfListScroll + visible) smfListScroll = smfCurrentTrack - visible + 1;
  }
  if (smfListScroll < 0) smfListScroll = 0;
  int maxScroll = max(0, smfPlaylistCount - visible);
  if (smfListScroll > maxScroll) smfListScroll = maxScroll;
  for (int row = 0; row < visible; ++row) {
    int idx = smfListScroll + row;
    if (idx >= smfPlaylistCount) break;
    Rect rr = { smfListArea.x + 10, top + row * lineH, smfListArea.w - 20, lineH - 2 };
    bool on = (idx == smfCurrentTrack);
    M5.Display.fillRoundRect(rr.x, rr.y, rr.w, rr.h, 6, on ? COL_BTN_HI2 : COL_PANEL);
    char labelBuf[PLAYLIST_PATH_MAX + 4];
    const char* name = formatPlaylistLabel(smfPlaylist[idx], labelBuf, sizeof(labelBuf));
    bool isNav = !entryIsFile(smfPlaylist[idx]);
    uint16_t txt = on ? COL_BTN_TXT_HI : (isNav ? COL_ACCENT : COL_BTN_TXT);
    M5.Display.setFont(FONT_SMALL);
    M5.Display.setTextColor(txt, on ? COL_BTN_HI2 : COL_PANEL);
    M5.Display.setTextDatum(middle_left);
    M5.Display.drawString(name, rr.x + 8, rr.y + rr.h / 2);
  }
}

static void drawMp3() {
  if (mp3StaticDirty) drawMp3Static();
  if (mp3Playing || mp3VisualDirty) drawMp3Visual();
}

static void drawInterface() {
  // SRC fullscreen instrument picker takes over the entire screen, including
  // header/toolbar, so it draws first and short-circuits.
  if (g_srcPickerOpen) {
    drawSrcPicker();
    return;
  }
  M5.Display.fillScreen(COL_BG);
  drawHeader();
  drawToolbarApp();
  drawNavApp();
  // CONFIG_EDIT / BASE_SET overlay completely takes over the content + nav
  // areas but leaves the header/toolbar visible (so BT/AOFF/MIDI in still
  // show through). Falls through to the per-app draw on normal modes.
  if (currentMode == CONFIG_EDIT_MODE) { drawConfigEditMode(); return; }
  if (currentMode == BASE_SET_MODE)    { drawBaseSetMode();    return; }
  switch (currentApp) {
    case APP_TRANSPOSE:
      switch (currentMode) {
        case DIRECT_MODE:      drawDirect();      break;
        case KEY_MODE:         drawKey();         break;
        case INSTANT_MODE:     drawInstant();     break;
        case SEQUENCE_MODE:    drawSequence();    break;
        default: break;  // overlay modes are handled above
      }
      break;
    case APP_MIDI: drawMidiManage(); break;
    case APP_PLAY:
      if      (currentPlay == PLAY_SRC) drawSrc();
      else if (currentPlay == PLAY_SMF) drawSmf();
      else                              drawMp3();
      break;
  }
}

// =================================================================
//  Touch handlers
// =================================================================
static void setCurrentTransposeButton() {
  for (int i = 0; i < 12; i++) transposeButtonsOn[i] = false;
  for (int i = 0; i < 12; i++) {
    if (directBtns[i].value == transposeValue) {
      transposeButtonsOn[i] = true;
      break;
    }
  }
}

// Defined further down where processMIDIByte() lives so the parser's static
// state can be cleared in one place. Forward-declared as a friend-of-sorts via
// a global flag the parser inspects on entry.
static volatile bool g_midiParserResetRequested = false;
static void resetMidiInputParser() {
  // Mark the parser dirty; processMIDIByte() will clear its internal latches
  // before consuming the next byte from any source.
  g_midiParserResetRequested = true;
}

// =================================================================
//  USB MIDI host (ESP-IDF usb_host driver)
// =================================================================
static void usbMidiRingPush(const uint8_t* bytes, size_t n) {
  portENTER_CRITICAL(&g_usbMidiRingMux);
  for (size_t i = 0; i < n; ++i) {
    uint32_t next = (g_usbMidiRingHead + 1) % USB_MIDI_RING_SIZE;
    if (next == g_usbMidiRingTail) {
      g_usbMidiRingDropCount += (uint32_t)(n - i);  // count what didn't fit
      break;
    }
    g_usbMidiRing[g_usbMidiRingHead] = bytes[i];
    g_usbMidiRingHead = next;
  }
  portEXIT_CRITICAL(&g_usbMidiRingMux);
}

static int usbMidiRingPop() {
  int v = -1;
  portENTER_CRITICAL(&g_usbMidiRingMux);
  if (g_usbMidiRingHead != g_usbMidiRingTail) {
    v = g_usbMidiRing[g_usbMidiRingTail];
    g_usbMidiRingTail = (g_usbMidiRingTail + 1) % USB_MIDI_RING_SIZE;
  }
  portEXIT_CRITICAL(&g_usbMidiRingMux);
  return v;
}

static size_t usbMidiRingAvailable() {
  size_t a;
  portENTER_CRITICAL(&g_usbMidiRingMux);
  uint32_t h = g_usbMidiRingHead, t = g_usbMidiRingTail;
  a = (h >= t) ? (size_t)(h - t) : (size_t)(USB_MIDI_RING_SIZE - t + h);
  portEXIT_CRITICAL(&g_usbMidiRingMux);
  return a;
}

// USB-MIDI 1.0 Code Index Number (CIN) → number of MIDI bytes following the
// header byte in a 4-byte event packet. Index by low nibble of the header.
static const uint8_t USB_MIDI_CIN_LEN[16] = {
  0, 0, 2, 3, 3, 1, 2, 3, 3, 3, 3, 3, 2, 2, 3, 1
};

// Submitted IN-endpoint transfer completed. Runs in USB task context. Parse
// USB-MIDI packets into a raw MIDI byte stream so the existing MIDI parser
// in the loop can process it.
static void usbMidiInDoneCb(usb_transfer_t* t) {
  // Defensive: the IDF may invoke the callback during teardown for terminal
  // statuses (NO_DEVICE, CANCELED, ERROR). In that case we must not touch
  // the transfer further or resubmit — the host stack will free it.
  if (t == nullptr) return;
  bool dataOk = (t->status == USB_TRANSFER_STATUS_COMPLETED && t->actual_num_bytes >= 4);
  if (dataOk) {
    for (int off = 0; off + 4 <= t->actual_num_bytes; off += 4) {
      const uint8_t* p = t->data_buffer + off;
      // Empty packet (no event) — common when an IN poll returns nothing.
      if (p[0] == 0x00 && p[1] == 0x00 && p[2] == 0x00 && p[3] == 0x00) continue;
      uint8_t cin = p[0] & 0x0F;
      uint8_t n   = USB_MIDI_CIN_LEN[cin];
      if (n == 0) continue;
      usbMidiRingPush(p + 1, n);
    }
  }
  // Re-arm only when the transfer ended cleanly AND the device is still
  // mounted. Resubmitting after NO_DEVICE / CANCELED races the teardown and
  // can use the transfer struct after free().
  if (g_usbMidiMounted &&
      t->status != USB_TRANSFER_STATUS_NO_DEVICE &&
      t->status != USB_TRANSFER_STATUS_CANCELED) {
    // Tight retry: transient submit failures under heavy load shouldn't
    // permanently retire this slot (which would silently shrink our queue
    // depth and re-introduce the gap that drops USB chord bytes).
    esp_err_t e = usb_host_transfer_submit(t);
    if (e != ESP_OK) {
      for (int retry = 0; retry < 3 && e != ESP_OK; ++retry) {
        e = usb_host_transfer_submit(t);
      }
      if (e != ESP_OK) {
        ++g_usbInResubmitFails;
      }
    }
  }
}

// Walk the active configuration descriptor and claim the first MIDI Streaming
// interface (Audio class 0x01, subclass 0x03), allocating an IN-endpoint
// transfer to receive USB-MIDI events.
static bool usbMidiClaimAndStart(const usb_config_desc_t* cfg) {
  const uint8_t* p = cfg->val;
  uint16_t total = cfg->wTotalLength;
  uint16_t i = 0;

  uint8_t  midiItfNum    = 0xFF;
  uint8_t  midiItfAlt    = 0;
  uint8_t  inEp          = 0;
  uint16_t inEpMaxPacket = 0;
  uint8_t  inEpInterval  = 1;

  // First pass: locate Audio/MIDI Streaming interface and IN endpoint.
  while (i + 2 <= total) {
    uint8_t len = p[i];
    if (len < 2 || i + len > total) break;
    uint8_t type = p[i + 1];
    if (type == 0x04 /*INTERFACE*/ && len >= 9) {
      uint8_t itfNum   = p[i + 2];
      uint8_t itfAlt   = p[i + 3];
      uint8_t numEp    = p[i + 4];
      uint8_t cls      = p[i + 5];
      uint8_t subcls   = p[i + 6];
      if (cls == 0x01 /*AUDIO*/ && subcls == 0x03 /*MIDI_STREAMING*/) {
        midiItfNum = itfNum;
        midiItfAlt = itfAlt;
        // Scan endpoints belonging to this interface
        uint16_t j = i + len;
        uint8_t  epSeen = 0;
        while (j + 2 <= total && epSeen < numEp) {
          uint8_t lj = p[j];
          if (lj < 2 || j + lj > total) break;
          uint8_t tj = p[j + 1];
          if (tj == 0x04 /*INTERFACE*/) break;  // hit next interface
          if (tj == 0x05 /*ENDPOINT*/ && lj >= 7) {
            uint8_t  addr   = p[j + 2];
            uint16_t maxPkt = (uint16_t)(p[j + 4] | (p[j + 5] << 8));
            uint8_t  bIntr  = p[j + 6];
            if ((addr & 0x80) && inEp == 0) {
              inEp          = addr;
              inEpMaxPacket = maxPkt > 64 ? 64 : (maxPkt == 0 ? 64 : maxPkt);
              inEpInterval  = bIntr ? bIntr : 1;
            }
            epSeen++;
          }
          j += lj;
        }
        if (inEp) break;  // got what we need
      }
    }
    i += len;
  }

  if (midiItfNum == 0xFF || inEp == 0) {
    Serial.println("[USB] no MIDI Streaming interface found");
    return false;
  }

  esp_err_t err = usb_host_interface_claim(g_usbClient, g_usbDevice, midiItfNum, midiItfAlt);
  if (err != ESP_OK) {
    Serial.printf("[USB] interface_claim itf=%u alt=%u err=0x%x\n",
                  (unsigned)midiItfNum, (unsigned)midiItfAlt, (unsigned)err);
    return false;
  }

  // Allocate USB_MIDI_IN_XFERS transfers and prime them all so the host
  // controller always has at least one buffer queued — eliminates the gap
  // during callback execution where chord bytes were silently dropped at
  // the USB layer (before reaching our ring counter, which is why
  // `usb_drop` stayed at 0 even when notes went missing).
  int allocated = 0;
  for (int i = 0; i < USB_MIDI_IN_XFERS; ++i) {
    err = usb_host_transfer_alloc(inEpMaxPacket, 0, &g_usbInXfers[i]);
    if (err != ESP_OK || g_usbInXfers[i] == nullptr) {
      Serial.printf("[USB] transfer_alloc[%d] err=0x%x\n", i, (unsigned)err);
      break;
    }
    g_usbInXfers[i]->device_handle    = g_usbDevice;
    g_usbInXfers[i]->bEndpointAddress = inEp;
    g_usbInXfers[i]->callback         = usbMidiInDoneCb;
    g_usbInXfers[i]->context          = nullptr;
    g_usbInXfers[i]->num_bytes        = inEpMaxPacket;
    g_usbInXfers[i]->timeout_ms       = 0;
    ++allocated;
  }
  if (allocated == 0) {
    usb_host_interface_release(g_usbClient, g_usbDevice, midiItfNum);
    return false;
  }

  g_usbMidiClaimedItf = midiItfNum;
  g_usbMidiInEpAddr   = inEp;
  g_usbMidiCableCount = 1;  // we don't parse Element/Jack descriptors; assume 1
  g_usbMidiActiveIndex = 0;
  g_usbMidiMounted    = true;

  for (int i = 0; i < allocated; ++i) {
    if (usb_host_transfer_submit(g_usbInXfers[i]) != ESP_OK) {
      Serial.printf("[USB] initial IN submit[%d] failed\n", i);
    }
  }
  Serial.printf("[USB] MIDI mounted itf=%u in_ep=0x%02x mps=%u poll=%u\n",
                (unsigned)midiItfNum, (unsigned)inEp,
                (unsigned)inEpMaxPacket, (unsigned)inEpInterval);
  // Full redraw because the MIX button highlight depends on g_usbMidiMounted
  // and partial updates intentionally don't repaint the right-side buttons.
  needFullRedraw = true;
  return true;
}

static void usbMidiTeardown() {
  // Mark unmounted FIRST so the IN callback stops resubmitting if it races.
  g_usbMidiMounted = false;

  // Halt the IN endpoint (cancels any in-flight transfer) before freeing the
  // transfer structure. Without this, `usb_host_transfer_free` on a pending
  // transfer can corrupt the IDF's internal lists and crash the host task
  // when a device is unplugged mid-stream. usb_host_endpoint_halt may fail
  // when the device is already gone (NO_DEVICE) — that's fine, the IDF has
  // already cancelled the transfer for us in that path.
  if (g_usbDevice && g_usbMidiInEpAddr != 0) {
    usb_host_endpoint_halt(g_usbDevice, g_usbMidiInEpAddr);
    usb_host_endpoint_flush(g_usbDevice, g_usbMidiInEpAddr);
    usb_host_endpoint_clear(g_usbDevice, g_usbMidiInEpAddr);
    // Halt/flush/clear cancels in-flight transfers, but the IN-completion
    // callback for them may already have been queued onto the host task
    // before we got here. If we free the transfer struct now the callback
    // will dereference freed memory when it finally runs. Yielding for
    // ~20 ms gives usbHostTask a chance to drain those pending callbacks
    // (each one early-exits via the !g_usbMidiMounted check we set above).
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  // Now it is safe to release the interface (must precede transfer_free per
  // IDF docs only when the transfer is idle — we just halted/flushed it).
  if (g_usbDevice && g_usbClient && g_usbMidiClaimedItf != 0xFF) {
    usb_host_interface_release(g_usbClient, g_usbDevice, g_usbMidiClaimedItf);
  }

  for (int i = 0; i < USB_MIDI_IN_XFERS; ++i) {
    if (g_usbInXfers[i]) {
      usb_host_transfer_free(g_usbInXfers[i]);
      g_usbInXfers[i] = nullptr;
    }
  }

  g_usbMidiClaimedItf = 0xFF;
  g_usbMidiInEpAddr   = 0;
  g_usbMidiCableCount = 0;
  g_usbMidiActiveIndex = 0xFF;
  if (g_usbDevice) {
    usb_host_device_close(g_usbClient, g_usbDevice);
    g_usbDevice = nullptr;
  }

  // Drain any half-parsed bytes left in the ring so the next mount starts
  // from a clean state — leftover bytes after an unplug confuse the running-
  // status MIDI parser.
  portENTER_CRITICAL(&g_usbMidiRingMux);
  g_usbMidiRingHead = 0;
  g_usbMidiRingTail = 0;
  portEXIT_CRITICAL(&g_usbMidiRingMux);
  g_midiParserResetRequested = true;

  // Full redraw because the MIX button highlight depends on g_usbMidiMounted
  // and partial updates intentionally don't repaint the right-side buttons.
  needFullRedraw = true;
}

// Called by the USB host stack when a device is attached or removed.
static void usbClientEventCb(const usb_host_client_event_msg_t* msg, void* /*arg*/) {
  switch (msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV: {
      if (g_usbDevice) break;  // already have one (we don't manage hubs of MIDI devices)
      esp_err_t err = usb_host_device_open(g_usbClient, msg->new_dev.address, &g_usbDevice);
      if (err != ESP_OK) {
        Serial.printf("[USB] device_open err=0x%x\n", (unsigned)err);
        g_usbDevice = nullptr;
        break;
      }
      const usb_config_desc_t* cfg = nullptr;
      err = usb_host_get_active_config_descriptor(g_usbDevice, &cfg);
      if (err != ESP_OK || !cfg) {
        Serial.printf("[USB] get_active_config_descriptor err=0x%x\n", (unsigned)err);
        usb_host_device_close(g_usbClient, g_usbDevice);
        g_usbDevice = nullptr;
        break;
      }
      if (!usbMidiClaimAndStart(cfg)) {
        // Not a MIDI device (or no claim) — release the device handle.
        usb_host_device_close(g_usbClient, g_usbDevice);
        g_usbDevice = nullptr;
      }
      break;
    }
    case USB_HOST_CLIENT_EVENT_DEV_GONE: {
      Serial.println("[USB] device gone");
      usbMidiTeardown();
      break;
    }
    default:
      break;
  }
}

// Background task: drives the host stack and the registered client.
static void usbHostTask(void* /*arg*/) {
  for (;;) {
    uint32_t evtFlags = 0;
    usb_host_lib_handle_events(pdMS_TO_TICKS(20), &evtFlags);
    if (g_usbClient) {
      usb_host_client_handle_events(g_usbClient, pdMS_TO_TICKS(5));
    }
    // No periodic resubmit needed — usbMidiInDoneCb resubmits.
  }
}

static bool startUsbHost() {
  if (g_usbHostReady) return true;

  usb_host_config_t hostCfg = {};
  hostCfg.skip_phy_setup = false;             // let IDF bring up the PHY
  hostCfg.intr_flags     = ESP_INTR_FLAG_LEVEL1;
  esp_err_t err = usb_host_install(&hostCfg);
  if (err != ESP_OK) {
    Serial.printf("[USB] host_install err=0x%x\n", (unsigned)err);
    return false;
  }

  usb_host_client_config_t clientCfg = {};
  clientCfg.is_synchronous   = false;
  clientCfg.max_num_event_msg = 5;
  clientCfg.async.client_event_callback = usbClientEventCb;
  clientCfg.async.callback_arg          = nullptr;
  err = usb_host_client_register(&clientCfg, &g_usbClient);
  if (err != ESP_OK) {
    Serial.printf("[USB] client_register err=0x%x\n", (unsigned)err);
    usb_host_uninstall();
    return false;
  }

  BaseType_t ok = xTaskCreatePinnedToCore(
    usbHostTask, "usbh", 4096, nullptr, 5, &g_usbTask, 0);
  if (ok != pdPASS) {
    Serial.println("[USB] usb host task spawn failed");
    usb_host_client_deregister(g_usbClient);
    g_usbClient = nullptr;
    usb_host_uninstall();
    return false;
  }

  g_usbHostReady = true;
  Serial.println("[USB] host ready (idf usb_host); waiting for MIDI device");
  return true;
}

static bool setMidiInputSource(MidiInputSource source) {
  if (midiInputSource == source) return true;
  sendAllNotesOff();
  resetMidiInputParser();
  midiInputSource = source;
  needFullRedraw = true;
  needPartialUpdate = true;
  return true;
}

static MidiInputSource nextMidiInputSource() {
  switch (midiInputSource) {
    case MIDI_INPUT_MIX:  return MIDI_INPUT_USB;
    case MIDI_INPUT_USB:  return MIDI_INPUT_UNIT;
    default:              return MIDI_INPUT_MIX;
  }
}

static void handleToolbarTouch(int x, int y) {
  if (hit(midiInputSourceBtn, x, y)) {
    setMidiInputSource(nextMidiInputSource());
    return;
  }
  // BASE / CONFIG header buttons (preferred dedicated entry points; the
  // long-tap-on-status fall-backs added earlier still work as alternates).
  if (hit(baseEntryBtn, x, y)) {
    enterBaseSetMode();
    return;
  }
  if (hit(cfgEntryBtn, x, y)) {
    enterConfigEditMode();
    return;
  }
  // Top app tabs are always visible.
  for (int i = 0; i < 3; ++i) {
    if (hit(appTab[i], x, y)) {
      setCurrentApp((AppMode)i);
      return;
    }
  }
  // AOFF button is always visible.
  if (hit(btnAllOff, x, y)) {
    allNotesOffEnabled = !allNotesOffEnabled;
    if (allNotesOffEnabled) sendAllNotesOff();
    needFullRedraw = true;
    return;
  }

  // MIDI app: FILTER / MAPPER / BYPASS|ACTIVE
  if (currentApp == APP_MIDI) {
    if (hit(midiAppTabFilter, x, y)) { midiManagePage = MIDI_PAGE_FILTER; needFullRedraw = true; return; }
    if (hit(midiAppTabMapper, x, y)) { midiManagePage = MIDI_PAGE_MAPPER; needFullRedraw = true; return; }
    if (hit(midiAppBypassBtn, x, y)) {
      if (midiManagePage == MIDI_PAGE_FILTER) midiFilterBypass = !midiFilterBypass;
      else                                     midiMapperBypass = !midiMapperBypass;
      needFullRedraw = true;
      return;
    }
    return;
  }

  // PLAY app: SRC / SMF / MP3 mode tabs
  if (currentApp == APP_PLAY) {
    for (int i = 0; i < 3; i++) {
      if (hit(playTab[i], x, y)) {
        if (currentPlay == (PlayMode)i) return;
        if (currentPlay == PLAY_SMF) stopSmf();
        else if (currentPlay == PLAY_MP3) stopMp3();
        currentPlay = (PlayMode)i;
        if (currentPlay == PLAY_SRC) {
          srcDirtyAll = true;
        } else if (currentPlay == PLAY_SMF) {
          ensureStorage();
          if (smfPlaylistCount == 0) scanSmfFiles();
          invalidateSmfMonitorAll();
        } else {
          ensureStorage();
          if (mp3PlaylistCount == 0) scanMp3Files();
        }
        needFullRedraw = true;
        return;
      }
    }
  }

  // SRC: 4 dedicated synth-control buttons in the toolbar slot.
  if ((currentApp == APP_PLAY && currentPlay == PLAY_SRC)) {
    if (hit(srcBtnGm, x, y)) {
      sendGMReset(); delay(30);
      srcResetState(true);
      needFullRedraw = true;
      return;
    }
    if (hit(srcBtnGs, x, y)) {
      sendGSReset(); delay(30);
      srcResetState(true);
      needFullRedraw = true;
      return;
    }
    if (hit(srcBtnInit, x, y)) {
      srcHardReset();
      needFullRedraw = true;
      return;
    }
    if (hit(srcBtnAuto, x, y)) {
      srcAutoFollow = !srcAutoFollow;
      needFullRedraw = true;
      return;
    }
    return;  // toolbar y-range; content widgets are handled by handleSrcTouch.
  }

  if ((currentApp == APP_PLAY && currentPlay == PLAY_SMF)) {
    if (hit(smfBtnPrev, x, y) && smfPlaylistCount > 0) {
      int i = (smfCurrentTrack >= 0) ? smfCurrentTrack : 0;
      for (int n = 0; n < smfPlaylistCount; n++) {
        i = (i - 1 + smfPlaylistCount) % smfPlaylistCount;
        if (entryIsFile(smfPlaylist[i])) { loadSmfTrack(i); break; }
      }
      needFullRedraw = true;
    } else if (hit(smfBtnPlay, x, y)) {
      if (smfPlaying) stopSmf();
      else playSmf();
      needFullRedraw = true;
    } else if (hit(smfBtnNext, x, y) && smfPlaylistCount > 0) {
      int i = (smfCurrentTrack >= 0) ? smfCurrentTrack : -1;
      for (int n = 0; n < smfPlaylistCount; n++) {
        i = (i + 1) % smfPlaylistCount;
        if (entryIsFile(smfPlaylist[i])) { loadSmfTrack(i); break; }
      }
      needFullRedraw = true;
    } else if (hit(smfBtnLoop, x, y)) {
      smfLoop = !smfLoop;
      smf.looping(smfLoop);
      needFullRedraw = true;
    }
    return;
  }

  if ((currentApp == APP_PLAY && currentPlay == PLAY_MP3)) {
    if (hit(mp3BtnPrev, x, y) && mp3PlaylistCount > 0) {
      int i = (mp3CurrentTrack >= 0) ? mp3CurrentTrack : 0;
      for (int n = 0; n < mp3PlaylistCount; n++) {
        i = (i - 1 + mp3PlaylistCount) % mp3PlaylistCount;
        if (entryIsFile(mp3Playlist[i])) { startMp3Track(i); break; }
      }
      needFullRedraw = true;
    } else if (hit(mp3BtnPlay, x, y)) {
      if (mp3Playing) stopMp3();
      else if (mp3CurrentTrack >= 0 && mp3CurrentTrack < mp3PlaylistCount &&
               entryIsFile(mp3Playlist[mp3CurrentTrack])) {
        startMp3Track(mp3CurrentTrack);
      }
      needFullRedraw = true;
    } else if (hit(mp3BtnNext, x, y) && mp3PlaylistCount > 0) {
      int i = (mp3CurrentTrack >= 0) ? mp3CurrentTrack : -1;
      for (int n = 0; n < mp3PlaylistCount; n++) {
        i = (i + 1) % mp3PlaylistCount;
        if (entryIsFile(mp3Playlist[i])) { startMp3Track(i); break; }
      }
      needFullRedraw = true;
    } else if (hit(mp3BtnVolDown, x, y)) {
      mp3Volume = max(0, mp3Volume - 16);
      M5.Speaker.setVolume(mp3Volume);
      mp3StaticDirty = true;
      needPartialUpdate = true;
    } else if (hit(mp3BtnVolUp, x, y)) {
      mp3Volume = min(255, mp3Volume + 16);
      M5.Speaker.setVolume(mp3Volume);
      mp3StaticDirty = true;
      needPartialUpdate = true;
    }
    return;
  }

  // XPOSE mode tabs.
  if (currentApp != APP_TRANSPOSE) return;
  for (int i = 0; i < 4; i++) {
    if (hit(modeTab[i], x, y)) {
      if (currentMode == (DisplayMode)i) return;
      sendAllNotesOff();
      currentMode = (DisplayMode)i;
      if (currentMode == DIRECT_MODE) setCurrentTransposeButton();
      else if (currentMode == KEY_MODE) { selectedMajorKey = -1; selectedMinorKey = -1; }
      else if (currentMode == SEQUENCE_MODE) {
        handleTransposeChange(seqPatterns[seqCurrentPattern][seqCurrentStep]);
      }
      needFullRedraw = true;
      return;
    }
  }
  // Range button.
  if (hit(btnRange, x, y)) {
    if (currentMode == DIRECT_MODE) {
      transposeRange = (TransposeRange)((transposeRange + 1) % 3);
      updateDirectButtonLabels();
      setCurrentTransposeButton();
      needFullRedraw = true;
    } else if (currentMode == KEY_MODE) {
      majorUpperTranspose = !majorUpperTranspose;
      minorUpperTranspose = !minorUpperTranspose;
      selectedMajorKey = -1;
      selectedMinorKey = -1;
      needFullRedraw = true;
    }
    return;
  }
  // All-notes-off toggle.
  if (hit(btnAllOff, x, y)) {
    allNotesOffEnabled = !allNotesOffEnabled;
    needFullRedraw = true;
    return;
  }
}

// Value list for the bottom PREV/NEXT nav.  Depends on the current mode.
static int buildValueList(int8_t* out) {
  if (currentMode == DIRECT_MODE) {
    for (int i = 0; i < 12; i++) out[i] = directBtns[i].value;
    return 12;
  }
  if (currentMode == KEY_MODE) {
    // Cover -11..+12 range in small steps.
    for (int i = 0; i < 12; i++) out[i] = i - 5;
    return 12;
  }
  // INSTANT / SEQUENCE fall back to -5..+6.
  for (int i = 0; i < 12; i++) out[i] = i - 5;
  return 12;
}

static void shiftTransposeBy(int dir) {
  if (currentMode == SEQUENCE_MODE) {
    if (dir < 0)
      seqCurrentStep = (seqCurrentStep > 0) ? seqCurrentStep - 1 : SEQ_STEP_COUNT - 1;
    else
      seqCurrentStep = (seqCurrentStep < SEQ_STEP_COUNT - 1) ? seqCurrentStep + 1 : 0;
    handleTransposeChange(seqPatterns[seqCurrentPattern][seqCurrentStep]);
    needFullRedraw = true;
    return;
  }

  int8_t values[12];
  int n = buildValueList(values);
  int idx = -1;
  for (int i = 0; i < n; i++) if (values[i] == transposeValue) { idx = i; break; }
  if (idx < 0) idx = n / 2;
  int newIdx = (dir < 0) ? ((idx > 0) ? idx - 1 : n - 1)
                         : ((idx < n - 1) ? idx + 1 : 0);
  handleTransposeChange(values[newIdx]);

  if (currentMode == DIRECT_MODE) {
    for (int j = 0; j < 12; j++) transposeButtonsOn[j] = false;
    transposeButtonsOn[newIdx] = true;
  } else if (currentMode == KEY_MODE) {
    selectedMajorKey = -1;
    selectedMinorKey = -1;
  }
  needFullRedraw = true;
}

static void handleNavTouch(int x, int y) {
  if (currentApp == APP_TRANSPOSE) {
    if (hit(btnPrev, x, y)) shiftTransposeBy(-1);
    else if (hit(btnNext, x, y)) shiftTransposeBy(+1);
    return;
  }
  // SRC parks its VOL / PB / SUS row in the navArea band (otherwise that ~100
  // px at the bottom of the screen is unused for non-XPOSE apps). Forward
  // those taps to the SRC handler so the controls work when tapped.
  if (currentApp == APP_PLAY && currentPlay == PLAY_SRC) {
    handleSrcTouch(x, y);
    return;
  }
}

static void handleDirectTouch(int x, int y) {
  for (int i = 0; i < 12; i++) {
    if (hit(directBtns[i].r, x, y)) {
      handleTransposeChange(directBtns[i].value);
      for (int j = 0; j < 12; j++) transposeButtonsOn[j] = false;
      transposeButtonsOn[i] = true;
      needFullRedraw = true;
      return;
    }
  }
}

static void handleKeyTouch(int x, int y) {
  // Black first so they win on overlap.
  for (int pass = 0; pass < 2; pass++) {
    bool wantBlack = (pass == 0);
    for (int i = 0; i < 12; i++) {
      if (majorKeys[i].isBlackKey != wantBlack) continue;
      if (hit(majorKeys[i].r, x, y)) {
        selectedMajorKey = i;
        selectedMinorKey = -1;
        int8_t nv = majorUpperTranspose ? (int8_t)(12 - i) : (int8_t)(-i);
        handleTransposeChange(nv);
        needFullRedraw = true;
        return;
      }
    }
    for (int i = 0; i < 12; i++) {
      if (minorKeys[i].isBlackKey != wantBlack) continue;
      if (hit(minorKeys[i].r, x, y)) {
        selectedMinorKey = i;
        selectedMajorKey = -1;
        int8_t nv = minorUpperTranspose ? (int8_t)(-(9 - i)) : (int8_t)(9 - i);
        handleTransposeChange(nv);
        needFullRedraw = true;
        return;
      }
    }
  }
}

static void handleInstantTouch(int x, int y) {
  if (hit(instantZero, x, y)) {
    handleTransposeChange(0);
    needFullRedraw = true;
    return;
  }
  for (int i = 0; i < 8; i++) {
    if (hit(instantBtns[i].r, x, y)) {
      handleTransposeChange(instantBtns[i].value);
      needFullRedraw = true;
      return;
    }
  }
}

static void handleSequenceTouch(int x, int y) {
  if (hit(seqPatLeft, x, y)) {
    seqCurrentPattern = (seqCurrentPattern > 0) ? seqCurrentPattern - 1 : SEQ_PATTERN_COUNT - 1;
    seqCurrentStep = 0;
    handleTransposeChange(seqPatterns[seqCurrentPattern][seqCurrentStep]);
    needFullRedraw = true;
    return;
  }
  if (hit(seqPatRight, x, y)) {
    seqCurrentPattern = (seqCurrentPattern < SEQ_PATTERN_COUNT - 1) ? seqCurrentPattern + 1 : 0;
    seqCurrentStep = 0;
    handleTransposeChange(seqPatterns[seqCurrentPattern][seqCurrentStep]);
    needFullRedraw = true;
    return;
  }
  if (hit(seqSave, x, y)) {
    g_seqSavePending = true;
    g_seqSaveUiState = SEQ_SAVE_UI_PENDING;
    g_seqSaveUiUntil = 0;
    needFullRedraw = true;
    return;
  }
  for (int i = 0; i < SEQ_STEP_COUNT; i++) {
    if (hit(seqSteps[i].up, x, y)) {
      seqPatterns[seqCurrentPattern][i] = clampTranspose(seqPatterns[seqCurrentPattern][i] + 1);
      seqCurrentStep = i;
      handleTransposeChange(seqPatterns[seqCurrentPattern][i]);
      needFullRedraw = true;
      return;
    }
    if (hit(seqSteps[i].down, x, y)) {
      seqPatterns[seqCurrentPattern][i] = clampTranspose(seqPatterns[seqCurrentPattern][i] - 1);
      seqCurrentStep = i;
      handleTransposeChange(seqPatterns[seqCurrentPattern][i]);
      needFullRedraw = true;
      return;
    }
    if (hit(seqSteps[i].slot, x, y)) {
      seqCurrentStep = i;
      handleTransposeChange(seqPatterns[seqCurrentPattern][i]);
      needFullRedraw = true;
      return;
    }
  }
}

static void setCurrentApp(AppMode app) {
  if (currentApp == app) return;

  sendAllNotesOff();
  if ((currentApp == APP_PLAY && currentPlay == PLAY_SMF)) stopSmf();
  if ((currentApp == APP_PLAY && currentPlay == PLAY_MP3)) stopMp3();

  currentApp = app;
  if (currentApp == APP_PLAY) {
    if (currentPlay == PLAY_SMF) {
      ensureStorage();
      if (smfPlaylistCount == 0) scanSmfFiles();
      invalidateSmfMonitorAll();
    } else if (currentPlay == PLAY_MP3) {
      ensureStorage();
      if (mp3PlaylistCount == 0) scanMp3Files();
    }
  }
  needFullRedraw = true;
  needPartialUpdate = true;
}

static bool ensureStorage() {
  if (storageReady) return true;

  SPI.begin(SD_SPI_SCK, SD_SPI_MISO, SD_SPI_MOSI, SD_SPI_CS);
  storageReady = SD.begin(SD_SPI_CS, SPI, 25000000);
  if (storageReady) {
    midiFsReady = midiSd.begin(SdSpiConfig(SD_SPI_CS, SHARED_SPI, SD_SCK_MHZ(25), &SPI));
  }
  if (!storageReady) Serial.println("[SD] SD.begin failed");
  if (storageReady && !midiFsReady) Serial.println("[SdFat] midiSd.begin failed");
  return storageReady;
}

// Case-insensitive suffix match on a NUL-terminated string. We avoid the
// String / std::string ecosystem entirely on the playback path so no heap
// allocation can happen here.
static bool nameHasExt(const char* name, const char* ext) {
  if (!name || !ext) return false;
  size_t nl = strlen(name);
  size_t el = strlen(ext);
  if (nl < el) return false;
  const char* p = name + (nl - el);
  for (size_t i = 0; i < el; i++) {
    char a = p[i];
    char b = ext[i];
    if (a >= 'A' && a <= 'Z') a = (char)(a + ('a' - 'A'));
    if (b >= 'A' && b <= 'Z') b = (char)(b + ('a' - 'A'));
    if (a != b) return false;
  }
  return true;
}

// Case-insensitive natural compare: digit runs compare numerically so
// "2_foo" sorts before "10_foo". Used to keep playlists in human order;
// the SD library returns entries in FAT directory-insertion order, which
// rarely matches the "0_, 1_, 2_, ..." prefix users expect.
static int playlistNaturalCmp(const char* a, const char* b) {
  while (*a && *b) {
    if (isdigit((unsigned char)*a) && isdigit((unsigned char)*b)) {
      while (*a == '0') a++;
      while (*b == '0') b++;
      const char* aDigits = a;
      const char* bDigits = b;
      while (isdigit((unsigned char)*a)) a++;
      while (isdigit((unsigned char)*b)) b++;
      size_t aLen = a - aDigits;
      size_t bLen = b - bDigits;
      if (aLen != bLen) return (aLen < bLen) ? -1 : 1;
      int c = strncmp(aDigits, bDigits, aLen);
      if (c) return c;
    } else {
      int ca = tolower((unsigned char)*a);
      int cb = tolower((unsigned char)*b);
      if (ca != cb) return (ca < cb) ? -1 : 1;
      a++; b++;
    }
  }
  if (*a) return 1;
  if (*b) return -1;
  return 0;
}

// Entry kind helpers. Paths in the playlist arrays encode kind via their
// trailing character: ".." literal == parent (up), trailing '/' == dir,
// otherwise file.
static inline bool entryIsParent(const char* p) { return strcmp(p, "..") == 0; }
static inline bool entryIsDir(const char* p) {
  size_t n = strlen(p);
  return n > 0 && p[n - 1] == '/';
}
static inline bool entryIsFile(const char* p) {
  return !entryIsParent(p) && !entryIsDir(p);
}

// Sort key: parent first, then directories, then files. Within each group
// entries are natural-sorted by basename.
static int playlistEntryCmp(const void* pa, const void* pb) {
  const char* a = (const char*)pa;
  const char* b = (const char*)pb;
  int ka = entryIsParent(a) ? 0 : (entryIsDir(a) ? 1 : 2);
  int kb = entryIsParent(b) ? 0 : (entryIsDir(b) ? 1 : 2);
  if (ka != kb) return ka - kb;
  // For dirs, strip the trailing slash before locating the basename so
  // "/smf/album1/" compares as "album1".
  char abuf[PLAYLIST_PATH_MAX], bbuf[PLAYLIST_PATH_MAX];
  const char* aRef = a;
  const char* bRef = b;
  if (entryIsDir(a)) {
    strlcpy(abuf, a, sizeof(abuf));
    size_t n = strlen(abuf);
    if (n > 0 && abuf[n - 1] == '/') abuf[n - 1] = '\0';
    aRef = abuf;
  }
  if (entryIsDir(b)) {
    strlcpy(bbuf, b, sizeof(bbuf));
    size_t n = strlen(bbuf);
    if (n > 0 && bbuf[n - 1] == '/') bbuf[n - 1] = '\0';
    bRef = bbuf;
  }
  const char* aName = strrchr(aRef, '/'); aName = aName ? aName + 1 : aRef;
  const char* bName = strrchr(bRef, '/'); bName = bName ? bName + 1 : bRef;
  return playlistNaturalCmp(aName, bName);
}

// Format an entry for the song list. Files render as their basename; dirs as
// "[name]"; the parent entry as "[..]". `out` must be at least
// PLAYLIST_PATH_MAX + 4 bytes.
static const char* formatPlaylistLabel(const char* path, char* out, size_t outSize) {
  if (entryIsParent(path)) {
    strlcpy(out, "[..]", outSize);
    return out;
  }
  if (entryIsDir(path)) {
    char tmp[PLAYLIST_PATH_MAX];
    strlcpy(tmp, path, sizeof(tmp));
    size_t n = strlen(tmp);
    if (n > 0 && tmp[n - 1] == '/') tmp[n - 1] = '\0';
    const char* slash = strrchr(tmp, '/');
    const char* base = slash ? slash + 1 : tmp;
    snprintf(out, outSize, "[%s]", base);
    return out;
  }
  const char* slash = strrchr(path, '/');
  return slash ? slash + 1 : path;
}

// Strip the last "/segment" off dir, capped at root. Used to ascend on "..".
static void playlistGoUp(char* dir, const char* root) {
  size_t rootLen = strlen(root);
  size_t n = strlen(dir);
  if (n <= rootLen) { strlcpy(dir, root, PLAYLIST_PATH_MAX); return; }
  while (n > 0 && dir[n - 1] == '/') dir[--n] = '\0';
  char* slash = strrchr(dir, '/');
  if (!slash || (size_t)(slash - dir) < rootLen) {
    strlcpy(dir, root, PLAYLIST_PATH_MAX);
  } else {
    *slash = '\0';
  }
}

// Reserve the playlist buffer in PSRAM on first use. Returns false if PSRAM
// allocation fails — callers must skip scanning in that case rather than
// dereferencing a null pointer.
static bool ensureSmfPlaylistBuffer() {
  if (smfPlaylist) return true;
  size_t bytes = (size_t)SMF_MAX_FILES * PLAYLIST_PATH_MAX;
  smfPlaylist = (char(*)[PLAYLIST_PATH_MAX])heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
  if (!smfPlaylist) {
    Serial.printf("[mem] SMF playlist PSRAM alloc failed (%u bytes)\n", (unsigned)bytes);
    return false;
  }
  return true;
}

static bool ensureMp3PlaylistBuffer() {
  if (mp3Playlist) return true;
  size_t bytes = (size_t)MP3_MAX_FILES * PLAYLIST_PATH_MAX;
  mp3Playlist = (char(*)[PLAYLIST_PATH_MAX])heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
  if (!mp3Playlist) {
    Serial.printf("[mem] MP3 playlist PSRAM alloc failed (%u bytes)\n", (unsigned)bytes);
    return false;
  }
  return true;
}

static void scanSmfFiles() {
  smfPlaylistCount = 0;
  smfListScroll = 0;
  if (!ensureStorage()) return;
  if (!ensureSmfPlaylistBuffer()) return;

  if (smfCurrentDir[0] == '\0') strlcpy(smfCurrentDir, SMF_FOLDER, sizeof(smfCurrentDir));

  File root = SD.open(smfCurrentDir);
  if (!root || !root.isDirectory()) {
    if (root) root.close();
    // currentDir went stale (deleted?). Fall back to SMF_FOLDER and retry.
    if (strcmp(smfCurrentDir, SMF_FOLDER) != 0) {
      strlcpy(smfCurrentDir, SMF_FOLDER, sizeof(smfCurrentDir));
      root = SD.open(smfCurrentDir);
      if (!root || !root.isDirectory()) {
        if (root) root.close();
        return;
      }
    } else {
      return;
    }
  }

  // Synthetic parent entry when inside a subfolder.
  if (strcmp(smfCurrentDir, SMF_FOLDER) != 0 && smfPlaylistCount < SMF_MAX_FILES) {
    strlcpy(smfPlaylist[smfPlaylistCount], "..", PLAYLIST_PATH_MAX);
    smfPlaylistCount++;
  }

  while (smfPlaylistCount < SMF_MAX_FILES) {
    File entry = root.openNextFile();
    if (!entry) break;
    const char* name = entry.name();
    if (name && name[0] != '.') {
      if (entry.isDirectory()) {
        // Trailing slash marks the entry as a directory for the touch /
        // sort / draw paths.
        snprintf(smfPlaylist[smfPlaylistCount], PLAYLIST_PATH_MAX,
                 "%s/%s/", smfCurrentDir, name);
        smfPlaylistCount++;
      } else if (nameHasExt(name, ".mid") || nameHasExt(name, ".smf")) {
        snprintf(smfPlaylist[smfPlaylistCount], PLAYLIST_PATH_MAX,
                 "%s/%s", smfCurrentDir, name);
        smfPlaylistCount++;
      }
    }
    entry.close();
  }
  root.close();

  if (smfPlaylistCount > 1) {
    qsort(smfPlaylist, smfPlaylistCount, PLAYLIST_PATH_MAX, playlistEntryCmp);
  }

  // Auto-load only if the previously selected slot still points to a file.
  // After folder navigation smfCurrentTrack is reset to -1, so we won't
  // accidentally try to "play" a directory entry.
  if (smfPlaylistCount > 0 && smfCurrentTrack >= 0 &&
      smfCurrentTrack < smfPlaylistCount &&
      entryIsFile(smfPlaylist[smfCurrentTrack])) {
    loadSmfTrack(smfCurrentTrack);
  }
}

static void resetSmfKeyboard() {
  memset(smfActiveVelocity, 0, sizeof(smfActiveVelocity));
  memset(smfChannelVelocity, 0, sizeof(smfChannelVelocity));
  memset(smfMonitorKeyDirty, 0, sizeof(smfMonitorKeyDirty));
  smfMonitorDirtyAll = true;
}

static void refreshSmfAggregateNote(uint8_t note) {
  uint8_t vel = 0;
  for (uint8_t ch = 0; ch < 16; ++ch) {
    if (smfChannelVelocity[ch][note] > vel) vel = smfChannelVelocity[ch][note];
  }
  smfActiveVelocity[note] = vel;
}

static void clearSmfChannelNotes(uint8_t channel) {
  if (channel >= 16) return;
  for (uint8_t note = 0; note < 128; ++note) {
    if (smfChannelVelocity[channel][note]) {
      smfChannelVelocity[channel][note] = 0;
      refreshSmfAggregateNote(note);
      invalidateSmfMonitorKey(channel, note);
    }
  }
}

static void closeSmf() {
  smf.pause(true);
  smf.close();
  smfLoaded = false;
  smfPlaying = false;
  smfPausedElapsedMs = 0;
  resetSmfKeyboard();
}

static bool loadSmfTrack(int index) {
  if (!ensureStorage() || !midiFsReady || smfPlaylistCount == 0) return false;
  if (index < 0 || index >= smfPlaylistCount) return false;

  closeSmf();
  smf.begin(&midiSd);
  smf.setMidiHandler(smfMidiEventHandler);
  smf.setSysexHandler(smfSysexEventHandler);
  smf.setMetaHandler(smfMetaEventHandler);
  smf.looping(smfLoop);

  smfCurrentTrack = index;
  const char* path = smfPlaylist[index];
  const char* slash = strrchr(path, '/');
  const char* name = slash ? slash + 1 : path;
  strncpy(smfCurrentName, name, sizeof(smfCurrentName) - 1);
  smfCurrentName[sizeof(smfCurrentName) - 1] = '\0';

  int err = smf.load(path);
  smfLoaded = (err == MD_MIDIFile::E_OK);
  if (!smfLoaded) {
    Serial.printf("[SMF] load failed %s err=%d\n", path, err);
    // Some failure paths inside MD_MIDIFile::load leave _fd open or the
    // track table partially populated. Force a clean close so the next
    // attempt starts from a known state and we never leak a file handle.
    smf.close();
    smfCurrentName[0] = '\0';
    return false;
  }
  invalidateSmfMonitorAll();
  needPartialUpdate = true;
  return true;
}

static void playSmf(bool sendReset) {
  if (!smfLoaded) {
    if (!loadSmfTrack(smfCurrentTrack)) return;
  }
  resetSmfKeyboard();
  sendAllNotesOff();
  if (sendReset && g_config.smfStartGSReset) {
    sendGSReset();
    delay(30);
  }
  smf.restart();
  smf.pause(false);
  smf.looping(smfLoop);
  smfPlaying = true;
  smfPausedElapsedMs = 0;
  smfPlaybackStartMs = millis();
  invalidateSmfMonitorAll();
  needFullRedraw = true;
  needPartialUpdate = true;
}

static void stopSmf() {
  if (!smfLoaded) return;
  smf.pause(true);
  smfPlaying = false;
  smfPausedElapsedMs = 0;
  sendAllNotesOff();
  resetSmfKeyboard();
  needPartialUpdate = true;
}

static void smfMidiEventHandler(midi_event* pev) {
  if (pev == nullptr) return;
  // Defensive: pev->size is uint8_t with a 4-byte data buffer in the
  // library, but a corrupt SMF or a future library change could give us
  // values outside [1..4]. Clamp before iterating so we never read past
  // pev->data.
  uint8_t messageSize = pev->size;
  if (messageSize == 0) return;
  if (messageSize > 4) messageSize = 4;

  uint8_t status = (pev->data[0] & 0xF0) | (pev->channel & 0x0F);
  uint8_t channel = pev->channel & 0x0F;
  uint8_t message[4];
  message[0] = status;
  for (uint8_t i = 1; i < messageSize; ++i) {
    message[i] = pev->data[i];
  }
  {
    Serial2TxLock lk;
    for (uint8_t i = 0; i < messageSize; ++i) {
      Serial2.write(message[i]);
      ++midiOutCount;
      if (!isHeartbeatByte(message[i])) ++midiOutRealCount;
    }
  }

  uint8_t type = status & 0xF0;
  if ((type == 0x90 || type == 0x80) && messageSize >= 3) {
    uint8_t note = message[1];
    uint8_t vel = message[2];
    bool on = (type == 0x90) && vel > 0;
    if (note < 128 && channel < 16) {
      smfChannelVelocity[channel][note] = on ? vel : 0;
      refreshSmfAggregateNote(note);
      invalidateSmfMonitorKey(channel, note);
    }
  } else if (type == 0xB0 && messageSize >= 3) {
    uint8_t cc = message[1];
    if (cc == 120 || cc == 123) {
      clearSmfChannelNotes(channel);
    }
  }
}

static void smfSysexEventHandler(sysex_event* pev) {
  if (pev == nullptr) return;
  // sysex_event::data is fixed at 50 bytes in the library; the size field
  // can in principle exceed that when a SysEx message was longer than the
  // buffer. Cap at the data array to avoid reading past it.
  uint16_t cap = (uint16_t)sizeof(pev->data);
  uint16_t n = pev->size;
  if (n > cap) n = cap;
  Serial2TxLock lk;
  for (uint16_t i = 0; i < n; ++i) {
    Serial2.write(pev->data[i]);
    ++midiOutCount;
    if (!isHeartbeatByte(pev->data[i])) ++midiOutRealCount;
  }
}

static void smfMetaEventHandler(const meta_event* mev) {
  (void)mev;
}

static void processSmf() {
  if (!smfPlaying || !smfLoaded) return;
  uint32_t startUs = micros();
  bool advanced = false;
  // Process events in short bursts. Each MD_MIDIFile::getNextEvent advances
  // by at most a tick, so a single call is normally cheap, but if the file
  // has many simultaneous events (chord stacks etc.) it can take a while.
  // Cap the time spent here per loop iteration so audio/UI/touch remain
  // responsive and we never starve the IDLE task long enough to trip the
  // 5-second task watchdog.
  while ((micros() - startUs) < 4000UL) {
    // Serial2 (MIDI OUT) runs at 31.25 kbaud — only ~4 bytes drain per ms.
    // The TX buffer is 512 bytes. If it is nearly full, the next
    // Serial2.write() inside the event handler will block past our 4 ms
    // cap and starve the IDLE task long enough to trip the watchdog.
    // Hold off when there is not enough headroom for a worst-case event
    // (a 50-byte SysEx). The next loop iteration will retry once the UART
    // has drained.
    if (Serial2.availableForWrite() < 64) break;
    bool a = smf.getNextEvent();
    if (a) advanced = true;
    if (!a) break;
    if (smf.isEOF()) break;
  }
  if (advanced && smf.isEOF()) {
    if (smfLoop) {
      playSmf(false);
    } else {
      stopSmf();
    }
    needFullRedraw = true;
  }
}

static void scanMp3Files() {
  mp3PlaylistCount = 0;
  mp3ListScroll = 0;
  if (!ensureStorage()) return;
  if (!ensureMp3PlaylistBuffer()) return;

  if (mp3CurrentDir[0] == '\0') strlcpy(mp3CurrentDir, MP3_FOLDER, sizeof(mp3CurrentDir));

  File root = SD.open(mp3CurrentDir);
  if (!root || !root.isDirectory()) {
    if (root) root.close();
    if (strcmp(mp3CurrentDir, MP3_FOLDER) != 0) {
      strlcpy(mp3CurrentDir, MP3_FOLDER, sizeof(mp3CurrentDir));
      root = SD.open(mp3CurrentDir);
      if (!root || !root.isDirectory()) {
        if (root) root.close();
        return;
      }
    } else {
      return;
    }
  }

  if (strcmp(mp3CurrentDir, MP3_FOLDER) != 0 && mp3PlaylistCount < MP3_MAX_FILES) {
    strlcpy(mp3Playlist[mp3PlaylistCount], "..", PLAYLIST_PATH_MAX);
    mp3PlaylistCount++;
  }

  while (mp3PlaylistCount < MP3_MAX_FILES) {
    File entry = root.openNextFile();
    if (!entry) break;
    const char* name = entry.name();
    if (name && name[0] != '.') {
      if (entry.isDirectory()) {
        snprintf(mp3Playlist[mp3PlaylistCount], PLAYLIST_PATH_MAX,
                 "%s/%s/", mp3CurrentDir, name);
        mp3PlaylistCount++;
      } else if (nameHasExt(name, ".mp3")) {
        snprintf(mp3Playlist[mp3PlaylistCount], PLAYLIST_PATH_MAX,
                 "%s/%s", mp3CurrentDir, name);
        mp3PlaylistCount++;
      }
    }
    entry.close();
  }
  root.close();

  if (mp3PlaylistCount > 1) {
    qsort(mp3Playlist, mp3PlaylistCount, PLAYLIST_PATH_MAX, playlistEntryCmp);
  }

  if (mp3PlaylistCount > 0 && mp3CurrentTrack >= 0 &&
      mp3CurrentTrack < mp3PlaylistCount &&
      entryIsFile(mp3Playlist[mp3CurrentTrack])) {
    const char* path = mp3Playlist[mp3CurrentTrack];
    const char* slash = strrchr(path, '/');
    const char* name = slash ? slash + 1 : path;
    strncpy(mp3CurrentName, name, sizeof(mp3CurrentName) - 1);
    mp3CurrentName[sizeof(mp3CurrentName) - 1] = '\0';
  } else {
    mp3CurrentName[0] = '\0';
  }
}

static void mp3MetadataCallback(void* cbData, const char* type, bool isUnicode, const char* str) {
  (void)cbData;
  (void)isUnicode;
  if (type == nullptr || str == nullptr || str[0] == '\0') return;

  if (strcmp(type, "Title") == 0 || strcmp(type, "TIT2") == 0) {
    strncpy(mp3Title, str, sizeof(mp3Title) - 1);
    mp3Title[sizeof(mp3Title) - 1] = '\0';
  } else if (strcmp(type, "Artist") == 0 || strcmp(type, "TPE1") == 0) {
    strncpy(mp3Artist, str, sizeof(mp3Artist) - 1);
    mp3Artist[sizeof(mp3Artist) - 1] = '\0';
  }
}

static bool startMp3Track(int index) {
  if (!ensureStorage() || mp3PlaylistCount == 0) return false;
  if (index < 0 || index >= mp3PlaylistCount) return false;

  stopMp3();
  mp3CurrentTrack = index;
  const char* path = mp3Playlist[index];
  const char* slash = strrchr(path, '/');
  const char* name = slash ? slash + 1 : path;
  strncpy(mp3CurrentName, name, sizeof(mp3CurrentName) - 1);
  mp3CurrentName[sizeof(mp3CurrentName) - 1] = '\0';
  mp3Title[0] = '\0';
  mp3Artist[0] = '\0';

  mp3File = new (std::nothrow) AudioFileSourceFS(SD, path);
  if (mp3File == nullptr) {
    Serial.println("[MP3] AudioFileSourceFS alloc failed");
    return false;
  }
  mp3Id3 = new (std::nothrow) AudioFileSourceID3(mp3File);
  if (mp3Id3 == nullptr) {
    Serial.println("[MP3] AudioFileSourceID3 alloc failed");
    mp3File->close();
    delete mp3File;
    mp3File = nullptr;
    return false;
  }
  mp3Id3->RegisterMetadataCB(mp3MetadataCallback, nullptr);
  M5.Speaker.setVolume(mp3Volume);
  mp3Playing = mp3Decoder.begin(mp3Id3, &mp3Out);
  if (!mp3Playing) {
    Serial.printf("[MP3] decoder.begin failed for %s\n", path);
    // Mirror stopMp3()'s cleanup path so we don't leave dangling pointers
    // attached to a half-initialised decoder.
    mp3Id3->close();
    delete mp3Id3;
    mp3Id3 = nullptr;
    mp3File->close();
    delete mp3File;
    mp3File = nullptr;
    mp3PlaybackStartMs = 0;
    mp3StaticDirty = true;
    mp3VisualDirty = true;
    return false;
  }
  mp3PlaybackStartMs = millis();
  mp3CassetteAngle = 0.0f;
  mp3LastAnimMs = millis();
  mp3StaticDirty = true;
  mp3VisualDirty = true;
  return mp3Playing;
}

static void stopMp3() {
  if (mp3Playing) {
    mp3Decoder.stop();
    mp3Out.stop();
  }
  if (mp3Id3 != nullptr) {
    mp3Id3->close();
    delete mp3Id3;
    mp3Id3 = nullptr;
  }
  if (mp3File != nullptr) {
    mp3File->close();
    delete mp3File;
    mp3File = nullptr;
  }
  mp3Playing = false;
  mp3PlaybackStartMs = 0;
  mp3StaticDirty = true;
  mp3VisualDirty = true;
}

static void processMp3() {
  if (!mp3Playing) return;
  if (!mp3Decoder.loop() || !mp3Decoder.isRunning()) {
    stopMp3();
    needPartialUpdate = true;
    return;
  }
  uint32_t now = millis();
  if (now - mp3LastAnimMs >= 120) {
    mp3LastAnimMs = now;
    mp3CassetteAngle += 0.25f;
    mp3VisualDirty = true;
  }
}

static void handleSmfTouch(int x, int y) {
  if (smfPlaying) return;
  const int lineH = 44;
  int visible = (smfListArea.h - 84) / lineH;
  if (visible < 1) visible = 1;
  int pageStep = max(1, visible - 1);
  int maxScroll = max(0, smfPlaylistCount - visible);
  if (hit(smfListPageUpBtn, x, y)) {
    if (smfListScroll > 0) {
      smfListScroll = max(0, smfListScroll - pageStep);
      needFullRedraw = true;
    }
    return;
  }
  if (hit(smfListUpBtn, x, y)) {
    if (smfListScroll > 0) {
      smfListScroll--;
      needFullRedraw = true;
    }
    return;
  }
  if (hit(smfListDownBtn, x, y)) {
    if (smfListScroll < maxScroll) {
      smfListScroll++;
      needFullRedraw = true;
    }
    return;
  }
  if (hit(smfListPageDownBtn, x, y)) {
    if (smfListScroll < maxScroll) {
      smfListScroll = min(maxScroll, smfListScroll + pageStep);
      needFullRedraw = true;
    }
    return;
  }
  if (hit(smfListArea, x, y)) {
    int top = smfListArea.y + 84;
    int idx = smfListScroll + ((y - top) / lineH);
    if (y >= top && idx >= 0 && idx < smfPlaylistCount) {
      const char* path = smfPlaylist[idx];
      if (entryIsParent(path)) {
        playlistGoUp(smfCurrentDir, SMF_FOLDER);
        smfCurrentTrack = -1;
        scanSmfFiles();
        needFullRedraw = true;
      } else if (entryIsDir(path)) {
        // Drop the trailing slash so the value matches what SD.open() expects.
        strlcpy(smfCurrentDir, path, sizeof(smfCurrentDir));
        size_t n = strlen(smfCurrentDir);
        if (n > 0 && smfCurrentDir[n - 1] == '/') smfCurrentDir[n - 1] = '\0';
        smfCurrentTrack = -1;
        scanSmfFiles();
        needFullRedraw = true;
      } else {
        loadSmfTrack(idx);
        needFullRedraw = true;
      }
    }
  }
}

static void handleMp3Touch(int x, int y) {
  const int lineH = 44;
  int visible = (mp3ListArea.h - 84) / lineH;
  if (visible < 1) visible = 1;
  int pageStep = max(1, visible - 1);
  int maxScroll = max(0, mp3PlaylistCount - visible);
  if (hit(mp3ListPageUpBtn, x, y)) {
    if (mp3ListScroll > 0) {
      mp3ListScroll = max(0, mp3ListScroll - pageStep);
      mp3StaticDirty = true;
      needPartialUpdate = true;
    }
    return;
  }
  if (hit(mp3ListUpBtn, x, y)) {
    if (mp3ListScroll > 0) {
      mp3ListScroll--;
      mp3StaticDirty = true;
      needPartialUpdate = true;
    }
    return;
  }
  if (hit(mp3ListDownBtn, x, y)) {
    if (mp3ListScroll < maxScroll) {
      mp3ListScroll++;
      mp3StaticDirty = true;
      needPartialUpdate = true;
    }
    return;
  }
  if (hit(mp3ListPageDownBtn, x, y)) {
    if (mp3ListScroll < maxScroll) {
      mp3ListScroll = min(maxScroll, mp3ListScroll + pageStep);
      mp3StaticDirty = true;
      needPartialUpdate = true;
    }
    return;
  }
  if (hit(mp3ListArea, x, y)) {
    int top = mp3ListArea.y + 84;
    int idx = mp3ListScroll + ((y - top) / lineH);
    if (y >= top && idx >= 0 && idx < mp3PlaylistCount) {
      const char* path = mp3Playlist[idx];
      if (entryIsParent(path)) {
        playlistGoUp(mp3CurrentDir, MP3_FOLDER);
        mp3CurrentTrack = -1;
        scanMp3Files();
        needFullRedraw = true;
      } else if (entryIsDir(path)) {
        strlcpy(mp3CurrentDir, path, sizeof(mp3CurrentDir));
        size_t n = strlen(mp3CurrentDir);
        if (n > 0 && mp3CurrentDir[n - 1] == '/') mp3CurrentDir[n - 1] = '\0';
        mp3CurrentTrack = -1;
        scanMp3Files();
        needFullRedraw = true;
      } else {
        startMp3Track(idx);
        needFullRedraw = true;
      }
    }
  }
}

static void handleTouch() {
  auto t = M5.Touch.getDetail();
  if (!t.wasPressed() && !t.wasHold()) return;  // one-shot tap / hold-begin only

  int x = t.x, y = t.y;

  // SRC fullscreen instrument picker swallows all touch input while open.
  // Only its own widgets (cells, prev/next/close) are reachable.
  if (g_srcPickerOpen) {
    handleSrcPickerTouch(x, y);
    return;
  }

  // ── Long-tap (wasHold ≥ default M5Unified hold threshold ~500 ms) on the
  //    toolbar AOFF button enters BASE_SET overlay. CONFIG_EDIT is reachable
  //    via the dedicated cfgEntryBtn header button (no long-tap fallback).
  if (t.wasHold()) {
    if (currentMode != BASE_SET_MODE && hit(btnAllOff, x, y)) {
      enterBaseSetMode();
      return;
    }
    // fall through — MIDI-page hold gestures (cycler keypad) are still active.
  }

  // ── Overlay touch dispatch ──
  // Header buttons (BASE / CONF / MIX / AppTabs / AOFF) stay live even while
  // an overlay is up. A tap on any of them cancels the overlay first, then
  // routes the tap to the normal toolbar handler so navigation isn't trapped.
  if (currentMode == CONFIG_EDIT_MODE || currentMode == BASE_SET_MODE) {
    bool headerTap = hit(baseEntryBtn, x, y)
                  || hit(cfgEntryBtn, x, y)
                  || hit(midiInputSourceBtn, x, y)
                  || hit(btnAllOff, x, y)
                  || hit(appTab[0], x, y)
                  || hit(appTab[1], x, y)
                  || hit(appTab[2], x, y);
    if (headerTap) {
      if (currentMode == CONFIG_EDIT_MODE) exitConfigEditMode(false, false);
      else                                  exitBaseSetMode();
      handleToolbarTouch(x, y);
      return;
    }
  }
  if (currentMode == CONFIG_EDIT_MODE) {
    processConfigEditTouch(x, y);
    return;
  }
  if (currentMode == BASE_SET_MODE) {
    processBaseSetTouch(x, y);
    return;
  }

  // Header now hosts the XPOSE/MIDI/PLAY app tabs — route header taps to the
  // same handler as the toolbar (it already gates by rect hit).
  if (y < toolbarArea.y + toolbarArea.h) {
    handleToolbarTouch(x, y);
    return;
  }
  if (currentApp == APP_MIDI) {
    processMidiManageTouch(t);
    return;
  }
  if (y >= navArea.y) {
    handleNavTouch(x, y);
    return;
  }
  // Otherwise it's content-area input.
  if (currentApp == APP_TRANSPOSE) {
    switch (currentMode) {
      case DIRECT_MODE:      handleDirectTouch(x, y);   break;
      case KEY_MODE:         handleKeyTouch(x, y);      break;
      case INSTANT_MODE:     handleInstantTouch(x, y);  break;
      case SEQUENCE_MODE:    handleSequenceTouch(x, y); break;
      default: break;
    }
  } else if ((currentApp == APP_PLAY && currentPlay == PLAY_SRC)) {
    handleSrcTouch(x, y);
  } else if ((currentApp == APP_PLAY && currentPlay == PLAY_SMF)) {
    handleSmfTouch(x, y);
  } else {
    handleMp3Touch(x, y);
  }
}

// =================================================================
//  Transpose logic (ported from the M5Core2 version)
// =================================================================
static int8_t clampTranspose(int8_t v) {
  if (v < -12) return -12;
  if (v >  12) return  12;
  return v;
}

static void handleTransposeChange(int8_t newValue) {
  newValue = clampTranspose(newValue);
  if (newValue == transposeValue) return;

  if (allNotesOffEnabled) {
    sendAllNotesOff();
    transposeValue = newValue;
    needPartialUpdate = true;
    return;
  }

  // Smooth transpose: remember which notes were on at the old transpose so
  // their matching Note-Off messages can undo the correct offset.
  for (int i = 0; i < TRACKED_NOTE_STATE_COUNT; i++) savedNoteStates[i] = currentNoteStates[i];
  transposeValue = newValue;
  needPartialUpdate = true;
}

static int getPianoKeyIndex(uint8_t midiNote) {
  if (midiNote < PIANO_LOWEST_NOTE || midiNote > PIANO_HIGHEST_NOTE) return -1;
  return midiNote - PIANO_LOWEST_NOTE;
}

static int getTrackedNoteStateIndex(uint8_t midiNote, uint8_t channel) {
  int pianoKeyIndex = getPianoKeyIndex(midiNote);
  if (pianoKeyIndex < 0 || channel >= MIDI_CHANNEL_COUNT) return -1;
  return pianoKeyIndex * MIDI_CHANNEL_COUNT + channel;
}

static void clearTrackedNoteStates() {
  for (int i = 0; i < TRACKED_NOTE_STATE_COUNT; i++) {
    currentNoteStates[i].isActive = false;
    savedNoteStates[i].isActive = false;
  }
}

static bool isMidiInputIdle(uint32_t now) {
  return getCurrentMidiInputAvailable() == 0 && (now - g_lastMidiInputAt) >= 250;
}

static void processDeferredStorageTasks(uint32_t now) {
  if (g_seqSaveUiState != SEQ_SAVE_UI_IDLE && g_seqSaveUiUntil != 0 && now >= g_seqSaveUiUntil) {
    g_seqSaveUiState = SEQ_SAVE_UI_IDLE;
    g_seqSaveUiUntil = 0;
    needFullRedraw = true;
  }

  if (!g_seqSavePending || !isMidiInputIdle(now)) return;
  // SD.h (used for the sequence file) and SdFat (used for SMF) share the
  // same SPI bus and CS pin. Issuing an SD write while a player is mid-
  // stream can corrupt the bus state and stall the player (or worse, crash
  // when both layers think they own the chip-select). Hold the save off
  // until playback is stopped — sequence saves are user-initiated, and the
  // UI keeps them in QUEUED state until they actually run.
  if (smfPlaying || mp3Playing) return;

  g_seqSavePending = false;
  bool ok = saveSequencesToSD();
  g_seqSaveUiState = ok ? SEQ_SAVE_UI_OK : SEQ_SAVE_UI_ERR;
  g_seqSaveUiUntil = now + (ok ? 1200 : 1600);
  needFullRedraw = true;
}

static int getMIDIMessageLength(uint8_t status) {
  uint8_t type = status & 0xF0;
  switch (type) {
    case 0x80: case 0x90: case 0xA0: case 0xB0: case 0xE0: return 3;
    case 0xC0: case 0xD0: return 2;
    default: return 1;
  }
}

// ==== MIDI Manager engine ====
static int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static const char* getMidiKindLabel(MidiMessageKind kind) {
  static const char* labels[MIDI_KIND_COUNT] = {
    "NoteOff","NoteOn","KeyPrs","PrgChg","CtrlChg","ChPrs","Bend",
    "SysEx","MTC","SongPos","SongSel","TuneReq",
    "Clock","Start","Cont","Stop","ActSn","Reset"
  };
  if (kind < 0 || kind >= MIDI_KIND_COUNT) return "Unknown";
  return labels[kind];
}

static bool midiKindHasChannel(MidiMessageKind kind) {
  return kind <= MIDI_KIND_PITCH_BEND;
}

static uint8_t getMidiStatusForKind(MidiMessageKind kind, uint8_t channel) {
  switch (kind) {
    case MIDI_KIND_NOTE_OFF:         return 0x80 | (channel & 0x0F);
    case MIDI_KIND_NOTE_ON:          return 0x90 | (channel & 0x0F);
    case MIDI_KIND_KEY_PRESSURE:     return 0xA0 | (channel & 0x0F);
    case MIDI_KIND_CONTROL_CHANGE:   return 0xB0 | (channel & 0x0F);
    case MIDI_KIND_PROGRAM_CHANGE:   return 0xC0 | (channel & 0x0F);
    case MIDI_KIND_CHANNEL_PRESSURE: return 0xD0 | (channel & 0x0F);
    case MIDI_KIND_PITCH_BEND:       return 0xE0 | (channel & 0x0F);
    case MIDI_KIND_SYSTEM_EXCLUSIVE: return 0xF0;
    case MIDI_KIND_MIDI_TIME_CODE:   return 0xF1;
    case MIDI_KIND_SONG_POSITION:    return 0xF2;
    case MIDI_KIND_SONG_SELECT:      return 0xF3;
    case MIDI_KIND_TUNE_REQUEST:     return 0xF6;
    case MIDI_KIND_CLOCK:            return 0xF8;
    case MIDI_KIND_START:            return 0xFA;
    case MIDI_KIND_CONTINUE:         return 0xFB;
    case MIDI_KIND_STOP:             return 0xFC;
    case MIDI_KIND_ACTIVE_SENSE:     return 0xFE;
    case MIDI_KIND_SYSTEM_RESET:     return 0xFF;
    default:                         return 0x00;
  }
}

static MidiMessageKind getMidiKindFromStatus(uint8_t status) {
  if ((status & 0xF0) == 0x80) return MIDI_KIND_NOTE_OFF;
  if ((status & 0xF0) == 0x90) return MIDI_KIND_NOTE_ON;
  if ((status & 0xF0) == 0xA0) return MIDI_KIND_KEY_PRESSURE;
  if ((status & 0xF0) == 0xB0) return MIDI_KIND_CONTROL_CHANGE;
  if ((status & 0xF0) == 0xC0) return MIDI_KIND_PROGRAM_CHANGE;
  if ((status & 0xF0) == 0xD0) return MIDI_KIND_CHANNEL_PRESSURE;
  if ((status & 0xF0) == 0xE0) return MIDI_KIND_PITCH_BEND;
  switch (status) {
    case 0xF0: return MIDI_KIND_SYSTEM_EXCLUSIVE;
    case 0xF1: return MIDI_KIND_MIDI_TIME_CODE;
    case 0xF2: return MIDI_KIND_SONG_POSITION;
    case 0xF3: return MIDI_KIND_SONG_SELECT;
    case 0xF6: return MIDI_KIND_TUNE_REQUEST;
    case 0xF8: return MIDI_KIND_CLOCK;
    case 0xFA: return MIDI_KIND_START;
    case 0xFB: return MIDI_KIND_CONTINUE;
    case 0xFC: return MIDI_KIND_STOP;
    case 0xFE: return MIDI_KIND_ACTIVE_SENSE;
    case 0xFF: return MIDI_KIND_SYSTEM_RESET;
    default:   return MIDI_KIND_SYSTEM_EXCLUSIVE;
  }
}

static uint8_t getMidiMessageLengthForKind(MidiMessageKind kind) {
  switch (kind) {
    case MIDI_KIND_NOTE_OFF:
    case MIDI_KIND_NOTE_ON:
    case MIDI_KIND_KEY_PRESSURE:
    case MIDI_KIND_CONTROL_CHANGE:
    case MIDI_KIND_PITCH_BEND:
    case MIDI_KIND_SONG_POSITION:        return 3;
    case MIDI_KIND_PROGRAM_CHANGE:
    case MIDI_KIND_CHANNEL_PRESSURE:
    case MIDI_KIND_MIDI_TIME_CODE:
    case MIDI_KIND_SONG_SELECT:          return 2;
    default:                             return 1;
  }
}

static bool midiKindSupportsData1(MidiMessageKind kind) {
  switch (kind) {
    case MIDI_KIND_NOTE_OFF:
    case MIDI_KIND_NOTE_ON:
    case MIDI_KIND_KEY_PRESSURE:
    case MIDI_KIND_CONTROL_CHANGE:
    case MIDI_KIND_PROGRAM_CHANGE:
    case MIDI_KIND_MIDI_TIME_CODE:
    case MIDI_KIND_SONG_SELECT:          return true;
    default:                             return false;
  }
}

static int getMidiValueMax(MidiMessageKind kind) {
  switch (kind) {
    case MIDI_KIND_PITCH_BEND:
    case MIDI_KIND_SONG_POSITION:        return 16383;
    default:                             return 127;
  }
}

static int getMidiPrimaryData(const MidiMessage& msg) {
  if (msg.length < 2) return 0;
  return msg.bytes[1];
}

static int getMidiValueData(const MidiMessage& msg) {
  if (msg.kind == MIDI_KIND_PITCH_BEND || msg.kind == MIDI_KIND_SONG_POSITION) {
    if (msg.length < 3) return 0;
    return (msg.bytes[1] & 0x7F) | ((msg.bytes[2] & 0x7F) << 7);
  }
  if (msg.length >= 3) return msg.bytes[2];
  if (msg.length >= 2) return msg.bytes[1];
  return 0;
}

static int mapMidiValueRange(int value, int srcMin, int srcMax, int dstMin, int dstMax) {
  srcMin = clampInt(srcMin, 0, 16383);
  srcMax = clampInt(srcMax, 0, 16383);
  dstMin = clampInt(dstMin, 0, 16383);
  dstMax = clampInt(dstMax, 0, 16383);
  if (srcMax < srcMin) srcMax = srcMin;
  if (dstMax < dstMin) dstMax = dstMin;
  value = clampInt(value, srcMin, srcMax);
  if (srcMax == srcMin) return dstMin;
  long num = (long)(value - srcMin) * (dstMax - dstMin);
  long den = (srcMax - srcMin);
  return dstMin + (int)(num / den);
}

static const char* getChannelLabel(int8_t channel, bool keepLabel) {
  static char label[8];
  if (channel < 0) return keepLabel ? "KEEP" : "ALL";
  snprintf(label, sizeof(label), "Ch%02d", channel + 1);
  return label;
}

static const char* getData1Label(int16_t data1, bool keepLabel) {
  static char label[8];
  if (data1 < 0) return keepLabel ? "KEEP" : "ANY";
  snprintf(label, sizeof(label), "%d", data1);
  return label;
}

static bool midiFilterRuleMatches(const MidiFilterRule& rule, const MidiMessage& msg) {
  if (!rule.enabled) return false;
  if (rule.kind != msg.kind) return false;
  if (rule.channel >= 0) {
    if (!msg.hasChannel) return false;
    if (rule.channel != msg.channel) return false;
  }
  return true;
}

static bool shouldAllowMidiMessage(const MidiMessage& msg) {
  if (midiFilterBypass) return true;
  for (int i = 0; i < midiFilterRuleCount; i++) {
    if (midiFilterRuleMatches(midiFilterRules[i], msg)) return false;
  }
  return true;
}

static bool midiMapperRuleMatches(const MidiMapperRule& rule, const MidiMessage& msg) {
  if (!rule.enabled) return false;
  if (rule.srcKind != msg.kind) return false;
  if (rule.srcChannel >= 0) {
    if (!msg.hasChannel) return false;
    if (rule.srcChannel != msg.channel) return false;
  }
  if (rule.srcData1 >= 0 && getMidiPrimaryData(msg) != rule.srcData1) return false;
  int value = getMidiValueData(msg);
  if (value < rule.srcMin || value > rule.srcMax) return false;
  return true;
}

static MidiMessage buildMappedMidiMessage(const MidiMapperRule& rule, const MidiMessage& srcMsg) {
  MidiMessage dstMsg = srcMsg;
  dstMsg.kind = rule.dstKind;
  dstMsg.hasChannel = midiKindHasChannel(rule.dstKind);
  dstMsg.length = getMidiMessageLengthForKind(rule.dstKind);

  uint8_t outChannel = 0;
  if (dstMsg.hasChannel) {
    if (rule.dstChannel >= 0) outChannel = rule.dstChannel;
    else if (srcMsg.hasChannel) outChannel = srcMsg.channel;
  }
  dstMsg.channel = dstMsg.hasChannel ? (int8_t)outChannel : -1;
  dstMsg.bytes[0] = getMidiStatusForKind(rule.dstKind, outChannel);

  int srcPrimary = getMidiPrimaryData(srcMsg);
  int srcValue = getMidiValueData(srcMsg);
  int mappedValue = mapMidiValueRange(srcValue, rule.srcMin, rule.srcMax, rule.dstMin, rule.dstMax);
  int primaryValue = (rule.dstData1 >= 0) ? rule.dstData1 : srcPrimary;

  if (dstMsg.length == 2) {
    if (midiKindSupportsData1(rule.dstKind))
      dstMsg.bytes[1] = clampInt(primaryValue, 0, 127);
    else
      dstMsg.bytes[1] = clampInt(mappedValue, 0, 127);
  } else if (dstMsg.length == 3) {
    if (rule.dstKind == MIDI_KIND_PITCH_BEND || rule.dstKind == MIDI_KIND_SONG_POSITION) {
      int v = clampInt(mappedValue, 0, 16383);
      dstMsg.bytes[1] = v & 0x7F;
      dstMsg.bytes[2] = (v >> 7) & 0x7F;
    } else {
      dstMsg.bytes[1] = clampInt(primaryValue, 0, 127);
      dstMsg.bytes[2] = clampInt(mappedValue, 0, 127);
    }
  }
  return dstMsg;
}

static MidiMessage applyMidiMapper(const MidiMessage& srcMsg) {
  if (midiMapperBypass) return srcMsg;
  for (int i = 0; i < midiMapperRuleCount; i++) {
    if (midiMapperRuleMatches(midiMapperRules[i], srcMsg)) {
      return buildMappedMidiMessage(midiMapperRules[i], srcMsg);
    }
  }
  return srcMsg;
}

static void initMidiManagementDefaults() {
  midiFilterRuleCount   = 1;
  midiMapperRuleCount   = 1;
  midiSelectedFilterRule = 0;
  midiSelectedMapperRule = 0;
  midiFilterRules[0] = { false, MIDI_KIND_CONTROL_CHANGE, -1 };
  midiMapperRules[0] = { false, MIDI_KIND_CONTROL_CHANGE, -1, -1, 0, 127,
                                MIDI_KIND_CONTROL_CHANGE, -1, -1, 0, 127 };
}

// Populate two filter + two mapper rules with deterministic test content.
// Used by `LOAD TESTRULES` USB-serial command from the regression script.
// filter[0] = block all PitchBend
// filter[1] = block all ControlChange
// mapper[0] = NoteOn Ch 1 -> NoteOn Ch 2 (keep velocity)
// mapper[1] = NoteOn Ch 3 velocity 0..127 -> 0..63 (halve)
// Disjoint by message subset so the mapper's first-match-wins still lets
// the x2 phase observe both rules acting independently.
static void loadTestRules() {
  midiFilterRuleCount = 2;
  midiSelectedFilterRule = 0;
  midiFilterRules[0] = { false, MIDI_KIND_PITCH_BEND, -1 };
  midiFilterRules[1] = { false, MIDI_KIND_CONTROL_CHANGE, -1 };

  midiMapperRuleCount = 2;
  midiSelectedMapperRule = 0;
  midiMapperRules[0] = { false, MIDI_KIND_NOTE_ON, 0,  -1, 0, 127,
                                MIDI_KIND_NOTE_ON, 1,  -1, 0, 127 };
  midiMapperRules[1] = { false, MIDI_KIND_NOTE_ON, 2,  -1, 0, 127,
                                MIDI_KIND_NOTE_ON, -1, -1, 0, 63  };
}

static void addDefaultFilterRule() {
  if (midiFilterRuleCount >= MAX_FILTER_RULES) return;
  midiFilterRules[midiFilterRuleCount] = { false, MIDI_KIND_NOTE_OFF, -1 };
  midiSelectedFilterRule = midiFilterRuleCount;
  midiFilterRuleCount++;
}

static void addDefaultMapperRule() {
  if (midiMapperRuleCount >= MAX_MAPPER_RULES) return;
  midiMapperRules[midiMapperRuleCount] = { false, MIDI_KIND_CONTROL_CHANGE, -1, -1, 0, 127,
                                                  MIDI_KIND_CONTROL_CHANGE, -1, -1, 0, 127 };
  midiSelectedMapperRule = midiMapperRuleCount;
  midiMapperRuleCount++;
}

static void deleteSelectedFilterRule() {
  if (midiFilterRuleCount <= 1) {
    midiFilterRules[0].enabled = false;
    return;
  }
  for (int i = midiSelectedFilterRule; i < midiFilterRuleCount - 1; i++)
    midiFilterRules[i] = midiFilterRules[i + 1];
  midiFilterRuleCount--;
  if (midiSelectedFilterRule >= midiFilterRuleCount)
    midiSelectedFilterRule = midiFilterRuleCount - 1;
}

static void deleteSelectedMapperRule() {
  if (midiMapperRuleCount <= 1) {
    midiMapperRules[0].enabled = false;
    return;
  }
  for (int i = midiSelectedMapperRule; i < midiMapperRuleCount - 1; i++)
    midiMapperRules[i] = midiMapperRules[i + 1];
  midiMapperRuleCount--;
  if (midiSelectedMapperRule >= midiMapperRuleCount)
    midiSelectedMapperRule = midiMapperRuleCount - 1;
}

static void formatMidiFilterRuleSummary(const MidiFilterRule& rule, int index, char* out, size_t outSize) {
  snprintf(out, outSize, "%d - %s %s", index + 1,
           getChannelLabel(rule.channel, false), getMidiKindLabel(rule.kind));
}

static void formatMidiMapperRuleSummary(const MidiMapperRule& rule, int index, char* out, size_t outSize) {
  char data1[8];
  if (rule.srcData1 < 0) snprintf(data1, sizeof(data1), "ANY");
  else snprintf(data1, sizeof(data1), "%d", rule.srcData1);
  snprintf(out, outSize, "%d - %s %s %s>%s %s",
           index + 1,
           getMidiKindLabel(rule.srcKind), getChannelLabel(rule.srcChannel, false),
           data1,
           getMidiKindLabel(rule.dstKind), getChannelLabel(rule.dstChannel, true));
}

// MIDI 出力 (FILTER → MAPPER → Transpose の最終出口で呼ばれる)
static void outputMidiMessage(const MidiMessage& msg);

// パース済み 1 メッセージを FILTER / MAPPER に通して出力
static void handleParsedMidiMessage(const MidiMessage& inMsg) {
  if (!shouldAllowMidiMessage(inMsg)) return;          // FILTER で破棄
  MidiMessage mapped = applyMidiMapper(inMsg);          // MAPPER で書換
  // Auto-follow incoming MIDI channel for the SRC mode. We sample after the
  // MAPPER so a channel-remap is reflected on the UI; sampling before would
  // chase the input channel even when the user has remapped it.
  if (mapped.hasChannel && mapped.channel >= 0) {
    srcOnIncomingChannel((uint8_t)mapped.channel);
    // Live keyboard monitor for SRC mode: track Note On / Note Off per
    // channel so the active-channel keyboard panel can light up the keys
    // currently held. Note On with velocity 0 is a Note Off in the MIDI spec.
    if (mapped.kind == MIDI_KIND_NOTE_ON) {
      bool isOn = (mapped.length >= 3) && (mapped.bytes[2] > 0);
      srcKeyboardOnNote(mapped.channel, mapped.bytes[1], mapped.bytes[2], isOn);
    } else if (mapped.kind == MIDI_KIND_NOTE_OFF) {
      srcKeyboardOnNote(mapped.channel, mapped.bytes[1], 0, false);
    }
  }
  outputMidiMessage(mapped);                            // Transpose + 送信
}

static void sendMIDIMessage(uint8_t* buffer, int length) {
  Serial2TxLock lk;  // serialise with UI-thread sends so chord bytes don't tangle
  uint8_t status = buffer[0];
  uint8_t type = status & 0xF0;
  uint8_t channel = status & 0x0F;

  if ((type == 0x90 || type == 0x80) && length == 3) {
    uint8_t note = buffer[1];
    uint8_t vel  = buffer[2];
    bool isNoteOn = (type == 0x90) && (vel > 0);
    int idx = getTrackedNoteStateIndex(note, channel);

    if (isNoteOn) {
      int16_t transposed = note + transposeValue;
      if (transposed >= 0 && transposed <= 127) {
        Serial2.write(status);
        Serial2.write((uint8_t)transposed);
        Serial2.write(vel);
        midiOutCount += 3;
        midiOutRealCount += 3;   // NoteOn — real event
        if (idx >= 0) {
          currentNoteStates[idx] = { true, (int8_t)transposeValue, channel, vel };
        }
      }
    } else {
      int16_t transposed;
      if (idx >= 0) {
        if (currentNoteStates[idx].isActive) {
          transposed = note + currentNoteStates[idx].originalTranspose;
          currentNoteStates[idx].isActive = false;
        } else if (savedNoteStates[idx].isActive) {
          transposed = note + savedNoteStates[idx].originalTranspose;
          savedNoteStates[idx].isActive = false;
        } else {
          transposed = note + transposeValue;
        }
      } else {
        transposed = note + transposeValue;
      }
      if (transposed >= 0 && transposed <= 127) {
        Serial2.write(status);
        Serial2.write((uint8_t)transposed);
        Serial2.write(vel);
        midiOutCount += 3;
        midiOutRealCount += 3;   // NoteOff — real event
      }
    }
  } else {
    for (int i = 0; i < length; i++) {
      Serial2.write(buffer[i]);
      if (!isHeartbeatByte(buffer[i])) ++midiOutRealCount;
    }
    midiOutCount += length;
  }
}

// MidiMessage 経由 (FILTER 通過後) で実際に Serial2 へ送出する。
// Note On/Off は Transpose を適用、それ以外はそのまま流す。
static void outputMidiMessage(const MidiMessage& msg) {
  sendMIDIMessage((uint8_t*)msg.bytes, msg.length);
}

static void processMIDIByte(uint8_t data) {
  static uint8_t midiBuffer[3];
  static int bufferIndex = 0;
  static uint8_t runningStatus = 0;
  static uint8_t currentStatus = 0;
  static bool inSysEx = false;
  static bool allowCurrentSysEx = true;

  // External reset (input-source switch, USB unplug, etc.) — clear all
  // half-parsed state so we never treat a stray byte as a continuation of a
  // message that came from a different stream.
  if (g_midiParserResetRequested) {
    g_midiParserResetRequested = false;
    bufferIndex = 0;
    runningStatus = 0;
    currentStatus = 0;
    inSysEx = false;
    allowCurrentSysEx = true;
  }

  // Real-time
  if (data >= 0xF8) {
    MidiMessage msg;
    msg.bytes[0] = data; msg.length = 1;
    msg.kind = getMidiKindFromStatus(data);
    msg.hasChannel = false; msg.channel = -1;
    handleParsedMidiMessage(msg);
    return;
  }

  // SysEx パススルー (FILTER で SysEx を遮断する場合は MAPPER 不可、ペイロードを直書き)
  if (inSysEx) {
    if (data >= 0x80 && data != 0xF7) {
      // A non-realtime status byte during SysEx is illegal but real-world
      // devices emit them; close the SysEx and fall through so the new
      // status byte is parsed normally instead of being swallowed forever.
      if (allowCurrentSysEx) { Serial2.write((uint8_t)0xF7); midiOutCount++; midiOutRealCount++; }
      inSysEx = false;
    } else {
      if (allowCurrentSysEx) { Serial2.write(data); midiOutCount++; midiOutRealCount++; }
      if (data == 0xF7) inSysEx = false;
      return;
    }
  }
  if (data == 0xF0) {
    MidiMessage sx;
    sx.bytes[0] = 0xF0; sx.length = 1;
    sx.kind = MIDI_KIND_SYSTEM_EXCLUSIVE;
    sx.hasChannel = false; sx.channel = -1;
    inSysEx = true;
    allowCurrentSysEx = shouldAllowMidiMessage(sx);
    if (allowCurrentSysEx) { Serial2.write(data); midiOutCount++; }
    return;
  }

  if (data & 0x80) {
    currentStatus = data;
    midiBuffer[0] = data; bufferIndex = 1;
    if (data < 0xF0) runningStatus = data; else runningStatus = 0;
    int messageLength = getMIDIMessageLength(currentStatus);
    if (messageLength == 1) {
      MidiMessage msg;
      msg.bytes[0] = currentStatus; msg.length = 1;
      msg.kind = getMidiKindFromStatus(currentStatus);
      msg.hasChannel = false; msg.channel = -1;
      handleParsedMidiMessage(msg);
      bufferIndex = 0;
      return;
    }
  } else if (bufferIndex > 0) {
    midiBuffer[bufferIndex++] = data;
  } else if (runningStatus != 0) {
    currentStatus = runningStatus;
    midiBuffer[0] = currentStatus; midiBuffer[1] = data; bufferIndex = 2;
  } else {
    return;
  }

  int messageLength = getMIDIMessageLength(currentStatus);
  if (bufferIndex >= messageLength) {
    MidiMessage msg;
    msg.length = messageLength;
    msg.kind = getMidiKindFromStatus(currentStatus);
    msg.hasChannel = midiKindHasChannel(msg.kind);
    msg.channel = msg.hasChannel ? (int8_t)(currentStatus & 0x0F) : -1;
    for (int i = 0; i < messageLength; i++) msg.bytes[i] = midiBuffer[i];
    handleParsedMidiMessage(msg);
    bufferIndex = 0;
  }
}

static size_t getCurrentMidiInputAvailable() {
  if (midiInputSource == MIDI_INPUT_UNIT) {
    return (size_t)Serial2.available();
  }
  size_t usbAvailable = g_usbMidiMounted ? usbMidiRingAvailable() : 0;
  if (midiInputSource == MIDI_INPUT_USB) return usbAvailable;
  return (size_t)Serial2.available() + usbAvailable;
}

static void serviceUsbHost(uint32_t now) {
  (void)now;
  // ESP-IDF host events are pumped by the dedicated task on core 0.
}

static void processUsbMidiInput() {
  if (midiInputSource != MIDI_INPUT_USB && midiInputSource != MIDI_INPUT_MIX) return;
  if (!g_usbHostReady || !g_usbMidiMounted) return;

  bool sawInput = false;
  while (true) {
    int b = usbMidiRingPop();
    if (b < 0) break;
    sawInput = true;
    midiInCount++;
    if (!isHeartbeatByte((uint8_t)b)) midiInRealCount++;
    processMIDIByte((uint8_t)b);
  }
  if (sawInput) g_lastMidiInputAt = millis();
}

static void processMidiInput() {
  // Cap the work done per drain pass. A keyboard player driving heavy
  // running-status + chord traffic can sustain bursts that outpace the
  // 31.25 kbaud MIDI OUT, and Serial2.write() blocks once the TX ring is
  // full. Without a budget, the MIDI task could hold the Serial2 mutex too
  // long and starve UI-driven sends. Unconsumed bytes stay in the source
  // buffer (Serial2 RX ring or the USB ring) and are picked up on the next
  // pass. 3 ms keeps the worst-case latency under one MIDI clock tick at
  // 240 BPM.
  //
  // The Serial2 TX mutex is taken for the whole drain pass so SysEx
  // pass-through bytes (written byte-by-byte across many processMIDIByte
  // calls) cannot tangle with a UI-driven send.
  Serial2TxLock lk;
  const uint32_t startUs = micros();
  const uint32_t kBudgetUs = 3000UL;
  bool sawInput = false;

  if (midiInputSource == MIDI_INPUT_UNIT || midiInputSource == MIDI_INPUT_MIX) {
    while (Serial2.available()) {
      if ((micros() - startUs) >= kBudgetUs) {
        if (sawInput) g_lastMidiInputAt = millis();
        return;
      }
      uint8_t b = Serial2.read();
      sawInput = true;
      midiInCount++;
      if (!isHeartbeatByte(b)) midiInRealCount++;
      processMIDIByte(b);
    }
  }

  if (midiInputSource == MIDI_INPUT_USB || midiInputSource == MIDI_INPUT_MIX) {
    if (g_usbMidiMounted) {
      while (true) {
        if ((micros() - startUs) >= kBudgetUs) {
          if (sawInput) g_lastMidiInputAt = millis();
          return;
        }
        int b = usbMidiRingPop();
        if (b < 0) break;
        sawInput = true;
        midiInCount++;
        if (!isHeartbeatByte((uint8_t)b)) midiInRealCount++;
        processMIDIByte((uint8_t)b);
      }
    }
  }

  if (sawInput) g_lastMidiInputAt = millis();
}

// =================================================================
//  Dedicated MIDI I/O task (core 0)
// =================================================================
// Runs processMidiInput() on a separate FreeRTOS task pinned to a dedicated
// core at high priority, so heavy UI redraws on the main loop — like the SRC
// piano-roll repaint — don't starve the MIDI input drain. Without this, a
// 7–10 ms fillRect on the piano-roll panel could let the Serial2 RX buffer
// accumulate enough bytes that a chord Note Off in the middle of a burst
// gets cut off mid-message.
//
// The task wakes every 1 ms via vTaskDelay so it doesn't busy-spin. At
// 31.25 kbaud, 1 ms covers ~3 incoming bytes — plenty of headroom.
static TaskHandle_t g_midiTaskHandle = nullptr;

static void midi_io_task(void* /*pv*/) {
  for (;;) {
    processMidiInput();
    // 1 ms wake-up interval, expressed in tick-rate-agnostic units. With
    // arduino-esp32's default 1 kHz tick this is 1 tick; on builds with a
    // slower tick rate it floors to 0 and we yield without sleeping (still
    // bounded — the task is the highest-priority cooperative consumer here).
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

static void startMidiIoTask() {
  if (g_midiTaskHandle) return;
  // Pin to core 1 (alongside loopTask) so we don't preempt the USB host task
  // (priority 5, core 0). When the MIDI task ran on core 0 at priority 10
  // it starved usb_host_lib_handle_events()/usb_host_client_handle_events(),
  // causing USB-MIDI IN transfers to back up at the USB driver layer and
  // chord bytes to be silently dropped. Priority 10 on core 1 still
  // preempts the priority-1 loopTask whenever MIDI bytes need handling, so
  // heavy UI redraws can't starve MIDI either.
  xTaskCreatePinnedToCore(midi_io_task, "midi_io", 8192, nullptr, 10,
                          &g_midiTaskHandle, 1);
}

static void sendAllNotesOff() {
  Serial2TxLock lk;
  for (int ch = 0; ch < 16; ch++) {
    Serial2.write(0xB0 | ch); Serial2.write((uint8_t)123); Serial2.write((uint8_t)0);
    Serial2.write(0xB0 | ch); Serial2.write((uint8_t)120); Serial2.write((uint8_t)0);
    midiOutCount += 6;
    midiOutRealCount += 6;  // CC 120/123 are real events
  }
  clearTrackedNoteStates();
  // Also clear the SRC live-monitor state so any stuck visual notes (from a
  // dropped Note Off etc.) get released along with the synth's voices.
  for (int ch = 0; ch < 16; ++ch) {
    for (int note = 0; note < 128; ++note) {
      if (srcKeyVel[ch][note]) srcKeyDirty[ch][note] = true;
      srcKeyVel[ch][note] = 0;
      srcNoteStartMs[ch][note] = 0;
    }
  }
  srcKeyAllDirty = true;
}

static void sendGSReset() {
  Serial2TxLock lk;
  static const uint8_t gsReset[] = {
    0xF0, 0x41, 0x10, 0x42, 0x12, 0x40, 0x00, 0x7F, 0x00, 0x41, 0xF7
  };
  for (uint8_t b : gsReset) {
    Serial2.write(b);
    ++midiOutCount;
    if (!isHeartbeatByte(b)) ++midiOutRealCount;
  }
}

// Universal Non-Realtime SysEx "GM System On" (turns the synth into a clean
// GM-compliant state). This is what most synth manuals call "GM Reset".
static void sendGMReset() {
  Serial2TxLock lk;
  static const uint8_t gmReset[] = { 0xF0, 0x7E, 0x7F, 0x09, 0x01, 0xF7 };
  for (uint8_t b : gmReset) {
    Serial2.write(b);
    ++midiOutCount;
    if (!isHeartbeatByte(b)) ++midiOutRealCount;
  }
}

// =================================================================
//  SRC (Sound source) playback mode — Program Change / per-channel state
// =================================================================
static void sendUnitProgramChange(uint8_t channel, uint8_t program) {
  Serial2TxLock lk;
  Serial2.write((uint8_t)(0xC0 | (channel & 0x0F)));
  Serial2.write((uint8_t)(program & 0x7F));
  midiOutCount += 2;
  midiOutRealCount += 2;
}

static void sendUnitCC(uint8_t channel, uint8_t cc, uint8_t value) {
  Serial2TxLock lk;
  Serial2.write((uint8_t)(0xB0 | (channel & 0x0F)));
  Serial2.write((uint8_t)(cc & 0x7F));
  Serial2.write((uint8_t)(value & 0x7F));
  midiOutCount += 3;
  midiOutRealCount += 3;
}

static void sendUnitPitchBend(uint8_t channel, uint16_t bend14) {
  Serial2TxLock lk;
  Serial2.write((uint8_t)(0xE0 | (channel & 0x0F)));
  Serial2.write((uint8_t)(bend14 & 0x7F));
  Serial2.write((uint8_t)((bend14 >> 7) & 0x7F));
  midiOutCount += 3;
  midiOutRealCount += 3;
}

// Initialise per-channel SRC state. Optionally re-broadcasts the cached
// program / volume / pitch-bend / sustain values to the synth so the device
// reflects what the UI shows.
static void srcResetState(bool resendToSynth) {
  for (int i = 0; i < SRC_CH_COUNT; ++i) {
    srcProgram[i]   = 0;
    srcVolume[i]    = 100;
    srcPitchBend[i] = 8192;
    srcSustain[i]   = false;
  }
  if (resendToSynth) {
    for (int i = 0; i < SRC_CH_COUNT; ++i) {
      sendUnitProgramChange((uint8_t)i, srcProgram[i]);
      sendUnitCC((uint8_t)i, 7,  srcVolume[i]);
      sendUnitCC((uint8_t)i, 64, srcSustain[i] ? 127 : 0);
      sendUnitPitchBend((uint8_t)i, srcPitchBend[i]);
    }
  }
  srcDirtyAll = true;
}

// Pull SRC defaults from the device config and push them to the active
// channel. Other channels keep program 0 / vol 100 (clean GM defaults).
static void srcApplyConfigDefaults() {
  srcResetState(false);
  uint8_t ch = (uint8_t)(g_config.srcInitChannel >= 1 && g_config.srcInitChannel <= 16
                           ? (g_config.srcInitChannel - 1) : 0);
  srcChannel = ch;
  srcProgram[ch] = (uint8_t)(g_config.srcInitProgram & 0x7F);
  srcVolume[ch]  = (uint8_t)(g_config.srcInitVolume  & 0x7F);
  srcAutoFollow  = g_config.srcAutoChannel;
}

// User-pressed INIT: nuke synth state and resync everything.
static void srcHardReset() {
  sendAllNotesOff();
  sendGSReset();
  delay(30);
  // CC#121 Reset All Controllers — separate from the GS SysEx because some
  // synths only honour the SysEx for tone state, not for controllers.
  for (int ch = 0; ch < 16; ++ch) sendUnitCC((uint8_t)ch, 121, 0);
  srcResetState(true);
}

// Send Program Change for a given channel using the current cached program.
// For Ch10 the SAM2695 interprets the program as a drum-kit selector.
static void srcSendCurrentProgram(uint8_t channel) {
  if (channel >= SRC_CH_COUNT) return;
  sendUnitProgramChange(channel, srcProgram[channel]);
}

static void srcSetProgram(uint8_t channel, uint8_t program) {
  if (channel >= SRC_CH_COUNT) return;
  srcProgram[channel] = (uint8_t)(program & 0x7F);
  srcSendCurrentProgram(channel);
  srcDirtyAll = true;
}

static void srcSetVolume(uint8_t channel, uint8_t volume) {
  if (channel >= SRC_CH_COUNT) return;
  srcVolume[channel] = (uint8_t)(volume & 0x7F);
  sendUnitCC(channel, 7, srcVolume[channel]);
  srcDirtyAll = true;
}

static void srcSetPitchBend(uint8_t channel, uint16_t bend14) {
  if (channel >= SRC_CH_COUNT) return;
  if (bend14 > 16383) bend14 = 16383;
  srcPitchBend[channel] = bend14;
  sendUnitPitchBend(channel, bend14);
  srcDirtyAll = true;
}

static void srcSetSustain(uint8_t channel, bool on) {
  if (channel >= SRC_CH_COUNT) return;
  srcSustain[channel] = on;
  sendUnitCC(channel, 64, on ? 127 : 0);
  srcDirtyAll = true;
}

// Called from MIDI parsing when a channel-bearing message arrives. Updates
// the active channel highlight while srcAutoFollow is on. Cheap to call from
// any input path — only mutates state when the channel changes.
static void srcOnIncomingChannel(uint8_t channel) {
  if (!srcAutoFollow) return;
  if (channel >= SRC_CH_COUNT) return;
  if (srcChannel == channel) return;
  srcChannel = channel;
  srcKeyboardClearVisualState();
  srcDirtyAll = true;
  if (currentApp == APP_PLAY && currentPlay == PLAY_SRC) {
    needPartialUpdate = true;
  }
}

// =================================================================
//  Sequence file I/O on shared SPI SD
// =================================================================
static bool sdReady = false;

static bool ensureSD() {
  if (sdReady) return true;
  if (ensureStorage()) { sdReady = true; return true; }
  return false;
}

static bool saveSequencesToSD() {
  if (!ensureSD()) return false;
  File f = SD.open(SEQ_FILE, FILE_WRITE);
  if (!f) { Serial.println("[SEQ] open for write failed"); return false; }
  f.write('S'); f.write('Q');
  f.write((uint8_t)1);
  f.write((uint8_t)SEQ_PATTERN_COUNT);
  f.write((uint8_t)SEQ_STEP_COUNT);
  f.write((uint8_t)0);
  for (int p = 0; p < SEQ_PATTERN_COUNT; p++) {
    f.write((const uint8_t*)seqPatterns[p], SEQ_STEP_COUNT);
  }
  f.close();
  Serial.printf("[SEQ] Saved %d patterns\n", SEQ_PATTERN_COUNT);
  return true;
}

static bool loadSequencesFromSD() {
  if (!ensureSD()) return false;
  if (!SD.exists(SEQ_FILE)) { Serial.println("[SEQ] no file"); return false; }
  File f = SD.open(SEQ_FILE, FILE_READ);
  if (!f) return false;
  uint8_t header[6];
  if (f.read(header, 6) != 6) { f.close(); return false; }
  if (header[0] != 'S' || header[1] != 'Q' || header[2] != 1 ||
      header[3] != SEQ_PATTERN_COUNT || header[4] != SEQ_STEP_COUNT) {
    Serial.println("[SEQ] incompatible header");
    f.close();
    return false;
  }
  for (int p = 0; p < SEQ_PATTERN_COUNT; p++) {
    f.read((uint8_t*)seqPatterns[p], SEQ_STEP_COUNT);
  }
  f.close();
  Serial.println("[SEQ] Loaded patterns");
  return true;
}

// =================================================================
//  Device configuration (/config.json on SD)
// =================================================================
// Mirrors the M5Core2-MIDIXposeFilBT schema 1:1 so a single config.json works
// across both devices. Keys are all optional — missing keys fall back to the
// built-in defaults set by setDefaultConfig().
//   DefaultApp:           Play|SMF|MP3|Transpose|Filter|Change
//   DefaultTransposeMode: DIRECT|KEY|INSTANT|SEQUENCE   (used when DefaultApp=Transpose)
//   InitialTranspose:     int (offset from TransposeBase)
//   TransposeBase:        int (base reference for all transpose submodes)
//   InitialAllNotesOff / InitialFilterBypass / InitialMapperBypass: bool
//   TransposeRange:       "0..11" | "-11..0" | "-5..6"
//   MidiInputSource:      USB|MIDIIN|MIX        (Tab5 actually applies this)
//   MajorUpperTranspose:  bool                  (KEY-mode behaviour)
//   ShowSplash:           bool
//
// SD未挿入 / config.json なし / パース失敗 はすべて「default 動作と同じ」として
// 静かに継続する。ブートを止めない。
static void setDefaultConfig() {
  strncpy(g_config.defaultApp, "Transpose", sizeof(g_config.defaultApp));
  strncpy(g_config.defaultTransposeMode, "DIRECT", sizeof(g_config.defaultTransposeMode));
  g_config.initialTranspose = 0;
  g_config.transposeBase = 0;
  g_config.initialAllNotesOff = false;
  g_config.initialFilterBypass = true;
  g_config.initialMapperBypass = true;
  strncpy(g_config.transposeRange, "-5..6", sizeof(g_config.transposeRange));
  strncpy(g_config.midiInputSource, "MIX", sizeof(g_config.midiInputSource));
  g_config.majorUpperTranspose = false;
  g_config.showSplash = true;
  g_config.startupGSReset   = false;
  g_config.smfStartGSReset  = true;
  g_config.srcInitChannel   = 1;
  g_config.srcInitProgram   = 0;
  g_config.srcInitVolume    = 100;
  g_config.srcAutoChannel   = true;
}

static bool loadDeviceConfigFromSD() {
  if (!ensureSD()) {
    Serial.println("[CFG] no SD — using defaults");
    return false;
  }
  if (!SD.exists("/config.json")) {
    Serial.println("[CFG] /config.json not found — using defaults");
    return false;
  }
  File f = SD.open("/config.json", FILE_READ);
  if (!f) {
    Serial.println("[CFG] open failed — using defaults");
    return false;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    Serial.printf("[CFG] parse error %s — using defaults\n", err.c_str());
    return false;
  }
  if (doc["DefaultApp"].is<const char*>())           strncpy(g_config.defaultApp, doc["DefaultApp"], sizeof(g_config.defaultApp));
  if (doc["DefaultTransposeMode"].is<const char*>()) strncpy(g_config.defaultTransposeMode, doc["DefaultTransposeMode"], sizeof(g_config.defaultTransposeMode));
  if (doc["InitialTranspose"].is<int>())             g_config.initialTranspose = doc["InitialTranspose"];
  if (doc["TransposeBase"].is<int>())                g_config.transposeBase = doc["TransposeBase"];
  if (doc["InitialAllNotesOff"].is<bool>())          g_config.initialAllNotesOff = doc["InitialAllNotesOff"];
  if (doc["InitialFilterBypass"].is<bool>())         g_config.initialFilterBypass = doc["InitialFilterBypass"];
  if (doc["InitialMapperBypass"].is<bool>())         g_config.initialMapperBypass = doc["InitialMapperBypass"];
  if (doc["TransposeRange"].is<const char*>())       strncpy(g_config.transposeRange, doc["TransposeRange"], sizeof(g_config.transposeRange));
  if (doc["MidiInputSource"].is<const char*>())      strncpy(g_config.midiInputSource, doc["MidiInputSource"], sizeof(g_config.midiInputSource));
  if (doc["MajorUpperTranspose"].is<bool>())         g_config.majorUpperTranspose = doc["MajorUpperTranspose"];
  if (doc["ShowSplash"].is<bool>())                  g_config.showSplash = doc["ShowSplash"];
  if (doc["StartupGSReset"].is<bool>())              g_config.startupGSReset = doc["StartupGSReset"];
  if (doc["SmfStartGSReset"].is<bool>())             g_config.smfStartGSReset = doc["SmfStartGSReset"];
  if (doc["SrcInitChannel"].is<int>())               g_config.srcInitChannel = (uint8_t)constrain((int)doc["SrcInitChannel"], 1, 16);
  if (doc["SrcInitProgram"].is<int>())               g_config.srcInitProgram = (uint8_t)constrain((int)doc["SrcInitProgram"], 0, 127);
  if (doc["SrcInitVolume"].is<int>())                g_config.srcInitVolume  = (uint8_t)constrain((int)doc["SrcInitVolume"], 0, 127);
  if (doc["SrcAutoChannel"].is<bool>())              g_config.srcAutoChannel = doc["SrcAutoChannel"];
  Serial.printf("[CFG] loaded: app=%s base=%d input=%s splash=%d gsBoot=%d srcCh=%d\n",
                g_config.defaultApp, g_config.transposeBase,
                g_config.midiInputSource, g_config.showSplash ? 1 : 0,
                g_config.startupGSReset ? 1 : 0, g_config.srcInitChannel);
  return true;
}

static bool saveDeviceConfigToSD() {
  if (!ensureSD()) {
    Serial.println("[CFG] save failed: SD not ready");
    return false;
  }
  JsonDocument doc;
  doc["DefaultApp"]            = g_config.defaultApp;
  doc["DefaultTransposeMode"]  = g_config.defaultTransposeMode;
  doc["InitialTranspose"]      = g_config.initialTranspose;
  doc["TransposeBase"]         = g_config.transposeBase;
  doc["InitialAllNotesOff"]    = g_config.initialAllNotesOff;
  doc["InitialFilterBypass"]   = g_config.initialFilterBypass;
  doc["InitialMapperBypass"]   = g_config.initialMapperBypass;
  doc["TransposeRange"]        = g_config.transposeRange;
  doc["MidiInputSource"]       = g_config.midiInputSource;
  doc["MajorUpperTranspose"]   = g_config.majorUpperTranspose;
  doc["ShowSplash"]            = g_config.showSplash;
  doc["StartupGSReset"]        = g_config.startupGSReset;
  doc["SmfStartGSReset"]       = g_config.smfStartGSReset;
  doc["SrcInitChannel"]        = g_config.srcInitChannel;
  doc["SrcInitProgram"]        = g_config.srcInitProgram;
  doc["SrcInitVolume"]         = g_config.srcInitVolume;
  doc["SrcAutoChannel"]        = g_config.srcAutoChannel;
  // FILE_WRITE on SD.h appends; remove first so the file is rewritten cleanly.
  if (SD.exists("/config.json")) SD.remove("/config.json");
  File f = SD.open("/config.json", FILE_WRITE);
  if (!f) { Serial.println("[CFG] save: open failed"); return false; }
  size_t n = serializeJsonPretty(doc, f);
  f.close();
  Serial.printf("[CFG] saved %u bytes\n", (unsigned)n);
  return n > 0;
}

static void applyDeviceConfig() {
  allNotesOffEnabled = g_config.initialAllNotesOff;
  midiFilterBypass   = g_config.initialFilterBypass;
  midiMapperBypass   = g_config.initialMapperBypass;
  majorUpperTranspose = g_config.majorUpperTranspose;
  // Tab5 actually consumes MidiInputSource (Core2 only accepts/ignores it).
  if      (strcmp(g_config.midiInputSource, "USB")    == 0) setMidiInputSource(MIDI_INPUT_USB);
  else if (strcmp(g_config.midiInputSource, "MIDIIN") == 0) setMidiInputSource(MIDI_INPUT_UNIT);
  else                                                       setMidiInputSource(MIDI_INPUT_MIX);
  if      (strcmp(g_config.transposeRange, "0..11") == 0)   transposeRange = RANGE_0_TO_12;
  else if (strcmp(g_config.transposeRange, "-11..0") == 0)  transposeRange = RANGE_MINUS12_TO_0;
  else                                                       transposeRange = RANGE_MINUS5_TO_6;
  // initial transpose = base + offset
  handleTransposeChange(clampTranspose((int8_t)(g_config.transposeBase + g_config.initialTranspose)));
  updateDirectButtonLabels();

  // SRC defaults from config (active channel / program / volume / auto-follow).
  srcApplyConfigDefaults();

  // Switch to default app / submode.
  if (strcmp(g_config.defaultApp, "Play") == 0
      || strcmp(g_config.defaultApp, "SRC") == 0
      || strcmp(g_config.defaultApp, "SMF") == 0
      || strcmp(g_config.defaultApp, "MP3") == 0) {
    if      (strcmp(g_config.defaultApp, "MP3") == 0) currentPlay = PLAY_MP3;
    else if (strcmp(g_config.defaultApp, "SMF") == 0) currentPlay = PLAY_SMF;
    else                                              currentPlay = PLAY_SRC; // "Play" / "SRC"
    setCurrentApp(APP_PLAY);
  } else if (strcmp(g_config.defaultApp, "Filter") == 0) {
    midiManagePage = MIDI_PAGE_FILTER;
    setCurrentApp(APP_MIDI);
  } else if (strcmp(g_config.defaultApp, "Change") == 0) {
    midiManagePage = MIDI_PAGE_MAPPER;
    setCurrentApp(APP_MIDI);
  } else { // Transpose default
    if      (strcmp(g_config.defaultTransposeMode, "KEY")      == 0) currentMode = KEY_MODE;
    else if (strcmp(g_config.defaultTransposeMode, "INSTANT")  == 0) currentMode = INSTANT_MODE;
    else if (strcmp(g_config.defaultTransposeMode, "SEQUENCE") == 0) currentMode = SEQUENCE_MODE;
    else                                                              currentMode = DIRECT_MODE;
    setCurrentApp(APP_TRANSPOSE);
    if (currentMode == DIRECT_MODE) setCurrentTransposeButton();
  }
  needFullRedraw = true;
}

// =================================================================
//  CONFIG_EDIT_MODE — entered via header `CONF` button (or `MODE CONFIG`)
// =================================================================
// 12 fields × 3 pages of 4. Footer SAVE / CANCEL / APPLY work the same way as
// the M5Core2 reference. UI is scaled for the 1280×720 Tab5 panel:
//   row height 80 px, label/value font FONT_MED, footer 4 buttons (incl.
//   page L/R) each 220×80 along the bottom of the content area.
static DeviceConfig g_configSnapshot;
static AppMode      g_appBeforeOverlay  = APP_TRANSPOSE;
static DisplayMode  g_modeBeforeOverlay = DIRECT_MODE;
static PlayMode     g_playBeforeOverlay = PLAY_SRC;

static const int CFG_ROWS = 17;
static const int CFG_ROWS_PER_PAGE = 4;
static const int CFG_PAGES = 5;       // 17 fields / 4 per page = 5 (last page partially used)
static int g_configPage = 0;
static const char* CFG_LABELS[CFG_ROWS] = {
  "DefaultApp",
  "DefTransMode",
  "InitTranspose",
  "TransposeBase",
  "AllNotesOff",
  "FilterBypass",
  "MapperBypass",
  "TransposeRange",
  "MidiInputSrc",
  "MajorUpperTr",
  "ShowSplash",
  "StartupGSRst",
  "SmfStartGSRst",
  "SrcInitCh",
  "SrcInitPrg",
  "SrcInitVol",
  "SrcAutoCh",
};

static void cfgCycleString(char* dst, size_t cap, const char* const* options, int n) {
  int idx = 0;
  for (int i = 0; i < n; i++) if (strcmp(dst, options[i]) == 0) { idx = i; break; }
  idx = (idx + 1) % n;
  strncpy(dst, options[idx], cap);
  dst[cap - 1] = '\0';
}

static void cfgFormatValue(int row, char* out, size_t outSize) {
  switch (row) {
    case 0:  snprintf(out, outSize, "%s", g_config.defaultApp); break;
    case 1:  snprintf(out, outSize, "%s", g_config.defaultTransposeMode); break;
    case 2:  snprintf(out, outSize, "%+d", g_config.initialTranspose); break;
    case 3:  snprintf(out, outSize, "%+d", g_config.transposeBase); break;
    case 4:  snprintf(out, outSize, "%s", g_config.initialAllNotesOff ? "ON" : "OFF"); break;
    case 5:  snprintf(out, outSize, "%s", g_config.initialFilterBypass ? "ON" : "OFF"); break;
    case 6:  snprintf(out, outSize, "%s", g_config.initialMapperBypass ? "ON" : "OFF"); break;
    case 7:  snprintf(out, outSize, "%s", g_config.transposeRange); break;
    case 8:  snprintf(out, outSize, "%s", g_config.midiInputSource); break;
    case 9:  snprintf(out, outSize, "%s", g_config.majorUpperTranspose ? "ON" : "OFF"); break;
    case 10: snprintf(out, outSize, "%s", g_config.showSplash ? "ON" : "OFF"); break;
    case 11: snprintf(out, outSize, "%s", g_config.startupGSReset  ? "ON" : "OFF"); break;
    case 12: snprintf(out, outSize, "%s", g_config.smfStartGSReset ? "ON" : "OFF"); break;
    case 13: snprintf(out, outSize, "%u", (unsigned)g_config.srcInitChannel); break;
    case 14: snprintf(out, outSize, "%u", (unsigned)g_config.srcInitProgram); break;
    case 15: snprintf(out, outSize, "%u", (unsigned)g_config.srcInitVolume); break;
    case 16: snprintf(out, outSize, "%s", g_config.srcAutoChannel  ? "ON" : "OFF"); break;
    default: out[0] = '\0';
  }
}

static void cfgCycleValue(int row) {
  static const char* APPS[]   = {"Transpose","Play","SRC","SMF","MP3","Filter","Change"};
  static const char* TMODES[] = {"DIRECT","KEY","INSTANT","SEQUENCE"};
  static const char* RANGES[] = {"-5..6","0..11","-11..0"};
  static const char* INPUTS[] = {"MIX","MIDIIN","USB"};
  switch (row) {
    case 0:  cfgCycleString(g_config.defaultApp, sizeof(g_config.defaultApp), APPS, 7); break;
    case 1:  cfgCycleString(g_config.defaultTransposeMode, sizeof(g_config.defaultTransposeMode), TMODES, 4); break;
    case 2:  g_config.initialTranspose = (g_config.initialTranspose >= 11) ? -11 : g_config.initialTranspose + 1; break;
    case 3:  g_config.transposeBase    = (g_config.transposeBase    >= 11) ? -11 : g_config.transposeBase + 1; break;
    case 4:  g_config.initialAllNotesOff = !g_config.initialAllNotesOff; break;
    case 5:  g_config.initialFilterBypass = !g_config.initialFilterBypass; break;
    case 6:  g_config.initialMapperBypass = !g_config.initialMapperBypass; break;
    case 7:  cfgCycleString(g_config.transposeRange, sizeof(g_config.transposeRange), RANGES, 3); break;
    case 8:  cfgCycleString(g_config.midiInputSource, sizeof(g_config.midiInputSource), INPUTS, 3); break;
    case 9:  g_config.majorUpperTranspose = !g_config.majorUpperTranspose; break;
    case 10: g_config.showSplash = !g_config.showSplash; break;
    case 11: g_config.startupGSReset  = !g_config.startupGSReset; break;
    case 12: g_config.smfStartGSReset = !g_config.smfStartGSReset; break;
    case 13: g_config.srcInitChannel  = (g_config.srcInitChannel >= 16) ? 1 : (uint8_t)(g_config.srcInitChannel + 1); break;
    case 14: g_config.srcInitProgram  = (g_config.srcInitProgram >= 127) ? 0 : (uint8_t)(g_config.srcInitProgram + 1); break;
    case 15: g_config.srcInitVolume   = (g_config.srcInitVolume  >= 127) ? 0 : (uint8_t)(g_config.srcInitVolume  + 1); break;
    case 16: g_config.srcAutoChannel  = !g_config.srcAutoChannel; break;
  }
}

// Layout helpers for the overlay. Use the FULL content+nav strip below the
// toolbar (y=contentArea.y .. SCREEN_H) so nothing is wasted on a 1280×720
// panel. Header (BT/AOFF/MIDI input selector) stays visible and tappable.
//
// Vertical zones (within the content+nav strip):
//   contentArea.y .. contentArea.y+44     title strip (FONT_LARGE)
//   +50 .. navArea.y-10                   field rows (4 rows × ~70 px + gaps)
//   navArea.y+10 .. SCREEN_H-10           footer buttons (80 px tall)
static void cfgGetRowRect(int rowOnPage, Rect& r) {
  const int rowH   = 72;
  const int rowGap = 12;
  const int marginX = 40;
  int yTop = contentArea.y + 56;       // below the title strip
  r = { marginX, yTop + rowOnPage * (rowH + rowGap),
        SCREEN_W - 2 * marginX, rowH };
}

static void cfgGetFooterRects(Rect out[5]) {
  // [0]=PAGE-, [1]=PAGE+, [2]=SAVE, [3]=CANCEL, [4]=APPLY — placed in the
  // navArea so they don't crowd the field rows.
  const int btnH = 80;
  const int gap = 16;
  const int btnW = 220;
  int y = navArea.y + (navArea.h - btnH) / 2;
  int totalW = 5 * btnW + 4 * gap;
  int x0 = (SCREEN_W - totalW) / 2;
  for (int i = 0; i < 5; i++) {
    out[i] = { x0 + i * (btnW + gap), y, btnW, btnH };
  }
}

static void enterConfigEditMode() {
  g_appBeforeOverlay  = currentApp;
  g_modeBeforeOverlay = currentMode;
  g_playBeforeOverlay = currentPlay;
  g_configSnapshot = g_config;
  g_configPage = 0;
  currentMode = CONFIG_EDIT_MODE;
  needFullRedraw = true;
  Serial.println("[CFG] enter CONFIG_EDIT");
}

static void exitConfigEditMode(bool save, bool apply) {
  if (!save && !apply) {
    g_config = g_configSnapshot;  // CANCEL — revert
  } else {
    saveDeviceConfigToSD();
    if (apply) {
      // applyDeviceConfig() jumps to the configured app/mode. It also calls
      // setCurrentApp() which clears overlay state implicitly.
      currentMode = g_modeBeforeOverlay;  // make sure currentMode is sane before apply
      currentApp  = g_appBeforeOverlay;
      currentPlay = g_playBeforeOverlay;
      applyDeviceConfig();
      Serial.println("[CFG] APPLY done");
      return;
    }
  }
  currentMode = g_modeBeforeOverlay;
  currentApp  = g_appBeforeOverlay;
  currentPlay = g_playBeforeOverlay;
  needFullRedraw = true;
  Serial.println(save ? "[CFG] SAVE done" : "[CFG] CANCEL done");
}

static void drawConfigEditMode() {
  // Content + nav area background — but keep the header & toolbar visible so
  // the user can still see BT / AOFF status while editing.
  M5.Display.fillRect(0, contentArea.y, SCREEN_W,
                      SCREEN_H - contentArea.y, COL_BG);

  // Title strip across the top of the content area.
  M5.Display.setFont(FONT_TITLE);
  M5.Display.setTextColor(COL_VALUE, COL_BG);
  M5.Display.setTextDatum(top_center);
  char title[48];
  snprintf(title, sizeof(title), "CONFIG  (page %d/%d)", g_configPage + 1, CFG_PAGES);
  M5.Display.drawString(title, SCREEN_W / 2, contentArea.y + 8);

  // Field rows.
  M5.Display.setFont(FONT_MED);
  for (int rOnPage = 0; rOnPage < CFG_ROWS_PER_PAGE; rOnPage++) {
    int field = g_configPage * CFG_ROWS_PER_PAGE + rOnPage;
    if (field >= CFG_ROWS) break;
    Rect rr; cfgGetRowRect(rOnPage, rr);
    M5.Display.fillRoundRect(rr.x, rr.y, rr.w, rr.h, 12, COL_PANEL);
    M5.Display.drawRoundRect(rr.x, rr.y, rr.w, rr.h, 12, COL_BTN_BDR);

    M5.Display.setTextColor(COL_BTN_TXT, COL_PANEL);
    M5.Display.setTextDatum(middle_left);
    M5.Display.drawString(CFG_LABELS[field], rr.x + 24, rr.y + rr.h / 2);

    char val[24];
    cfgFormatValue(field, val, sizeof(val));
    M5.Display.setTextColor(COL_ACCENT, COL_PANEL);
    M5.Display.setTextDatum(middle_right);
    M5.Display.drawString(val, rr.x + rr.w - 24, rr.y + rr.h / 2);
  }

  // Footer buttons: < PAGE | PAGE > | SAVE | CANCEL | APPLY.
  Rect ft[5]; cfgGetFooterRects(ft);
  drawRectBtn(ft[0], COL_BTN,    COL_BTN_BDR, "< PAGE",  COL_BTN_TXT, FONT_MED);
  drawRectBtn(ft[1], COL_BTN,    COL_BTN_BDR, "PAGE >",  COL_BTN_TXT, FONT_MED);
  drawRectBtn(ft[2], TFT_DARKGREEN, COL_BTN_BDR, "SAVE",    COL_BTN_TXT, FONT_MED);
  drawRectBtn(ft[3], COL_DANGER, COL_BTN_BDR, "CANCEL",  COL_BTN_TXT, FONT_MED);
  drawRectBtn(ft[4], TFT_BLUE,   COL_BTN_BDR, "APPLY",   COL_BTN_TXT, FONT_MED);
}

static void processConfigEditTouch(int tx, int ty) {
  Rect ft[5]; cfgGetFooterRects(ft);
  if (hit(ft[0], tx, ty)) { g_configPage = (g_configPage + CFG_PAGES - 1) % CFG_PAGES; needFullRedraw = true; return; }
  if (hit(ft[1], tx, ty)) { g_configPage = (g_configPage + 1) % CFG_PAGES; needFullRedraw = true; return; }
  if (hit(ft[2], tx, ty)) { exitConfigEditMode(true, false); return; }
  if (hit(ft[3], tx, ty)) { exitConfigEditMode(false, false); return; }
  if (hit(ft[4], tx, ty)) { exitConfigEditMode(true, true);  return; }

  for (int rOnPage = 0; rOnPage < CFG_ROWS_PER_PAGE; rOnPage++) {
    int field = g_configPage * CFG_ROWS_PER_PAGE + rOnPage;
    if (field >= CFG_ROWS) break;
    Rect rr; cfgGetRowRect(rOnPage, rr);
    if (hit(rr, tx, ty)) {
      cfgCycleValue(field);
      needFullRedraw = true;
      return;
    }
  }
}

// =================================================================
//  BASE_SET_MODE — long-tap on AOFF (or `MODE BASE`)
// =================================================================
// Picks the transpose base reference. Three pages mirror the FilBTUM picker:
//   page L: -12..-1   (entered by tapping -5 on M)
//   page M: -5..+6    (default entry)
//   page R: +1..+12   (entered by tapping +6 on M)
// Tapping -1 on L or +1 on R returns to M (per spec).
// On-screen PAGE / EXIT buttons replace the C / A-long buttons of FilBTUM.
static int g_basePage = 0;          // -1=L, 0=M, +1=R

static void getBasePageRange(int page, int* lo, int* hi) {
  if (page == -1)     { *lo = -12; *hi = -1; }
  else if (page == 1) { *lo =   1; *hi = 12; }
  else                { *lo =  -5; *hi =  6; }
}

static int basePageValueAt(int page, int idx) {
  int lo, hi; getBasePageRange(page, &lo, &hi);
  return lo + idx;
}

static int pageContainingBase(int base) {
  if (base < -5)     return -1;
  else if (base > 6) return 1;
  else                return 0;
}

static void setTransposeBase(int newBase) {
  newBase = clampTranspose((int8_t)newBase);
  int delta = newBase - g_config.transposeBase;
  g_config.transposeBase = newBase;
  if (delta != 0) {
    // Apply immediately — shift the current effective transpose by delta so
    // any user-selected offset in DIRECT/KEY/INSTANT/SEQUENCE follows the
    // new base.
    handleTransposeChange(clampTranspose((int8_t)(transposeValue + delta)));
  }
  needFullRedraw = true;
}

static void enterBaseSetMode() {
  g_appBeforeOverlay  = currentApp;
  g_modeBeforeOverlay = currentMode;
  g_playBeforeOverlay = currentPlay;
  g_basePage = pageContainingBase(g_config.transposeBase);
  currentMode = BASE_SET_MODE;
  needFullRedraw = true;
  Serial.println("[BASE] enter BASE_SET");
}

static void exitBaseSetMode() {
  currentMode = g_modeBeforeOverlay;
  currentApp  = g_appBeforeOverlay;
  currentPlay = g_playBeforeOverlay;
  needFullRedraw = true;
  Serial.println("[BASE] exit");
}

static void cycleBaseSetPage() {
  if      (g_basePage == -1) g_basePage = 0;
  else if (g_basePage ==  0) g_basePage = 1;
  else                        g_basePage = -1;
  needFullRedraw = true;
}

static void baseGetGridRect(int idx, Rect& r) {
  // 4×3 grid mirroring DIRECT mode style. Tab5 has a 1280×~424 px content
  // area plus a 100 px nav area below — use the full content strip for
  // the grid and put PAGE/EXIT buttons in the navArea.
  const int colsPerRow = 4;
  const int gapX = 14;
  const int gapY = 10;
  const int marginX = 30;
  // Grid spans contentArea.y+56 (below the title strip) down to navArea.y-10.
  int gridY0     = contentArea.y + 56;
  int gridYBot   = navArea.y - 10;
  int gridH      = gridYBot - gridY0;
  int btnH       = (gridH - 2 * gapY) / 3;
  int btnW       = (SCREEN_W - 2 * marginX - 3 * gapX) / colsPerRow;
  int col = idx % colsPerRow;
  int row = idx / colsPerRow;
  r = { marginX + col * (btnW + gapX),
        gridY0  + row * (btnH + gapY),
        btnW, btnH };
}

static void baseGetFooterRects(Rect out[2]) {
  // [0]=PAGE, [1]=EXIT — placed in the nav area so they sit clear of the grid.
  const int btnH = 80;
  const int gap = 24;
  const int btnW = 280;
  int y = navArea.y + (navArea.h - btnH) / 2;
  int totalW = 2 * btnW + gap;
  int x0 = (SCREEN_W - totalW) / 2;
  for (int i = 0; i < 2; i++) out[i] = { x0 + i * (btnW + gap), y, btnW, btnH };
}

static void drawBaseSetMode() {
  M5.Display.fillRect(0, contentArea.y, SCREEN_W, SCREEN_H - contentArea.y, COL_BG);

  M5.Display.setFont(FONT_TITLE);
  M5.Display.setTextColor(COL_VALUE, COL_BG);
  M5.Display.setTextDatum(top_center);
  const char* pgLabel = (g_basePage == -1) ? "L (-12..-1)"
                       : (g_basePage == 1) ? "R (+1..+12)"
                                            : "M (-5..+6)";
  char title[64];
  snprintf(title, sizeof(title), "BASE = %+d   Page %s",
           g_config.transposeBase, pgLabel);
  M5.Display.drawString(title, SCREEN_W / 2, contentArea.y + 8);

  // Match XPOSE/DIRECT button sizing (FONT_HUGE). Digits stay white because
  // the yellow "BASE = ..." title above already signals which screen this is.
  M5.Display.setFont(FONT_HUGE);
  for (int i = 0; i < 12; i++) {
    int val = basePageValueAt(g_basePage, i);
    bool selected = (val == g_config.transposeBase);
    Rect rr; baseGetGridRect(i, rr);
    uint16_t bg  = selected ? COL_BTN_HI : COL_BTN;
    uint16_t txt = selected ? COL_BTN_TXT_HI : COL_BTN_TXT;
    char vs[8];
    if (val > 0) snprintf(vs, sizeof(vs), "+%d", val);
    else         snprintf(vs, sizeof(vs), "%d",  val);
    drawRectBtn(rr, bg, COL_BTN_BDR, vs, txt, FONT_HUGE);
  }

  Rect ft[2]; baseGetFooterRects(ft);
  drawRectBtn(ft[0], TFT_BLUE,   COL_BTN_BDR, "PAGE",  COL_BTN_TXT, FONT_MED);
  drawRectBtn(ft[1], COL_DANGER, COL_BTN_BDR, "EXIT",  COL_BTN_TXT, FONT_MED);
}

static void processBaseSetTouch(int tx, int ty) {
  Rect ft[2]; baseGetFooterRects(ft);
  if (hit(ft[0], tx, ty)) { cycleBaseSetPage(); return; }
  if (hit(ft[1], tx, ty)) { exitBaseSetMode();  return; }

  for (int i = 0; i < 12; i++) {
    Rect rr; baseGetGridRect(i, rr);
    if (hit(rr, tx, ty)) {
      int val = basePageValueAt(g_basePage, i);
      setTransposeBase(val);
      // Page transitions per spec: M→R via +6, M→L via -5, R→M via +1, L→M via -1.
      if (g_basePage == 0) {
        if (val == 6)       g_basePage = 1;
        else if (val == -5) g_basePage = -1;
      } else if (g_basePage == 1) {
        if (val == 1) g_basePage = 0;
      } else { // L
        if (val == -1) g_basePage = 0;
      }
      needFullRedraw = true;
      return;
    }
  }
}

// =================================================================
//  Arduino entry points
// =================================================================
// M5GFX font loading uses a lot of stack on P4; bump the loop-task stack.
size_t getArduinoLoopTaskStackSize(void) { return 32 * 1024; }

// ==== USB Serial command interface (port from M5Core2) ====
#define USB_COMMAND_BUFFER_SIZE 192
static char  g_usbCmdBuf[USB_COMMAND_BUFFER_SIZE];
static size_t g_usbCmdLen = 0;

static const char* getDisplayModeLabel(DisplayMode mode) {
  switch (mode) {
    case DIRECT_MODE:      return "DIRECT";
    case KEY_MODE:         return "KEY";
    case INSTANT_MODE:     return "INSTANT";
    case SEQUENCE_MODE:    return "SEQUENCE";
    default:               return "UNKNOWN";
  }
}

static const char* getAppLabel(AppMode app) {
  switch (app) {
    case APP_TRANSPOSE: return "XPOSE";
    case APP_MIDI:      return "MIDI";
    case APP_PLAY:
      if (currentPlay == PLAY_SRC) return "PLAY:SRC";
      if (currentPlay == PLAY_SMF) return "PLAY:SMF";
      return "PLAY:MP3";
    default:            return "UNKNOWN";
  }
}

static bool tokenEqualsIgnoreCase(const char* lhs, const char* rhs) {
  if (lhs == nullptr || rhs == nullptr) return false;
  while (*lhs && *rhs) {
    if (toupper((unsigned char)*lhs) != toupper((unsigned char)*rhs)) return false;
    lhs++; rhs++;
  }
  return *lhs == '\0' && *rhs == '\0';
}

static bool parseIntValue(const char* token, int& outValue) {
  if (!token || !*token) return false;
  char* endPtr = nullptr;
  long v = strtol(token, &endPtr, 10);
  if (endPtr == token || *endPtr != '\0') return false;
  outValue = (int)v;
  return true;
}

static int8_t clampTransposeI(int v) {
  if (v < -12) return -12;
  if (v > 12)  return  12;
  return (int8_t)v;
}

static bool setModeFromCommand(const char* mode) {
  if (tokenEqualsIgnoreCase(mode, "DIRECT"))   { setCurrentApp(APP_TRANSPOSE); currentMode = DIRECT_MODE;   needFullRedraw = true; return true; }
  if (tokenEqualsIgnoreCase(mode, "KEY"))      { setCurrentApp(APP_TRANSPOSE); currentMode = KEY_MODE;      needFullRedraw = true; return true; }
  if (tokenEqualsIgnoreCase(mode, "INSTANT"))  { setCurrentApp(APP_TRANSPOSE); currentMode = INSTANT_MODE;  needFullRedraw = true; return true; }
  if (tokenEqualsIgnoreCase(mode, "SEQUENCE") ||
      tokenEqualsIgnoreCase(mode, "SEQ"))      { setCurrentApp(APP_TRANSPOSE); currentMode = SEQUENCE_MODE; needFullRedraw = true; return true; }
  if (tokenEqualsIgnoreCase(mode, "FILTER")) {
    setCurrentApp(APP_MIDI); midiManagePage = MIDI_PAGE_FILTER; needFullRedraw = true; return true;
  }
  if (tokenEqualsIgnoreCase(mode, "MAPPER")) {
    setCurrentApp(APP_MIDI); midiManagePage = MIDI_PAGE_MAPPER; needFullRedraw = true; return true;
  }
  if (tokenEqualsIgnoreCase(mode, "MIDI") ||
      tokenEqualsIgnoreCase(mode, "MANAGER") ||
      tokenEqualsIgnoreCase(mode, "MIDI_MANAGER")) {
    setCurrentApp(APP_MIDI); needFullRedraw = true; return true;
  }
  if (tokenEqualsIgnoreCase(mode, "SRC"))      { setCurrentApp(APP_PLAY); currentPlay = PLAY_SRC; needFullRedraw = true; return true; }
  if (tokenEqualsIgnoreCase(mode, "SMF"))      { setCurrentApp(APP_PLAY); currentPlay = PLAY_SMF; needFullRedraw = true; return true; }
  if (tokenEqualsIgnoreCase(mode, "MP3"))      { setCurrentApp(APP_PLAY); currentPlay = PLAY_MP3; needFullRedraw = true; return true; }
  if (tokenEqualsIgnoreCase(mode, "CONFIG") ||
      tokenEqualsIgnoreCase(mode, "CONFIG_EDIT")) { enterConfigEditMode(); return true; }
  if (tokenEqualsIgnoreCase(mode, "BASE") ||
      tokenEqualsIgnoreCase(mode, "BASE_SET"))    { enterBaseSetMode();  return true; }
  return false;
}

static bool setGroupFromCommand(const char* group) {
  if (tokenEqualsIgnoreCase(group, "TRANSPOSE")) {
    setCurrentApp(APP_TRANSPOSE);
    needFullRedraw = true;
    return true;
  }
  if (tokenEqualsIgnoreCase(group, "MIDI") ||
      tokenEqualsIgnoreCase(group, "MANAGER") ||
      tokenEqualsIgnoreCase(group, "MIDI_MANAGER")) {
    setCurrentApp(APP_MIDI);
    needFullRedraw = true;
    return true;
  }
  return false;
}

// 物理ボタンが無いので、A=AllOffトグル、B=現モードに応じた副作用、
// C短押し=サブモード巡回、C長押し=Group切替 として疑似実装。
static void handleButtonAAction() {
  allNotesOffEnabled = !allNotesOffEnabled;
  if (allNotesOffEnabled) sendAllNotesOff();
  needPartialUpdate = true;
}

static void handleButtonBAction() {
  if (currentMode == DIRECT_MODE) {
    transposeRange = (TransposeRange)(((int)transposeRange + 1) % 3);
    needFullRedraw = true;
  } else if (currentMode == KEY_MODE) {
    majorUpperTranspose = !majorUpperTranspose;
    minorUpperTranspose = !minorUpperTranspose;
    selectedMajorKey = -1; selectedMinorKey = -1;
    needFullRedraw = true;
  } else if (currentApp == APP_MIDI) {
    // Toggle MAPPER edit was here in M5Core2; on Tab5 the side-by-side editor needs no toggle.
    needFullRedraw = true;
  }
}

static void handleButtonCShortAction() {
  if (currentApp == APP_MIDI) {
    midiManagePage = (midiManagePage == MIDI_PAGE_FILTER) ? MIDI_PAGE_MAPPER : MIDI_PAGE_FILTER;
  } else if (currentApp == APP_TRANSPOSE) {
    int next = ((int)currentMode + 1) % 4;
    currentMode = (DisplayMode)next;
  } else {  // APP_PLAY
    currentPlay = (currentPlay == PLAY_SMF) ? PLAY_MP3 : PLAY_SMF;
  }
  needFullRedraw = true;
}

static void handleButtonCLongAction() {
  // Cycle through top apps: XPOSE → MIDI → PLAY → XPOSE
  int next = ((int)currentApp + 1) % 3;
  setCurrentApp((AppMode)next);
  needFullRedraw = true;
}

static void injectTouchPoint(int16_t x, int16_t y) {
  // 既存の handleTouch() は M5.Touch を読むので、シリアルからのタッチ注入は
  // 後段の UI 実装フェーズで対応する (現状は座標を Serial にエコーするのみ)。
  Serial.printf("# TOUCH inject (%d,%d) — UI handler pending\n", x, y);
  (void)x; (void)y;
}

static void printUsbSerialStatus() {
  const char* pageLabel       = (midiManagePage == MIDI_PAGE_FILTER) ? "FILTER" : "MAPPER";
  const char* mapperPageLabel = (midiMapperEditPage == MAPPER_PAGE_SOURCE) ? "PG1" : "PG2";
  const char* inputLabel      = getMidiInputSourceLabel();
  const char* usbLabel        = g_usbMidiMounted ? "connected" : "disconnected";
  Serial.printf(
    "OK STATUS app=%s mode=%s input=%s transpose=%d range=%d filter_bypass=%d mapper_bypass=%d "
    "filter_rule=%d/%d mapper_rule=%d/%d page=%s mapper_page=%s midi_in=%lu midi_out=%lu midi_in_real=%lu midi_out_real=%lu usb_in=%s cables=%u usb_drop=%lu usb_resub_fail=%lu\n",
    getAppLabel(currentApp),
    getDisplayModeLabel(currentMode),
    inputLabel,
    (int)transposeValue,
    (int)transposeRange,
    midiFilterBypass ? 1 : 0,
    midiMapperBypass ? 1 : 0,
    midiSelectedFilterRule + 1, midiFilterRuleCount,
    midiSelectedMapperRule + 1, midiMapperRuleCount,
    pageLabel, mapperPageLabel,
    midiInCount, midiOutCount,
    midiInRealCount, midiOutRealCount,
    usbLabel,
    (unsigned)g_usbMidiCableCount,
    (unsigned long)g_usbMidiRingDropCount,
    (unsigned long)g_usbInResubmitFails
  );
}

static void printUsbSerialHelp() {
  Serial.println("OK HELP BEGIN");
  Serial.println("HELP");
  Serial.println("STATUS");
  Serial.println("REDRAW");
  Serial.println("BUTTON A|B|C [LONG]");
  Serial.println("TOUCH <x> <y>");
  Serial.println("MODE DIRECT|KEY|INSTANT|SEQUENCE|FILTER|MAPPER|MIDI|SRC|SMF|MP3|CONFIG|BASE");
  Serial.println("GROUP TRANSPOSE|MIDI");
  Serial.println("SET TRANSPOSE <-12..12>");
  Serial.println("SET INPUT USBIN|MIDIIN|MIX");
  Serial.println("SET FILTER BYPASS [0|1] | ACTIVE | ENABLED <n> 0|1");
  Serial.println("SET MAPPER BYPASS [0|1] | ACTIVE | ENABLED <n> 0|1");
  Serial.println("LOAD TESTRULES");
  Serial.println("INFO SCREEN");
  Serial.println("OK HELP END");
}

static void handleUsbSerialCommand(char* line) {
  while (*line && isspace((unsigned char)*line)) line++;
  if (!*line) return;
  char* end = line + strlen(line) - 1;
  while (end >= line && isspace((unsigned char)*end)) { *end = '\0'; end--; }
  if (!*line) return;

  char* save = nullptr;
  char* cmd = strtok_r(line, " \t", &save);
  if (!cmd) return;

  if (tokenEqualsIgnoreCase(cmd, "HELP"))   { printUsbSerialHelp();   return; }
  if (tokenEqualsIgnoreCase(cmd, "STATUS")) { printUsbSerialStatus(); return; }
  if (tokenEqualsIgnoreCase(cmd, "REDRAW")) { needFullRedraw = true; Serial.println("OK REDRAW"); return; }

  if (tokenEqualsIgnoreCase(cmd, "INFO")) {
    char* sub = strtok_r(nullptr, " \t", &save);
    if (sub && tokenEqualsIgnoreCase(sub, "SCREEN")) {
      Serial.printf("OK SCREEN width=%d height=%d\n", (int)SCREEN_W, (int)SCREEN_H);
      return;
    }
    Serial.println("ERR INFO requires SCREEN");
    return;
  }

  if (tokenEqualsIgnoreCase(cmd, "BUTTON")) {
    char* btn = strtok_r(nullptr, " \t", &save);
    char* mod = strtok_r(nullptr, " \t", &save);
    if (!btn) { Serial.println("ERR BUTTON requires A, B, or C"); return; }
    if (tokenEqualsIgnoreCase(btn, "A")) { handleButtonAAction(); Serial.println("OK BUTTON A"); return; }
    if (tokenEqualsIgnoreCase(btn, "B")) { handleButtonBAction(); Serial.println("OK BUTTON B"); return; }
    if (tokenEqualsIgnoreCase(btn, "C")) {
      if (mod && tokenEqualsIgnoreCase(mod, "LONG")) { handleButtonCLongAction(); Serial.println("OK BUTTON C LONG"); }
      else { handleButtonCShortAction(); Serial.println("OK BUTTON C"); }
      return;
    }
    Serial.println("ERR BUTTON requires A, B, or C");
    return;
  }

  if (tokenEqualsIgnoreCase(cmd, "TOUCH")) {
    char* xs = strtok_r(nullptr, " \t", &save);
    char* ys = strtok_r(nullptr, " \t", &save);
    int x, y;
    if (!parseIntValue(xs, x) || !parseIntValue(ys, y)) { Serial.println("ERR TOUCH requires integer x and y"); return; }
    if (x < 0 || x >= SCREEN_W || y < 0 || y >= SCREEN_H) { Serial.println("ERR TOUCH out of range"); return; }
    injectTouchPoint((int16_t)x, (int16_t)y);
    Serial.printf("OK TOUCH %d %d\n", x, y);
    return;
  }

  if (tokenEqualsIgnoreCase(cmd, "MODE")) {
    char* m = strtok_r(nullptr, " \t", &save);
    if (!m)                 { Serial.println("ERR MODE requires a target mode"); return; }
    if (!setModeFromCommand(m)) { Serial.println("ERR MODE unknown target"); return; }
    Serial.printf("OK MODE %s\n", m);
    return;
  }

  if (tokenEqualsIgnoreCase(cmd, "GROUP")) {
    char* g = strtok_r(nullptr, " \t", &save);
    if (!g)                  { Serial.println("ERR GROUP requires TRANSPOSE or MIDI"); return; }
    if (!setGroupFromCommand(g)) { Serial.println("ERR GROUP unknown target"); return; }
    Serial.printf("OK GROUP %s\n", g);
    return;
  }

  if (tokenEqualsIgnoreCase(cmd, "SET")) {
    char* target = strtok_r(nullptr, " \t", &save);
    if (target && tokenEqualsIgnoreCase(target, "TRANSPOSE")) {
      char* vt = strtok_r(nullptr, " \t", &save);
      int nv;
      if (!parseIntValue(vt, nv)) { Serial.println("ERR SET TRANSPOSE requires an integer"); return; }
      handleTransposeChange(clampTransposeI(nv));
      needFullRedraw = true;
      Serial.printf("OK SET TRANSPOSE %d\n", (int)transposeValue);
      return;
    }
    if (target && tokenEqualsIgnoreCase(target, "INPUT")) {
      char* val = strtok_r(nullptr, " \t", &save);
      if (val && (tokenEqualsIgnoreCase(val, "USB") || tokenEqualsIgnoreCase(val, "USB-MIDI") || tokenEqualsIgnoreCase(val, "USBIN"))) {
        if (!setMidiInputSource(MIDI_INPUT_USB)) { Serial.println("ERR SET INPUT USBIN failed"); return; }
        Serial.println("OK SET INPUT USBIN");
        return;
      }
      if (val && (tokenEqualsIgnoreCase(val, "MIDIIF") || tokenEqualsIgnoreCase(val, "UNIT") || tokenEqualsIgnoreCase(val, "MIDI-IF") || tokenEqualsIgnoreCase(val, "MIDIIN"))) {
        if (!setMidiInputSource(MIDI_INPUT_UNIT)) { Serial.println("ERR SET INPUT MIDIIN failed"); return; }
        Serial.println("OK SET INPUT MIDIIN");
        return;
      }
      if (val && tokenEqualsIgnoreCase(val, "MIX")) {
        if (!setMidiInputSource(MIDI_INPUT_MIX)) { Serial.println("ERR SET INPUT MIX failed"); return; }
        Serial.println("OK SET INPUT MIX");
        return;
      }
      Serial.println("ERR SET INPUT requires USBIN, MIDIIN, or MIX");
      return;
    }
    if (target && (tokenEqualsIgnoreCase(target, "FILTER") || tokenEqualsIgnoreCase(target, "MAPPER"))) {
      const bool isFilter = tokenEqualsIgnoreCase(target, "FILTER");
      char* val = strtok_r(nullptr, " \t", &save);
      if (val && tokenEqualsIgnoreCase(val, "BYPASS")) {
        // Two forms: `BYPASS` (no arg) sets bypass=true (legacy), `BYPASS 0|1` sets explicit value.
        char* arg = strtok_r(nullptr, " \t", &save);
        int v;
        if (arg == nullptr) {
          if (isFilter) midiFilterBypass = true; else midiMapperBypass = true;
          needFullRedraw = true;
          Serial.printf("OK SET %s BYPASS\n", isFilter ? "FILTER" : "MAPPER");
          return;
        }
        if (parseIntValue(arg, v) && (v == 0 || v == 1)) {
          if (isFilter) midiFilterBypass = (v != 0); else midiMapperBypass = (v != 0);
          needFullRedraw = true;
          Serial.printf("OK SET %s BYPASS %d\n", isFilter ? "FILTER" : "MAPPER", v);
          return;
        }
        Serial.printf("ERR SET %s BYPASS [0|1]\n", isFilter ? "FILTER" : "MAPPER");
        return;
      }
      if (val && tokenEqualsIgnoreCase(val, "ACTIVE")) {
        if (isFilter) midiFilterBypass = false; else midiMapperBypass = false;
        needFullRedraw = true;
        Serial.printf("OK SET %s ACTIVE\n", isFilter ? "FILTER" : "MAPPER");
        return;
      }
      if (val && tokenEqualsIgnoreCase(val, "ENABLED")) {
        char* idxToken = strtok_r(nullptr, " \t", &save);
        char* valToken = strtok_r(nullptr, " \t", &save);
        int idx, v;
        const int ruleCount = isFilter ? midiFilterRuleCount : midiMapperRuleCount;
        if (!parseIntValue(idxToken, idx) || !parseIntValue(valToken, v)
            || idx < 1 || idx > ruleCount || (v != 0 && v != 1)) {
          Serial.printf("ERR SET %s ENABLED <1..%d> 0|1\n",
                        isFilter ? "FILTER" : "MAPPER", ruleCount);
          return;
        }
        if (isFilter) midiFilterRules[idx - 1].enabled = (v != 0);
        else          midiMapperRules[idx - 1].enabled = (v != 0);
        needFullRedraw = true;
        Serial.printf("OK SET %s ENABLED %d %d\n",
                      isFilter ? "FILTER" : "MAPPER", idx, v);
        return;
      }
      Serial.printf("ERR SET %s requires BYPASS, ACTIVE, or ENABLED\n", isFilter ? "FILTER" : "MAPPER");
      return;
    }
    Serial.println("ERR SET supports TRANSPOSE / INPUT / FILTER / MAPPER");
    return;
  }

  if (tokenEqualsIgnoreCase(cmd, "LOAD")) {
    char* target = strtok_r(nullptr, " \t", &save);
    if (target != nullptr && tokenEqualsIgnoreCase(target, "TESTRULES")) {
      loadTestRules();
      needFullRedraw = true;
      Serial.printf("OK LOAD TESTRULES filter=%d mapper=%d\n",
                    midiFilterRuleCount, midiMapperRuleCount);
      return;
    }
    Serial.println("ERR LOAD supports only TESTRULES");
    return;
  }

  Serial.println("ERR Unknown command");
}

static void processUsbSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      g_usbCmdBuf[g_usbCmdLen] = '\0';
      handleUsbSerialCommand(g_usbCmdBuf);
      g_usbCmdLen = 0;
      continue;
    }
    if (g_usbCmdLen + 1 < sizeof(g_usbCmdBuf)) {
      g_usbCmdBuf[g_usbCmdLen++] = c;
    } else {
      g_usbCmdLen = 0;  // overflow → drop
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  auto m5cfg = M5.config();
  m5cfg.output_power = true;
  m5cfg.internal_spk = true;
  m5cfg.internal_mic = false;
  M5.begin(m5cfg);

  M5.Display.setRotation(3);
  M5.Display.setBrightness(220);
  M5.Speaker.setVolume(mp3Volume);

  // Load /config.json before the splash so ShowSplash:false skips it cleanly.
  // Failures (no SD / no config.json / parse error) silently fall back to
  // built-in defaults so the boot never stalls.
  setDefaultConfig();
  ensureStorage();
  loadDeviceConfigFromSD();

  if (g_config.showSplash) {
    drawSplash();
  }

  // setRxBufferSize / setTxBufferSize must be called BEFORE begin() — the
  // arduino-esp32 implementation early-returns once the UART driver is
  // installed (see HardwareSerial.cpp::setTxBufferSize: `if (_uart) return 0;`).
  // The previous order silently no-op'd both calls, leaving Serial2 with the
  // default 256-byte RX ring and no TX ring at all (just the ~128-byte
  // hardware FIFO). At 31.25 kbaud that overflowed almost immediately when
  // a keyboard player drove sustained MIDI input, so every Serial2.write()
  // blocked waiting for the hardware FIFO to drain — starving the loop and
  // tripping the task watchdog. Generous TX/RX rings absorb realistic
  // bursts (a 4 KB TX ring buys ~125 ms of headroom).
  // Generous RX/TX rings: the dedicated MIDI task drains RX every ~1 ms but
  // a heavy chord burst (6 notes × 3 bytes = 18 bytes for NoteOn alone) can
  // arrive in <6 ms, and stacking with Active Sense / clock heartbeat is
  // common. 8 KB RX = ~2.6 s of headroom even if the task is briefly stalled.
  Serial2.setRxBufferSize(8192);
  Serial2.setTxBufferSize(4096);
  Serial2.begin(MIDI_BAUD, SERIAL_8N1, RXD2, TXD2);
  // Recursive mutex protects the Serial2 TX FIFO across the MIDI task on
  // core 0 and any UI-driven sends on core 1. Must exist before either
  // sendGSReset() below or the midi_io_task spin-up touches Serial2.
  g_serial2TxMux = xSemaphoreCreateRecursiveMutex();

  // Start USB host once at boot. Input-source changes only toggle whether
  // received data is consumed, never the USB host itself.
  if (startUsbHost()) {
    Serial.println("[USB] host ready");
  } else {
    Serial.println("[USB] host init failed; UART-only fallback active");
  }

  // GS Reset on boot is opt-in (config: StartupGSReset). Some users keep
  // tone/effect state on their connected synth across power cycles and don't
  // want the M5Tab to clobber it on every boot.
  if (g_config.startupGSReset) {
    sendGSReset();
    delay(30);
  }
  sendAllNotesOff();

  for (int i = 0; i < TRACKED_NOTE_STATE_COUNT; i++) {
    currentNoteStates[i] = { false, 0, 0, 0 };
    savedNoteStates[i]   = { false, 0, 0, 0 };
  }

  initMidiManagementDefaults();

  computeLayout();
  initDirect();
  initKeys();
  initInstant();
  initSequence();

  ensureStorage();
  loadSequencesFromSD();
  scanSmfFiles();
  scanMp3Files();

  // Start with transpose 0 highlighted on the DIRECT grid (-5..+6 range).
  setCurrentTransposeButton();

  // Apply the boot config (default app/mode, initial transpose=base+offset,
  // bypass flags, MIDI input source, range, etc.). Falls back to defaults
  // if the SD load failed earlier — applyDeviceConfig() reads g_config which
  // setDefaultConfig() always populated first.
  applyDeviceConfig();

  drawInterface();

  // Start the dedicated MIDI I/O task on core 0 so input drain / output
  // pass-through stay responsive even when the main loop is busy redrawing
  // the SRC piano roll or other heavy UI work. Must come AFTER Serial2 is
  // initialised and the Serial2 TX mutex is created.
  startMidiIoTask();

#ifdef M5TAB_DIAG
  Serial.printf("[boot] Tab5 MIDI Transposer ready  panel=%dx%d  reset_reason=%d\n",
                M5.Display.width(), M5.Display.height(), (int)esp_reset_reason());
  Serial.printf("[boot] heap=%u psram=%u\n",
                (unsigned)esp_get_free_heap_size(),
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
#else
  Serial.printf("[boot] Tab5 MIDI Transposer ready  panel=%dx%d\n",
                M5.Display.width(), M5.Display.height());
#endif
}

#ifdef M5TAB_DIAG
// Lightweight memory-leak monitor. Sampled once per loop() iteration so a
// transient drop between reports is still captured; cost is one heap-counter
// read per iteration (~hundreds of ns each — far below 1% at our 18 µs/loop).
// Prints every 5 s. The signal that matters most is `all_min`: it must stay
// flat across the run, otherwise something is leaking.
static uint32_t g_diagLastReportMs = 0;
static uint32_t g_diagWinMinHeap   = UINT32_MAX;

static inline void diagSample() {
  uint32_t heap = (uint32_t)esp_get_free_heap_size();
  if (heap < g_diagWinMinHeap) g_diagWinMinHeap = heap;
}

static inline void diagMaybeReport(uint32_t nowMs) {
  if (nowMs - g_diagLastReportMs < 5000) return;
  uint32_t heapNow  = (uint32_t)esp_get_free_heap_size();
  uint32_t allMin   = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
  uint32_t psramNow = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  uint32_t psramMin = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);
  uint32_t stackHW  = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
  // midi_in / midi_out let the regression script prove the device is
  // actually receiving + emitting MIDI bytes — a flat all_min with a flat
  // midi_in is a deaf box, not a leak-free one.
  Serial.printf("[mem] heap=%u win_min=%u all_min=%u psram=%u psram_min=%u stack_hw=%u midi_in=%lu midi_out=%lu uptime_ms=%u\n",
                (unsigned)heapNow, (unsigned)g_diagWinMinHeap,
                (unsigned)allMin, (unsigned)psramNow,
                (unsigned)psramMin, (unsigned)stackHW,
                midiInCount, midiOutCount, (unsigned)nowMs);
  g_diagLastReportMs = nowMs;
  g_diagWinMinHeap   = heapNow;
}
#else
static inline void diagSample() {}
static inline void diagMaybeReport(uint32_t /*nowMs*/) {}
#endif

void loop() {
  diagSample();
  uint32_t now = millis();
  diagMaybeReport(now);
  serviceUsbHost(now);

  // MIDI input drain (processMidiInput) runs on the dedicated midi_io_task
  // (core 0). Heavy UI work in this loop no longer affects MIDI throughput.
  // Only the mode-specific OUTPUT/PLAYBACK helpers stay on the main loop here.
  if (currentApp == APP_PLAY && currentPlay == PLAY_SMF) {
    processSmf();
  } else if (currentApp == APP_PLAY && currentPlay == PLAY_MP3) {
    processMp3();
  }
  // PLAY_SRC, APP_TRANSPOSE, APP_MIDI need no extra per-loop work — the MIDI
  // task already handles them.

  processUsbSerialCommands();

  static uint32_t lastUI = 0;
  static uint32_t lastSmfHeaderRefresh = 0;
  static uint32_t lastMp3HeaderRefresh = 0;
  processDeferredStorageTasks(now);
  if (now - lastUI >= 20) {
    lastUI = now;
    M5.update();
    handleTouch();
    // Refresh MIDI activity stripes ~50 Hz so the flash is visible without
    // requiring a full header redraw.
    drawMidiActivityLines();

    if ((currentApp == APP_PLAY && currentPlay == PLAY_SMF) && smfPlaying && now - lastSmfHeaderRefresh >= 200) {
      lastSmfHeaderRefresh = now;
      needPartialUpdate = true;
    }

    if ((currentApp == APP_PLAY && currentPlay == PLAY_MP3) && mp3Playing && now - lastMp3HeaderRefresh >= 500) {
      lastMp3HeaderRefresh = now;
      needPartialUpdate = true;
    }

    if (needFullRedraw) {
      drawInterface();
      needFullRedraw = false;
      midiManageDirty = false;
      needPartialUpdate = false;
    } else {
      if (currentApp == APP_MIDI && midiManageDirty) {
        drawMidiManage();
        midiManageDirty = false;
      }
      // SRC partial-redraw: only run when the SRC screen is actually showing.
      // Skip during CONFIG/BASE overlays (they cover the whole content area)
      // and while the fullscreen instrument picker is open — otherwise the
      // piano roll paints right on top of the overlay's UI.
      bool srcVisible = (currentApp == APP_PLAY && currentPlay == PLAY_SRC)
                     && (currentMode != CONFIG_EDIT_MODE)
                     && (currentMode != BASE_SET_MODE)
                     && !g_srcPickerOpen;
      if (srcVisible && srcDirtyAll) {
        drawSrc();
      }
      if (srcVisible) {
        // Incremental key-light updates so the live keyboard tracks input
        // without a full repaint every tick. Piano roll interior scrolls
        // every tick (cheap fillRect over a small panel — its border was
        // drawn once during the full repaint and stays put).
        srcFlushKeyboardDirty();
        srcDrawRoll();
      }
      if ((currentApp == APP_PLAY && currentPlay == PLAY_SMF) && smfPlaying) {
        Rect fullArea = {
          contentArea.x,
          contentArea.y,
          contentArea.w,
          contentArea.h + navArea.h
        };
        flushSmfMonitorDirty(fullArea);
      }
      if ((currentApp == APP_PLAY && currentPlay == PLAY_MP3) && (mp3StaticDirty || mp3VisualDirty)) {
        drawMp3();
      }
      if (needPartialUpdate) {
        drawHeaderStatusApp();
        needPartialUpdate = false;
      }
    }
  }
}
