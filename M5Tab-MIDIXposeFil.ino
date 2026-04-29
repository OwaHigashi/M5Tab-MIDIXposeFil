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
// MIDI I/O is unified on Tab5 PortA by repurposing the two signal pins as UART:
//   RX = G54, TX = G53
// This matches a 4-pin Unit connection physically plugged into PortA.
//
// Differences vs the M5Core2 original:
//   - Full 1280x720 layout, FreeSans proportional fonts, larger tap targets.
//   - No hardware buttons on Tab5, so A/B/C are replaced by on-screen toolbar
//     buttons (ALL OFF, RANGE, MODE).
//   - BT foot pedal support moved from classic-BT L2CAP HID (unavailable on
//     ESP32-P4) to BLE HID host.  The pedal must advertise as a BLE HID
//     device (HID service 0x1812).  Most modern "Bluetooth music pedals"
//     are BLE HID by default.
//   - SD access is unified on the Tab5 SPI-wired microSD slot so the SMF
//     library, MP3 decoder and transposer sequence storage share one card.

#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <vector>
#include <math.h>
#include <SdFat.h>
#include <M5Unified.h>
#include <AudioFileSourceFS.h>
#include <AudioFileSourceID3.h>
#include <AudioGeneratorMP3.h>

#define SD_FAT_TYPE 3
#include "src/MD_MIDIFile.h"
#include "src/AudioOutputM5Speaker.h"
#include "src/ble_hid.h"

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
// XPOSE 内のサブモード (4 種)
enum DisplayMode { DIRECT_MODE, KEY_MODE, INSTANT_MODE, SEQUENCE_MODE };
// PLAY 内のサブモード (2 種)
enum PlayMode { PLAY_SMF, PLAY_MP3 };
enum TransposeRange { RANGE_0_TO_12, RANGE_MINUS12_TO_0, RANGE_MINUS5_TO_6 };

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
static PlayMode currentPlay = PLAY_SMF;
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

// ==== BT HID foot pedal ====
// The pedal reports are 8-byte HID boot-keyboard reports:
//   [0]=modifier, [1]=reserved, [2..7]=up to 6 keys (USB HID Usage IDs).
// SPT-10 / generic music pedals send Up-Arrow (0x52) and Down-Arrow (0x51).
static const uint8_t PEDAL_LEFT_KEY  = 0x52;  // Up arrow
static const uint8_t PEDAL_RIGHT_KEY = 0x51;  // Down arrow

static portMUX_TYPE    g_pedalMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool   g_pedalLeftHeld  = false;
static volatile bool   g_pedalRightHeld = false;
static volatile bool   g_pedalDirty     = false;
static BT_STATUS       g_lastBtStatus   = BT_UNINITIALIZED;

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
static MD_MIDIFile smf;
static std::vector<String> smfPlaylist;
static int smfCurrentTrack = 0;
static int smfListScroll = 0;
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
static std::vector<String> mp3Playlist;
static int mp3CurrentTrack = 0;
static int mp3ListScroll = 0;
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
// PLAY app sub-mode tabs (SMF / MP3)
static Rect playTab[2];
static const char* playName[2] = { "SMF", "MP3" };
static Rect btnAllOff;        // aux
static Rect btnRange;         // aux (DIRECT range cycle / KEY upper/lower swap)
static Rect btnPrev, btnNext; // pedal-replacement
static Rect smfBtnPrev, smfBtnPlay, smfBtnNext, smfBtnLoop;
static Rect mp3BtnPrev, mp3BtnPlay, mp3BtnNext, mp3BtnVolDown, mp3BtnVolUp;

static ValueBtn directBtns[12];
static KeyBtn   majorKeys[12];
static KeyBtn   minorKeys[12];
static ValueBtn instantBtns[8];
static Rect     instantZero;

static SeqSlot  seqSteps[SEQ_STEP_COUNT];
static Rect     seqPatLeft, seqPatRight, seqSave;
static Rect     smfListArea, smfInfoArea, smfPianoArea;
static Rect     smfListUpBtn, smfListDownBtn;
static Rect     mp3ListArea, mp3InfoArea, mp3VisualArea;
static Rect     mp3ListUpBtn, mp3ListDownBtn;

// Touch input uses M5Unified's edge-triggered wasPressed(), so no manual latch.

// ==== Redraw flags ====
static bool needFullRedraw    = true;
static bool needPartialUpdate = false;
static bool midiManageDirty   = false;
static uint32_t g_lastMidiInputAt = 0;

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
static void drawHeaderStatusApp();
static void drawToolbarApp();
static void drawNavApp();
static void updateStatusArea();
static void handleTouch();
static void setCurrentApp(AppMode app);
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
static void processMIDI();
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
static void pedalReportCb(const uint8_t* rpt, size_t len);
static void processPedal();
static const char* btStatusLabel(BT_STATUS s);
static uint16_t     btStatusColor(BT_STATUS s);

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
  int s = (r.w < r.h ? r.w : r.h) / 3;
  if (up) {
    M5.Display.fillTriangle(cx, cy - s, cx - s, cy + s, cx + s, cy + s, triColor);
  } else {
    M5.Display.fillTriangle(cx - s, cy - s, cx + s, cy - s, cx, cy + s, triColor);
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
  int appW = 140;
  int appTabHX0 = 460;                                       // start x in header
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
  // PLAY: 2 mode tabs SMF/MP3 share the toolbar with transport buttons.
  int playTabW = 130;
  int playTabGap = 8;
  playTab[0] = { toolStartX,                            tabY, playTabW, tabH };
  playTab[1] = { toolStartX + playTabW + playTabGap,    tabY, playTabW, tabH };
  // Transport buttons start to the right of PLAY tabs.
  int transStartX = toolStartX + (playTabW + playTabGap) * 2;

  btnRange  = { SCREEN_W - (auxW * 2 + auxGap + 20), tabY, auxW, tabH };
  btnAllOff = { SCREEN_W - (auxW + 20),              tabY, auxW, tabH };

  int playerGap = 12;
  // Transport buttons share the right portion next to the SMF/MP3 mode tabs and
  // before the AOFF button. Compute available width from transStartX.
  int transRightX = btnAllOff.x - 12;  // leave some space before AOFF
  int playerAvail = transRightX - transStartX;
  int smfPrevW = 170;
  int smfPlayW = 230;
  int smfNextW = 170;
  int smfLoopW = playerAvail - (smfPrevW + smfPlayW + smfNextW + playerGap * 3);
  if (smfLoopW < 120) smfLoopW = 120;
  smfBtnPrev = { transStartX, tabY, smfPrevW, tabH };
  smfBtnPlay = { smfBtnPrev.x + smfBtnPrev.w + playerGap, tabY, smfPlayW, tabH };
  smfBtnNext = { smfBtnPlay.x + smfBtnPlay.w + playerGap, tabY, smfNextW, tabH };
  smfBtnLoop = { smfBtnNext.x + smfBtnNext.w + playerGap, tabY, smfLoopW, tabH };

  int mp3PrevW = 160;
  int mp3PlayW = 220;
  int mp3NextW = 160;
  int mp3VolW = (playerAvail - (mp3PrevW + mp3PlayW + mp3NextW + playerGap * 4)) / 2;
  if (mp3VolW < 100) mp3VolW = 100;
  mp3BtnPrev    = { transStartX, tabY, mp3PrevW, tabH };
  mp3BtnPlay    = { mp3BtnPrev.x + mp3BtnPrev.w + playerGap, tabY, mp3PlayW, tabH };
  mp3BtnNext    = { mp3BtnPlay.x + mp3BtnPlay.w + playerGap, tabY, mp3NextW, tabH };
  mp3BtnVolDown = { mp3BtnNext.x + mp3BtnNext.w + playerGap, tabY, mp3VolW, tabH };
  mp3BtnVolUp   = { mp3BtnVolDown.x + mp3BtnVolDown.w + playerGap, tabY, mp3VolW, tabH };

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
  smfListUpBtn   = { smfListArea.x + smfListArea.w - 102, smfListArea.y + 8, 44, 44 };
  smfListDownBtn = { smfListArea.x + smfListArea.w - 52,  smfListArea.y + 8, 44, 44 };
  smfInfoArea  = { rightX, contentArea.y + margin, rightW, 140 };
  int smfKeyboardTop = smfInfoArea.y + smfInfoArea.h + 16;
  int smfKeyboardBottom = navArea.y + navArea.h - margin;
  smfPianoArea = { rightX, smfKeyboardTop,
                   rightW, smfKeyboardBottom - smfKeyboardTop };

  mp3ListArea   = { contentArea.x + margin, contentArea.y + margin,
                    listW, listBottom - (contentArea.y + margin) };
  mp3ListUpBtn   = { mp3ListArea.x + mp3ListArea.w - 102, mp3ListArea.y + 8, 44, 44 };
  mp3ListDownBtn = { mp3ListArea.x + mp3ListArea.w - 52,  mp3ListArea.y + 8, 44, 44 };
  mp3InfoArea   = { rightX, contentArea.y + margin, rightW, 118 };
  mp3VisualArea = { rightX, mp3InfoArea.y + mp3InfoArea.h + 12,
                    rightW, listBottom - (mp3InfoArea.y + mp3InfoArea.h + 12) };

  initPianoKeys(smfPianoArea);
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
  M5.Display.drawString("MIDI Transposer  -  Player  -  BLE Pedal", cx, cy + 40);

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

  const char* title = "MIDI Transposer";
  if (currentApp == APP_MIDI) {
    title = (midiManagePage == MIDI_PAGE_FILTER) ? "MIDI Filter" : "MIDI Mapper";
  } else if ((currentApp == APP_PLAY && currentPlay == PLAY_SMF)) {
    title = "SMF Player";
  } else if ((currentApp == APP_PLAY && currentPlay == PLAY_MP3)) {
    title = "MP3 Player";
  }

  M5.Display.setFont(FONT_TITLE);
  M5.Display.setTextColor(COL_TITLE, COL_PANEL);
  M5.Display.setTextDatum(middle_left);
  M5.Display.drawString(title, 30, headerArea.y + headerArea.h / 2);

  drawAppTabs();
  drawHeaderStatusApp();
}

static void updateStatusArea() {
  // Right-aligned status block. The clear width starts just past the app
  // tabs in the header so we don't wipe them out on partial refreshes.
  int sx = SCREEN_W - 30;
  int y  = headerArea.y + 10;
  int h  = headerArea.h - 20;
  int statusX = appTab[2].x + appTab[2].w + 12;
  int statusW = SCREEN_W - statusX;
  M5.Display.fillRect(statusX, headerArea.y, statusW, headerArea.h, COL_PANEL);

  // Transpose value — large.
  char buf[32];
  if (transposeValue > 0)       snprintf(buf, sizeof(buf), "Transpose  +%d", transposeValue);
  else if (transposeValue < 0)  snprintf(buf, sizeof(buf), "Transpose  %d",  transposeValue);
  else                          snprintf(buf, sizeof(buf), "Transpose   0");
  M5.Display.setFont(FONT_HUGE);
  M5.Display.setTextColor(COL_VALUE, COL_PANEL);
  M5.Display.setTextDatum(middle_right);
  M5.Display.drawString(buf, sx, y + h / 2);

  // Small MIDI I/O counters above the value line.
  char line[48];
  snprintf(line, sizeof(line), "IN %lu   OUT %lu", midiInCount, midiOutCount);
  M5.Display.setFont(FONT_TINY);
  M5.Display.setTextColor(COL_MUTED, COL_PANEL);
  M5.Display.setTextDatum(top_right);
  M5.Display.drawString(line, sx, headerArea.y + 8);

  // BT status indicator above the title on the left side.
  BT_STATUS bt = ble_hid_status();
  M5.Display.fillRect(30, headerArea.y + 4, 260, 24, COL_PANEL);
  M5.Display.setFont(FONT_TINY);
  M5.Display.setTextColor(btStatusColor(bt), COL_PANEL);
  M5.Display.setTextDatum(top_left);
  M5.Display.drawString(btStatusLabel(bt), 30, headerArea.y + 8);
}

static const char* btStatusLabel(BT_STATUS s) {
  switch (s) {
    case BT_CONNECTED:   return "BT PEDAL: connected";
    case BT_CONNECTING:  return "BT PEDAL: connecting...";
    case BT_SCANNING:    return "BT PEDAL: scanning...";
    case BT_DISCONNECTED:return "BT PEDAL: disconnected";
    default:             return "BT PEDAL: off";
  }
}
static uint16_t btStatusColor(BT_STATUS s) {
  switch (s) {
    case BT_CONNECTED:  return TFT_GREEN;
    case BT_CONNECTING: return TFT_YELLOW;
    case BT_SCANNING:   return TFT_CYAN;
    default:            return COL_MUTED;
  }
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

  int sx = SCREEN_W - 30;
  int y  = headerArea.y + 10;
  int h  = headerArea.h - 20;
  int statusX = appTab[2].x + appTab[2].w + 12;
  int statusW = SCREEN_W - statusX;
  M5.Display.fillRect(statusX, headerArea.y, statusW, headerArea.h, COL_PANEL);

  M5.Display.setFont(FONT_TINY);
  M5.Display.setTextColor(COL_MUTED, COL_PANEL);
  M5.Display.setTextDatum(top_right);

  if ((currentApp == APP_PLAY && currentPlay == PLAY_SMF)) {
    char line[48];
    snprintf(line, sizeof(line), "%d file(s)   %s",
             (int)smfPlaylist.size(), smfLoop ? "Loop ON" : "Loop OFF");
    M5.Display.drawString(line, sx, headerArea.y + 8);

    uint32_t elapsed = smfPlaying
                     ? smfPausedElapsedMs + (millis() - smfPlaybackStartMs)
                     : smfPausedElapsedMs;
    char buf[96];
    snprintf(buf, sizeof(buf), "%s   %02lu:%02lu",
             smfPlaying ? "PLAY" : "STOP",
             (unsigned long)(elapsed / 60000UL),
             (unsigned long)((elapsed / 1000UL) % 60UL));
    M5.Display.setFont(FONT_HUGE);
    M5.Display.setTextColor(smfPlaying ? COL_BTN_HI : COL_MUTED, COL_PANEL);
    M5.Display.setTextDatum(middle_right);
    M5.Display.drawString(buf, sx, y + h / 2);
  } else {
    char line[48];
    snprintf(line, sizeof(line), "%d file(s)   Vol %d%%",
             (int)mp3Playlist.size(), (mp3Volume * 100) / 255);
    M5.Display.drawString(line, sx, headerArea.y + 8);

    uint32_t elapsed = (mp3Playing && mp3PlaybackStartMs)
                     ? (millis() - mp3PlaybackStartMs) : 0;
    char buf[64];
    snprintf(buf, sizeof(buf), "%s   %02lu:%02lu",
             mp3Playing ? "PLAY" : "STOP",
             (unsigned long)(elapsed / 60000UL),
             (unsigned long)((elapsed / 1000UL) % 60UL));
    M5.Display.setFont(FONT_HUGE);
    M5.Display.setTextColor(mp3Playing ? COL_ACCENT : COL_MUTED, COL_PANEL);
    M5.Display.setTextDatum(middle_right);
    M5.Display.drawString(buf, sx, y + h / 2);
  }

  BT_STATUS bt = ble_hid_status();
  M5.Display.fillRect(30, headerArea.y + 4, 260, 24, COL_PANEL);
  M5.Display.setFont(FONT_TINY);
  M5.Display.setTextColor(btStatusColor(bt), COL_PANEL);
  M5.Display.setTextDatum(top_left);
  M5.Display.drawString(btStatusLabel(bt), 30, headerArea.y + 8);
}

static void drawAllOffBtn() {
  drawRectBtn(btnAllOff,
              allNotesOffEnabled ? COL_BTN_HI : COL_DANGER,
              COL_BTN_BDR, "AOFF",
              allNotesOffEnabled ? COL_BTN_TXT_HI : COL_BTN_TXT, FONT_MED);
}

static void drawAppTabs() {
  for (int i = 0; i < 3; ++i) {
    bool on = (currentApp == (AppMode)i);
    drawRectBtn(appTab[i], on ? COL_BTN_HI2 : COL_BTN, COL_BTN_BDR, appName[i],
                on ? COL_BTN_TXT_HI : COL_BTN_TXT, FONT_MED);
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

  // APP_PLAY: SMF/MP3 mode tabs + transport buttons of the active sub-mode.
  for (int i = 0; i < 2; i++) {
    bool on = ((PlayMode)i == currentPlay);
    drawRectBtn(playTab[i], on ? COL_BTN_HI : COL_BTN, COL_BTN_BDR, playName[i],
                on ? COL_BTN_TXT_HI : COL_BTN_TXT, FONT_MED);
  }
  if (currentPlay == PLAY_SMF) {
    drawRectBtn(smfBtnPrev, TFT_BLUE, COL_BTN_BDR, "PREV", COL_BTN_TXT, FONT_MED);
    drawRectBtn(smfBtnPlay, smfPlaying ? COL_DANGER : COL_BTN_HI, COL_BTN_BDR,
                smfPlaying ? "STOP" : "PLAY",
                smfPlaying ? COL_BTN_TXT : COL_BTN_TXT_HI, FONT_MED);
    drawRectBtn(smfBtnNext, TFT_BLUE, COL_BTN_BDR, "NEXT", COL_BTN_TXT, FONT_MED);
    drawRectBtn(smfBtnLoop, smfLoop ? COL_BTN_HI : COL_BTN, COL_BTN_BDR,
                smfLoop ? "LOOP ON" : "LOOP OFF",
                smfLoop ? COL_BTN_TXT_HI : COL_BTN_TXT, FONT_MED);
  } else {
    drawRectBtn(mp3BtnPrev, TFT_BLUE, COL_BTN_BDR, "PREV", COL_BTN_TXT, FONT_MED);
    drawRectBtn(mp3BtnPlay, mp3Playing ? COL_DANGER : COL_BTN_HI, COL_BTN_BDR,
                mp3Playing ? "STOP" : "PLAY",
                mp3Playing ? COL_BTN_TXT : COL_BTN_TXT_HI, FONT_MED);
    drawRectBtn(mp3BtnNext, TFT_BLUE, COL_BTN_BDR, "NEXT", COL_BTN_TXT, FONT_MED);
    drawRectBtn(mp3BtnVolDown, COL_BTN, COL_BTN_BDR, "VOL-", COL_BTN_TXT, FONT_MED);
    drawRectBtn(mp3BtnVolUp, COL_BTN, COL_BTN_BDR, "VOL+", COL_BTN_TXT, FONT_MED);
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
  M5.Display.drawString("MP3 Playlist", mp3ListArea.x + 12, mp3ListArea.y + 10);
  drawTriangleBtn(mp3ListUpBtn,   TFT_BLUE, COL_BTN_BDR, true,  COL_BTN_TXT);
  drawTriangleBtn(mp3ListDownBtn, TFT_BLUE, COL_BTN_BDR, false, COL_BTN_TXT);

  int lineH = 34;
  int top = mp3ListArea.y + 58;
  int listRight = mp3ListArea.x + mp3ListArea.w - 12;
  int visible = (mp3ListArea.h - 56) / lineH;
  if (mp3CurrentTrack < mp3ListScroll) mp3ListScroll = mp3CurrentTrack;
  if (mp3CurrentTrack >= mp3ListScroll + visible) mp3ListScroll = mp3CurrentTrack - visible + 1;
  if (mp3ListScroll < 0) mp3ListScroll = 0;
  for (int row = 0; row < visible; ++row) {
    int idx = mp3ListScroll + row;
    if (idx >= (int)mp3Playlist.size()) break;
    Rect rr = { mp3ListArea.x + 10, top + row * lineH, mp3ListArea.w - 20, lineH - 2 };
    rr.w = listRight - rr.x;
    bool on = (idx == mp3CurrentTrack);
    M5.Display.fillRoundRect(rr.x, rr.y, rr.w, rr.h, 6, on ? COL_BTN_HI2 : COL_PANEL);
    const char* slash = strrchr(mp3Playlist[idx].c_str(), '/');
    const char* name = slash ? slash + 1 : mp3Playlist[idx].c_str();
    drawTextFit(rr, name, FONT_TINY, on ? COL_BTN_TXT_HI : COL_BTN_TXT,
                on ? COL_BTN_HI2 : COL_PANEL);
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
  M5.Display.drawString("SMF Playlist", smfListArea.x + 12, smfListArea.y + 10);
  drawTriangleBtn(smfListUpBtn,   TFT_BLUE, COL_BTN_BDR, true,  COL_BTN_TXT);
  drawTriangleBtn(smfListDownBtn, TFT_BLUE, COL_BTN_BDR, false, COL_BTN_TXT);

  int lineH = 30;
  int top = smfListArea.y + 58;
  int visible = (smfListArea.h - 60) / lineH;
  if (smfCurrentTrack < smfListScroll) smfListScroll = smfCurrentTrack;
  if (smfCurrentTrack >= smfListScroll + visible) smfListScroll = smfCurrentTrack - visible + 1;
  if (smfListScroll < 0) smfListScroll = 0;
  int maxScroll = max(0, (int)smfPlaylist.size() - visible);
  if (smfListScroll > maxScroll) smfListScroll = maxScroll;
  for (int row = 0; row < visible; ++row) {
    int idx = smfListScroll + row;
    if (idx >= (int)smfPlaylist.size()) break;
    Rect rr = { smfListArea.x + 10, top + row * lineH, smfListArea.w - 20, lineH - 2 };
    bool on = (idx == smfCurrentTrack);
    M5.Display.fillRoundRect(rr.x, rr.y, rr.w, rr.h, 6, on ? COL_BTN_HI2 : COL_PANEL);
    const char* slash = strrchr(smfPlaylist[idx].c_str(), '/');
    const char* name = slash ? slash + 1 : smfPlaylist[idx].c_str();
    M5.Display.setFont(FONT_TINY);
    M5.Display.setTextColor(on ? COL_BTN_TXT_HI : COL_BTN_TXT, on ? COL_BTN_HI2 : COL_PANEL);
    M5.Display.setTextDatum(middle_left);
    M5.Display.drawString(name, rr.x + 8, rr.y + rr.h / 2);
  }
}

static void drawMp3() {
  if (mp3StaticDirty) drawMp3Static();
  if (mp3Playing || mp3VisualDirty) drawMp3Visual();
}

static void drawInterface() {
  M5.Display.fillScreen(COL_BG);
  drawHeader();
  drawToolbarApp();
  drawNavApp();
  switch (currentApp) {
    case APP_TRANSPOSE:
      switch (currentMode) {
        case DIRECT_MODE:      drawDirect();      break;
        case KEY_MODE:         drawKey();         break;
        case INSTANT_MODE:     drawInstant();     break;
        case SEQUENCE_MODE:    drawSequence();    break;
      }
      break;
    case APP_MIDI: drawMidiManage(); break;
    case APP_PLAY: if (currentPlay == PLAY_SMF) drawSmf(); else drawMp3(); break;
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

static void handleToolbarTouch(int x, int y) {
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

  // PLAY app: SMF / MP3 mode tabs
  if (currentApp == APP_PLAY) {
    for (int i = 0; i < 2; i++) {
      if (hit(playTab[i], x, y)) {
        if (currentPlay == (PlayMode)i) return;
        if (currentPlay == PLAY_SMF) stopSmf();
        else if (currentPlay == PLAY_MP3) stopMp3();
        currentPlay = (PlayMode)i;
        if (currentPlay == PLAY_SMF) {
          ensureStorage();
          if (smfPlaylist.empty()) scanSmfFiles();
          invalidateSmfMonitorAll();
        } else {
          ensureStorage();
          if (mp3Playlist.empty()) scanMp3Files();
        }
        needFullRedraw = true;
        return;
      }
    }
  }

  if ((currentApp == APP_PLAY && currentPlay == PLAY_SMF)) {
    if (hit(smfBtnPrev, x, y) && !smfPlaylist.empty()) {
      loadSmfTrack((smfCurrentTrack > 0) ? smfCurrentTrack - 1 : (int)smfPlaylist.size() - 1);
      needFullRedraw = true;
    } else if (hit(smfBtnPlay, x, y)) {
      if (smfPlaying) stopSmf();
      else playSmf();
      needFullRedraw = true;
    } else if (hit(smfBtnNext, x, y) && !smfPlaylist.empty()) {
      loadSmfTrack((smfCurrentTrack + 1) % (int)smfPlaylist.size());
      needFullRedraw = true;
    } else if (hit(smfBtnLoop, x, y)) {
      smfLoop = !smfLoop;
      smf.looping(smfLoop);
      needFullRedraw = true;
    }
    return;
  }

  if ((currentApp == APP_PLAY && currentPlay == PLAY_MP3)) {
    if (hit(mp3BtnPrev, x, y) && !mp3Playlist.empty()) {
      startMp3Track((mp3CurrentTrack > 0) ? mp3CurrentTrack - 1 : (int)mp3Playlist.size() - 1);
      needFullRedraw = true;
    } else if (hit(mp3BtnPlay, x, y)) {
      if (mp3Playing) stopMp3();
      else startMp3Track(mp3CurrentTrack);
      needFullRedraw = true;
    } else if (hit(mp3BtnNext, x, y) && !mp3Playlist.empty()) {
      startMp3Track((mp3CurrentTrack + 1) % (int)mp3Playlist.size());
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

  return;
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
      if (smfPlaylist.empty()) scanSmfFiles();
      invalidateSmfMonitorAll();
    } else if (currentPlay == PLAY_MP3) {
      ensureStorage();
      if (mp3Playlist.empty()) scanMp3Files();
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

static void scanSmfFiles() {
  smfPlaylist.clear();
  smfListScroll = 0;
  if (!ensureStorage()) return;

  File root = SD.open(SMF_FOLDER);
  if (!root || !root.isDirectory()) return;

  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    if (!entry.isDirectory()) {
      String name = entry.name();
      if (name.endsWith(".mid") || name.endsWith(".MID") ||
          name.endsWith(".smf") || name.endsWith(".SMF")) {
        smfPlaylist.push_back(String(SMF_FOLDER) + "/" + name);
      }
    }
    entry.close();
  }
  root.close();

  if (!smfPlaylist.empty()) {
    loadSmfTrack(min(smfCurrentTrack, (int)smfPlaylist.size() - 1));
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
  if (!ensureStorage() || !midiFsReady || smfPlaylist.empty()) return false;
  if (index < 0 || index >= (int)smfPlaylist.size()) return false;

  closeSmf();
  smf.begin(&midiSd);
  smf.setMidiHandler(smfMidiEventHandler);
  smf.setSysexHandler(smfSysexEventHandler);
  smf.setMetaHandler(smfMetaEventHandler);
  smf.looping(smfLoop);

  smfCurrentTrack = index;
  String path = smfPlaylist[index];
  const char* slash = strrchr(path.c_str(), '/');
  const char* name = slash ? slash + 1 : path.c_str();
  strncpy(smfCurrentName, name, sizeof(smfCurrentName) - 1);
  smfCurrentName[sizeof(smfCurrentName) - 1] = '\0';

  int err = smf.load(path.c_str());
  smfLoaded = (err == MD_MIDIFile::E_OK);
  if (!smfLoaded) {
    Serial.printf("[SMF] load failed %s err=%d\n", path.c_str(), err);
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
  if (sendReset) {
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

  uint8_t status = (pev->data[0] & 0xF0) | (pev->channel & 0x0F);
  uint8_t channel = pev->channel & 0x0F;
  uint8_t message[4];
  uint8_t messageSize = min((uint8_t)4, pev->size);
  message[0] = status;
  for (uint8_t i = 1; i < messageSize; ++i) {
    message[i] = pev->data[i];
  }
  for (uint8_t i = 0; i < messageSize; ++i) {
    Serial2.write(message[i]);
    ++midiOutCount;
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
  for (uint16_t i = 0; i < pev->size; ++i) {
    Serial2.write(pev->data[i]);
    ++midiOutCount;
  }
}

static void smfMetaEventHandler(const meta_event* mev) {
  (void)mev;
}

static void processSmf() {
  if (!smfPlaying || !smfLoaded) return;
  bool advanced = smf.getNextEvent();
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
  mp3Playlist.clear();
  mp3ListScroll = 0;
  if (!ensureStorage()) return;

  File root = SD.open(MP3_FOLDER);
  if (!root || !root.isDirectory()) return;

  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    if (!entry.isDirectory()) {
      String name = entry.name();
      if (name.endsWith(".mp3") || name.endsWith(".MP3")) {
        mp3Playlist.push_back(String(MP3_FOLDER) + "/" + name);
      }
    }
    entry.close();
  }
  root.close();

  if (!mp3Playlist.empty()) {
    String path = mp3Playlist[min(mp3CurrentTrack, (int)mp3Playlist.size() - 1)];
    const char* slash = strrchr(path.c_str(), '/');
    const char* name = slash ? slash + 1 : path.c_str();
    strncpy(mp3CurrentName, name, sizeof(mp3CurrentName) - 1);
    mp3CurrentName[sizeof(mp3CurrentName) - 1] = '\0';
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
  if (!ensureStorage() || mp3Playlist.empty()) return false;
  if (index < 0 || index >= (int)mp3Playlist.size()) return false;

  stopMp3();
  mp3CurrentTrack = index;
  String path = mp3Playlist[index];
  const char* slash = strrchr(path.c_str(), '/');
  const char* name = slash ? slash + 1 : path.c_str();
  strncpy(mp3CurrentName, name, sizeof(mp3CurrentName) - 1);
  mp3CurrentName[sizeof(mp3CurrentName) - 1] = '\0';
  mp3Title[0] = '\0';
  mp3Artist[0] = '\0';

  mp3File = new AudioFileSourceFS(SD, path.c_str());
  mp3Id3 = new AudioFileSourceID3(mp3File);
  mp3Id3->RegisterMetadataCB(mp3MetadataCallback, nullptr);
  M5.Speaker.setVolume(mp3Volume);
  mp3Playing = mp3Decoder.begin(mp3Id3, &mp3Out);
  mp3PlaybackStartMs = mp3Playing ? millis() : 0;
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
  if (hit(smfListUpBtn, x, y)) {
    if (smfListScroll > 0) {
      smfListScroll--;
      needFullRedraw = true;
    }
    return;
  }
  if (hit(smfListDownBtn, x, y)) {
    int lineH = 30;
    int visible = (smfListArea.h - 60) / lineH;
    int maxScroll = max(0, (int)smfPlaylist.size() - visible);
    if (smfListScroll < maxScroll) {
      smfListScroll++;
      needFullRedraw = true;
    }
    return;
  }
  if (hit(smfListArea, x, y)) {
    int top = smfListArea.y + 58;
    int lineH = 30;
    int idx = smfListScroll + ((y - top) / lineH);
    if (y >= top && idx >= 0 && idx < (int)smfPlaylist.size()) {
      loadSmfTrack(idx);
      needFullRedraw = true;
    }
  }
}

static void handleMp3Touch(int x, int y) {
  if (hit(mp3ListUpBtn, x, y)) {
    if (mp3ListScroll > 0) {
      mp3ListScroll--;
      mp3StaticDirty = true;
      needPartialUpdate = true;
    }
    return;
  }
  if (hit(mp3ListDownBtn, x, y)) {
    int lineH = 34;
    int visible = (mp3ListArea.h - 56) / lineH;
    int maxScroll = max(0, (int)mp3Playlist.size() - visible);
    if (mp3ListScroll < maxScroll) {
      mp3ListScroll++;
      mp3StaticDirty = true;
      needPartialUpdate = true;
    }
    return;
  }
  if (hit(mp3ListArea, x, y)) {
    int top = mp3ListArea.y + 58;
    int lineH = 34;
    int idx = mp3ListScroll + ((y - top) / lineH);
    if (y >= top && idx >= 0 && idx < (int)mp3Playlist.size()) {
      startMp3Track(idx);
      needFullRedraw = true;
    }
  }
}

// =================================================================
//  BLE HID foot pedal
// =================================================================
// Runs in BLE-host task context: do not touch UI/MIDI state here.
static void pedalReportCb(const uint8_t* rpt, size_t len) {
  if (len < 3) return;  // too short to be a keyboard report
  // Keyboard reports are [mod][reserved][key1..key6].  Some stacks drop the
  // reserved byte, so accept both "boot" (8) and "short" (>=3) forms.
  const uint8_t* keys;
  size_t nkeys;
  if (len >= 8) { keys = rpt + 2; nkeys = 6; }
  else          { keys = rpt + 1; nkeys = len - 1; }

  bool leftPressed  = false;
  bool rightPressed = false;
  for (size_t i = 0; i < nkeys; i++) {
    uint8_t k = keys[i];
    if (k == 0) continue;
    if (k == PEDAL_LEFT_KEY)  leftPressed  = true;
    if (k == PEDAL_RIGHT_KEY) rightPressed = true;
  }
  portENTER_CRITICAL(&g_pedalMux);
  if (leftPressed != g_pedalLeftHeld || rightPressed != g_pedalRightHeld) {
    g_pedalLeftHeld  = leftPressed;
    g_pedalRightHeld = rightPressed;
    g_pedalDirty = true;
  }
  portEXIT_CRITICAL(&g_pedalMux);
}

// Drains the pedal report state into edge events and dispatches them through
// the same shift-transpose path used by the on-screen PREV / NEXT buttons.
static void processPedal() {
  if (currentApp != APP_TRANSPOSE) return;
  bool dirty = false;
  bool left = false, right = false;
  portENTER_CRITICAL(&g_pedalMux);
  if (g_pedalDirty) {
    g_pedalDirty = false;
    left  = g_pedalLeftHeld;
    right = g_pedalRightHeld;
    dirty = true;
  }
  portEXIT_CRITICAL(&g_pedalMux);
  if (!dirty) return;

  static bool lastLeft = false, lastRight = false;
  bool leftEdge  = left  && !lastLeft;
  bool rightEdge = right && !lastRight;
  lastLeft  = left;
  lastRight = right;
  if (leftEdge)  shiftTransposeBy(-1);
  if (rightEdge) shiftTransposeBy(+1);
}

static void handleTouch() {
  auto t = M5.Touch.getDetail();
  if (!t.wasPressed() && !t.wasHold()) return;  // one-shot tap / hold-begin only

  int x = t.x, y = t.y;
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
    }
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
  return Serial2.available() == 0 && (now - g_lastMidiInputAt) >= 250;
}

static void processDeferredStorageTasks(uint32_t now) {
  if (g_seqSaveUiState != SEQ_SAVE_UI_IDLE && g_seqSaveUiUntil != 0 && now >= g_seqSaveUiUntil) {
    g_seqSaveUiState = SEQ_SAVE_UI_IDLE;
    g_seqSaveUiUntil = 0;
    needFullRedraw = true;
  }

  if (!g_seqSavePending || !isMidiInputIdle(now)) return;

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
  outputMidiMessage(mapped);                            // Transpose + 送信
}

static void sendMIDIMessage(uint8_t* buffer, int length) {
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
      }
    }
  } else {
    for (int i = 0; i < length; i++) Serial2.write(buffer[i]);
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
    if (allowCurrentSysEx) { Serial2.write(data); midiOutCount++; }
    if (data == 0xF7) inSysEx = false;
    return;
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

static void processMIDI() {
  bool sawInput = false;
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    sawInput = true;
    midiInCount++;
    processMIDIByte(b);
  }
  if (sawInput) g_lastMidiInputAt = millis();
}

static void sendAllNotesOff() {
  for (int ch = 0; ch < 16; ch++) {
    Serial2.write(0xB0 | ch); Serial2.write((uint8_t)123); Serial2.write((uint8_t)0);
    Serial2.write(0xB0 | ch); Serial2.write((uint8_t)120); Serial2.write((uint8_t)0);
    midiOutCount += 6;
  }
  clearTrackedNoteStates();
}

static void sendGSReset() {
  static const uint8_t gsReset[] = {
    0xF0, 0x41, 0x10, 0x42, 0x12, 0x40, 0x00, 0x7F, 0x00, 0x41, 0xF7
  };
  for (uint8_t b : gsReset) {
    Serial2.write(b);
    ++midiOutCount;
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
    case APP_PLAY:      return (currentPlay == PLAY_SMF) ? "PLAY:SMF" : "PLAY:MP3";
    default:            return "UNKNOWN";
  }
}

static const char* getBtStatusLabelTab(BT_STATUS s) {
  switch (s) {
    case BT_UNINITIALIZED: return "UNINITIALIZED";
    case BT_DISCONNECTED:  return "DISCONNECTED";
    case BT_CONNECTING:    return "CONNECTING";
    case BT_CONNECTED:     return "CONNECTED";
    default:               return "UNKNOWN";
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
  if (tokenEqualsIgnoreCase(mode, "SMF"))      { setCurrentApp(APP_PLAY); currentPlay = PLAY_SMF; needFullRedraw = true; return true; }
  if (tokenEqualsIgnoreCase(mode, "MP3"))      { setCurrentApp(APP_PLAY); currentPlay = PLAY_MP3; needFullRedraw = true; return true; }
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
  Serial.printf(
    "OK STATUS app=%s mode=%s transpose=%d range=%d filter_bypass=%d mapper_bypass=%d "
    "filter_rule=%d/%d mapper_rule=%d/%d page=%s mapper_page=%s midi_in=%lu midi_out=%lu bt=%s\n",
    getAppLabel(currentApp),
    getDisplayModeLabel(currentMode),
    (int)transposeValue,
    (int)transposeRange,
    midiFilterBypass ? 1 : 0,
    midiMapperBypass ? 1 : 0,
    midiSelectedFilterRule + 1, midiFilterRuleCount,
    midiSelectedMapperRule + 1, midiMapperRuleCount,
    pageLabel, mapperPageLabel,
    midiInCount, midiOutCount,
    getBtStatusLabelTab(g_lastBtStatus)
  );
}

static void printUsbSerialHelp() {
  Serial.println("OK HELP BEGIN");
  Serial.println("HELP");
  Serial.println("STATUS");
  Serial.println("REDRAW");
  Serial.println("BUTTON A|B|C [LONG]");
  Serial.println("TOUCH <x> <y>");
  Serial.println("MODE DIRECT|KEY|INSTANT|SEQUENCE|FILTER|MAPPER|MIDI|SMF|MP3");
  Serial.println("GROUP TRANSPOSE|MIDI");
  Serial.println("SET TRANSPOSE <-12..12>");
  Serial.println("SET FILTER BYPASS|ACTIVE");
  Serial.println("SET MAPPER BYPASS|ACTIVE");
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
    if (target && tokenEqualsIgnoreCase(target, "FILTER")) {
      char* val = strtok_r(nullptr, " \t", &save);
      if (val && tokenEqualsIgnoreCase(val, "BYPASS")) { midiFilterBypass = true;  needFullRedraw = true; Serial.println("OK SET FILTER BYPASS"); return; }
      if (val && tokenEqualsIgnoreCase(val, "ACTIVE")) { midiFilterBypass = false; needFullRedraw = true; Serial.println("OK SET FILTER ACTIVE"); return; }
      Serial.println("ERR SET FILTER requires BYPASS or ACTIVE");
      return;
    }
    if (target && tokenEqualsIgnoreCase(target, "MAPPER")) {
      char* val = strtok_r(nullptr, " \t", &save);
      if (val && tokenEqualsIgnoreCase(val, "BYPASS")) { midiMapperBypass = true;  needFullRedraw = true; Serial.println("OK SET MAPPER BYPASS"); return; }
      if (val && tokenEqualsIgnoreCase(val, "ACTIVE")) { midiMapperBypass = false; needFullRedraw = true; Serial.println("OK SET MAPPER ACTIVE"); return; }
      Serial.println("ERR SET MAPPER requires BYPASS or ACTIVE");
      return;
    }
    Serial.println("ERR SET supports TRANSPOSE / FILTER / MAPPER");
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

  drawSplash();

  Serial2.begin(MIDI_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial2.setRxBufferSize(1024);
  Serial2.setTxBufferSize(512);
  sendGSReset();
  delay(30);
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

  drawInterface();

  // BLE HID pedal — initialise asynchronously so the UI stays responsive
  // while the C6 radio comes up and NimBLE does its setup.
  ble_hid_begin_async("M5Tab5-MIDITransposer", pedalReportCb);

  Serial.printf("[boot] Tab5 MIDI Transposer ready  panel=%dx%d\n",
                M5.Display.width(), M5.Display.height());
}

void loop() {
  if (currentApp == APP_TRANSPOSE || currentApp == APP_MIDI) {
    processMIDI();
    processPedal();
  } else if ((currentApp == APP_PLAY && currentPlay == PLAY_SMF)) {
    processSmf();
  } else {
    processMp3();
  }

  processUsbSerialCommands();

  static uint32_t lastUI = 0;
  static uint32_t lastSmfHeaderRefresh = 0;
  static uint32_t lastMp3HeaderRefresh = 0;
  uint32_t now = millis();
  processDeferredStorageTasks(now);
  if (now - lastUI >= 20) {
    lastUI = now;
    M5.update();
    handleTouch();
    ble_hid_service();

    // Refresh the header whenever the BT status changes.
    BT_STATUS bt = ble_hid_status();
    if (bt != g_lastBtStatus) {
      g_lastBtStatus = bt;
      needPartialUpdate = true;
    }

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
