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
enum AppMode { APP_TRANSPOSE, APP_SMF, APP_MP3 };
enum DisplayMode { DIRECT_MODE, KEY_MODE, INSTANT_MODE, SEQUENCE_MODE };
enum TransposeRange { RANGE_0_TO_12, RANGE_MINUS12_TO_0, RANGE_MINUS5_TO_6 };

struct Rect { int x, y, w, h; };
struct ValueBtn { Rect r; int8_t value; char label[5]; };
struct KeyBtn   { Rect r; int8_t keyValue; const char* keyName; bool isBlackKey; };
struct SeqSlot  { Rect up, slot, down; };
struct PianoKeyGeom { Rect r; uint8_t note; bool isBlackKey; };
struct SmfKeyGeom { Rect r; bool isBlackKey; };

// ==== State ====
static AppMode currentApp = APP_TRANSPOSE;
static DisplayMode currentMode = DIRECT_MODE;
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

// ==== Storage ====
static bool storageReady = false;
static bool midiFsReady = false;
static SdFs midiSd;

// ==== 88-key tracking (A0=21 .. C8=108) ====
#define PIANO_LOWEST_NOTE 21
#define PIANO_HIGHEST_NOTE 108
#define PIANO_KEY_COUNT   88
struct NoteState {
  bool isActive;
  int8_t originalTranspose;
  uint8_t channel;
  uint8_t velocity;
};
static NoteState currentNoteStates[PIANO_KEY_COUNT];
static NoteState savedNoteStates  [PIANO_KEY_COUNT];
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
static const char* appName[3] = { "XPOSE", "MIDI", "MP3" };
static Rect modeTab[4];       // DIRECT / KEY / INSTANT / SEQUENCE
static const char* modeName[4] = { "DIRECT", "KEY", "INST.", "SEQ." };
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

// ==== Forward decls ====
static void computeLayout();
static void initDirect();
static void initKeys();
static void initInstant();
static void initSequence();
static void initPianoKeys(const Rect& area);
static void drawInterface();
static void drawHeader();
static void drawToolbar();
static void drawNav();
static void drawDirect();
static void drawKey();
static void drawInstant();
static void drawSequence();
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
  int tabY = toolbarArea.y + 10;
  int tabH = toolbarArea.h - 20;

  int appGap = 6;
  int appW = 124;
  for (int i = 0; i < 3; ++i) {
    appTab[i] = { tabX + i * (appW + appGap), tabY, appW, tabH };
  }

  int toolStartX = appTab[2].x + appTab[2].w + 18;
  int auxW = 138;
  int auxGap = 8;
  int toolRightX = SCREEN_W - 20 - (auxW * 2 + auxGap);
  int transTabGap = 12;
  int transTabW = (toolRightX - toolStartX - transTabGap * 3) / 4;
  for (int i = 0; i < 4; i++) {
    modeTab[i] = { toolStartX + i * (transTabW + transTabGap), tabY, transTabW, tabH };
  }

  btnRange  = { SCREEN_W - (auxW * 2 + auxGap + 20), tabY, auxW, tabH };
  btnAllOff = { SCREEN_W - (auxW + 20),              tabY, auxW, tabH };

  int playerGap = 12;
  int playerAvail = SCREEN_W - 20 - toolStartX;
  int smfPrevW = 150;
  int smfPlayW = 210;
  int smfNextW = 150;
  int smfLoopW = playerAvail - (smfPrevW + smfPlayW + smfNextW + playerGap * 3);
  smfBtnPrev = { toolStartX, tabY, smfPrevW, tabH };
  smfBtnPlay = { smfBtnPrev.x + smfBtnPrev.w + playerGap, tabY, smfPlayW, tabH };
  smfBtnNext = { smfBtnPlay.x + smfBtnPlay.w + playerGap, tabY, smfNextW, tabH };
  smfBtnLoop = { smfBtnNext.x + smfBtnNext.w + playerGap, tabY, smfLoopW, tabH };

  int mp3PrevW = 140;
  int mp3PlayW = 190;
  int mp3NextW = 140;
  int mp3VolW = (playerAvail - (mp3PrevW + mp3PlayW + mp3NextW + playerGap * 4)) / 2;
  mp3BtnPrev    = { toolStartX, tabY, mp3PrevW, tabH };
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
static void drawHeader() {
  M5.Display.fillRect(headerArea.x, headerArea.y, headerArea.w, headerArea.h, COL_PANEL);

  const char* title = "MIDI Transposer";
  if (currentApp == APP_SMF) title = "MIDI Player";
  else if (currentApp == APP_MP3) title = "MP3 Player";

  M5.Display.setFont(FONT_TITLE);
  M5.Display.setTextColor(COL_TITLE, COL_PANEL);
  M5.Display.setTextDatum(middle_left);
  M5.Display.drawString(title, 30, headerArea.y + headerArea.h / 2);

  drawHeaderStatusApp();
}

static void updateStatusArea() {
  // Right-aligned status block.
  int sx = SCREEN_W - 30;
  int y  = headerArea.y + 10;
  int h  = headerArea.h - 20;
  M5.Display.fillRect(sx - 640, headerArea.y, 640, headerArea.h, COL_PANEL);

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

  // Mode tabs.
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
  M5.Display.fillRect(sx - 640, headerArea.y, 640, headerArea.h, COL_PANEL);

  M5.Display.setFont(FONT_TINY);
  M5.Display.setTextColor(COL_MUTED, COL_PANEL);
  M5.Display.setTextDatum(top_right);

  if (currentApp == APP_SMF) {
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

    char buf[96];
    snprintf(buf, sizeof(buf), "%s   %s",
             mp3Playing ? "PLAY" : "STOP",
             mp3Artist[0] ? mp3Artist : "Local SD");
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

static void drawToolbarApp() {
  if (currentApp == APP_TRANSPOSE) {
    drawToolbar();
    for (int i = 0; i < 3; ++i) {
      bool on = (currentApp == (AppMode)i);
      drawRectBtn(appTab[i], on ? COL_BTN_HI2 : COL_BTN, COL_BTN_BDR, appName[i],
                  on ? COL_BTN_TXT_HI : COL_BTN_TXT, FONT_MED);
    }
    return;
  }

  M5.Display.fillRect(toolbarArea.x, toolbarArea.y, toolbarArea.w, toolbarArea.h, COL_BG);
  for (int i = 0; i < 3; ++i) {
    bool on = (currentApp == (AppMode)i);
    drawRectBtn(appTab[i], on ? COL_BTN_HI2 : COL_BTN, COL_BTN_BDR, appName[i],
                on ? COL_BTN_TXT_HI : COL_BTN_TXT, FONT_MED);
  }

  if (currentApp == APP_SMF) {
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
  drawRectBtn(seqSave, TFT_RED, COL_BTN_BDR, "SAVE", COL_BTN_TXT, FONT_HUGE);

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
        case DIRECT_MODE:   drawDirect();   break;
        case KEY_MODE:      drawKey();      break;
        case INSTANT_MODE:  drawInstant();  break;
        case SEQUENCE_MODE: drawSequence(); break;
      }
      break;
    case APP_SMF: drawSmf(); break;
    case APP_MP3: drawMp3(); break;
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
  for (int i = 0; i < 3; ++i) {
    if (hit(appTab[i], x, y)) {
      setCurrentApp((AppMode)i);
      return;
    }
  }

  if (currentApp == APP_SMF) {
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

  if (currentApp == APP_MP3) {
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

  // Mode tabs.
  for (int i = 0; i < 4; i++) {
    if (hit(modeTab[i], x, y)) {
      if (currentMode == (DisplayMode)i) return;
      sendAllNotesOff();
      delay(5);
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
    bool ok = saveSequencesToSD();
    drawRectBtn(seqSave,
                ok ? TFT_GREEN : TFT_ORANGE,
                COL_BTN_BDR,
                ok ? "SAVED" : "SD ERR",
                TFT_BLACK, FONT_LARGE);
    delay(600);
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
  if (currentApp == APP_SMF) stopSmf();
  if (currentApp == APP_MP3) stopMp3();

  currentApp = app;
  if (currentApp == APP_SMF) {
    ensureStorage();
    if (smfPlaylist.empty()) scanSmfFiles();
    invalidateSmfMonitorAll();
  } else if (currentApp == APP_MP3) {
    ensureStorage();
    if (mp3Playlist.empty()) scanMp3Files();
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
  if (!t.wasPressed()) return;  // M5Unified gives us a one-shot edge

  int x = t.x, y = t.y;
  // Toolbar / nav are always available.
  if (y < toolbarArea.y + toolbarArea.h && y >= toolbarArea.y) {
    handleToolbarTouch(x, y);
    return;
  }
  if (y >= navArea.y) {
    handleNavTouch(x, y);
    return;
  }
  // Otherwise it's content-area input.
  if (currentApp == APP_TRANSPOSE) {
    switch (currentMode) {
      case DIRECT_MODE:   handleDirectTouch(x, y);   break;
      case KEY_MODE:      handleKeyTouch(x, y);      break;
      case INSTANT_MODE:  handleInstantTouch(x, y);  break;
      case SEQUENCE_MODE: handleSequenceTouch(x, y); break;
    }
  } else if (currentApp == APP_SMF) {
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
    delay(5);
    transposeValue = newValue;
    needPartialUpdate = true;
    return;
  }

  // Smooth transpose: remember which notes were on at the old transpose so
  // their matching Note-Off messages can undo the correct offset.
  for (int i = 0; i < PIANO_KEY_COUNT; i++) savedNoteStates[i] = currentNoteStates[i];
  transposeValue = newValue;
  needPartialUpdate = true;
}

static int getPianoKeyIndex(uint8_t midiNote) {
  if (midiNote < PIANO_LOWEST_NOTE || midiNote > PIANO_HIGHEST_NOTE) return -1;
  return midiNote - PIANO_LOWEST_NOTE;
}

static int getMIDIMessageLength(uint8_t status) {
  uint8_t type = status & 0xF0;
  switch (type) {
    case 0x80: case 0x90: case 0xA0: case 0xB0: case 0xE0: return 3;
    case 0xC0: case 0xD0: return 2;
    default: return 1;
  }
}

static void sendMIDIMessage(uint8_t* buffer, int length) {
  uint8_t status = buffer[0];
  uint8_t type = status & 0xF0;
  uint8_t channel = status & 0x0F;

  if ((type == 0x90 || type == 0x80) && length == 3) {
    uint8_t note = buffer[1];
    uint8_t vel  = buffer[2];
    bool isNoteOn = (type == 0x90) && (vel > 0);
    int idx = getPianoKeyIndex(note);

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

static void processMIDIByte(uint8_t data) {
  static uint8_t midiBuffer[3];
  static int bufferIndex = 0;
  static uint8_t runningStatus = 0;
  static bool inSysEx = false;

  if (data >= 0xF8) {                 // real-time: pass through
    Serial2.write(data);
    midiOutCount++;
    return;
  }
  if (inSysEx) {
    Serial2.write(data);
    midiOutCount++;
    if (data == 0xF7) inSysEx = false;
    return;
  }
  if (data == 0xF0) {
    inSysEx = true;
    Serial2.write(data);
    midiOutCount++;
    return;
  }
  if (data & 0x80) {
    runningStatus = data;
    midiBuffer[0] = data;
    bufferIndex = 1;
    if (data >= 0xF0 && data < 0xF8) {
      Serial2.write(data);
      midiOutCount++;
      return;
    }
  } else if (runningStatus != 0) {
    midiBuffer[bufferIndex++] = data;
  } else {
    return;
  }
  int messageLength = getMIDIMessageLength(runningStatus);
  if (bufferIndex >= messageLength) {
    sendMIDIMessage(midiBuffer, messageLength);
    bufferIndex = 1;
  }
}

static void processMIDI() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    midiInCount++;
    processMIDIByte(b);
  }
}

static void sendAllNotesOff() {
  for (int ch = 0; ch < 16; ch++) {
    Serial2.write(0xB0 | ch); Serial2.write((uint8_t)123); Serial2.write((uint8_t)0);
    Serial2.write(0xB0 | ch); Serial2.write((uint8_t)120); Serial2.write((uint8_t)0);
    midiOutCount += 6;
  }
  for (int i = 0; i < PIANO_KEY_COUNT; i++) {
    currentNoteStates[i].isActive = false;
    savedNoteStates[i].isActive   = false;
  }
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

  Serial2.begin(MIDI_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial2.setRxBufferSize(256);
  Serial2.setTxBufferSize(256);
  sendGSReset();
  delay(30);
  sendAllNotesOff();

  for (int i = 0; i < PIANO_KEY_COUNT; i++) {
    currentNoteStates[i] = { false, 0, 0, 0 };
    savedNoteStates[i]   = { false, 0, 0, 0 };
  }

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
  if (currentApp == APP_TRANSPOSE) {
    processMIDI();
    processPedal();
  } else if (currentApp == APP_SMF) {
    processSmf();
  } else {
    processMp3();
  }

  static uint32_t lastUI = 0;
  static uint32_t lastSmfHeaderRefresh = 0;
  uint32_t now = millis();
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

    if (currentApp == APP_SMF && smfPlaying && now - lastSmfHeaderRefresh >= 200) {
      lastSmfHeaderRefresh = now;
      needPartialUpdate = true;
    }

    if (needFullRedraw) {
      drawInterface();
      needFullRedraw = false;
      needPartialUpdate = false;
    } else if (currentApp == APP_SMF && smfPlaying) {
      Rect fullArea = {
        contentArea.x,
        contentArea.y,
        contentArea.w,
        contentArea.h + navArea.h
      };
      flushSmfMonitorDirty(fullArea);
    } else if (currentApp == APP_MP3 && (mp3StaticDirty || mp3VisualDirty)) {
      drawMp3();
    } else if (needPartialUpdate) {
      drawHeaderStatusApp();
      needPartialUpdate = false;
    }
  }
}
