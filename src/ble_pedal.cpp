#include "ble_pedal.h"

#include "sdkconfig.h"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <host/ble_hs.h>
#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <host/ble_uuid.h>
#include <host/ble_store.h>
#include <host/ble_hs_adv.h>
#include <host/util/util.h>
#include "esp32-hal-hosted.h"

extern "C" void ble_store_config_init(void);

// HID over GATT.
static const uint16_t UUID_HID_SERVICE   = 0x1812;
static const uint16_t UUID_HID_REPORT    = 0x2A4D;
static const uint16_t UUID_BOOT_KB_INPUT = 0x2A22;
static const uint16_t UUID_CCCD          = 0x2902;

// Scanning is bounded on purpose (see the header): one window per pairing
// request, then the radio goes quiet again.
static const int32_t  PAIR_SCAN_MS       = 30000;
// One directed connect attempt. The controller does the listening; the host
// sees nothing until the pedal actually answers.
static const int32_t  CONNECT_ATTEMPT_MS = 30000;
// Link up but encryption / discovery never finished: drop it and retry.
static const uint32_t SETUP_TIMEOUT_MS   = 15000;
static const uint32_t RETRY_MIN_MS       = 500;
static const uint32_t RETRY_MAX_MS       = 30000;

static BlePedalReportCb s_reportCb = nullptr;
static volatile BlePedalState s_state = BLE_PEDAL_OFF;
static volatile uint32_t s_stateSince = 0;
static volatile bool     s_synced = false;
static volatile bool     s_enabled = true;
static uint8_t           s_ownAddrType = BLE_OWN_ADDR_PUBLIC;

static ble_addr_t        s_target = {};
static volatile bool     s_haveTarget = false;
static volatile bool     s_scanRequested = false;
static volatile bool     s_scanConnects = false;   // pairing scan (true) or just a survey
static volatile uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;
static volatile uint32_t s_nextConnectAt = 0;
static volatile uint8_t  s_setupFails = 0;         // consecutive, drives the retry backoff
static volatile int      s_lastErr = 0;            // last NimBLE rc / status that broke a step
static const char* volatile s_lastErrAt = "";

// Notifying characteristics of the HID service, filled by discovery.
struct HidChr {
  uint16_t valHandle;
  uint16_t cccdHandle;
  uint16_t uuid16;
  uint8_t  props;
};
static HidChr   s_chrs[12];
static int      s_chrCount = 0;
static int      s_subIdx = 0;
static int      s_subscribed = 0;
static uint16_t s_svcStart = 0;
static uint16_t s_svcEnd = 0;

// What the last scan saw, for blePedalPrintSeen().
struct Seen {
  ble_addr_t addr;
  int8_t     rssi;
  bool       hid;
  uint16_t   appearance;
  uint16_t   mfg;          // company ID from manufacturer data, 0xFFFF if none
  char       name[20];
};
static Seen         s_seen[16];
static volatile int s_seenCount = 0;

static int gapEvent(struct ble_gap_event* ev, void* arg);

static void setState(BlePedalState st) {
  s_state = st;
  s_stateSince = millis();
}

static void noteError(const char* where, int rc) {
  s_lastErrAt = where;
  s_lastErr = rc;
}

static void retryAfter(uint32_t ms) {
  s_nextConnectAt = millis() + ms;
}

// Back to IDLE after a link that never became usable; each consecutive
// failure doubles the wait so a pedal that can't be set up isn't hammered.
static void scheduleRetryAfterFailure() {
  uint32_t wait = RETRY_MIN_MS << (s_setupFails < 6 ? s_setupFails : 6);
  if (wait > RETRY_MAX_MS) wait = RETRY_MAX_MS;
  if (s_setupFails < 255) s_setupFails = s_setupFails + 1;
  retryAfter(wait);
}

static void dropLink(const char* where, int rc) {
  noteError(where, rc);
  uint16_t conn = s_conn;
  if (conn != BLE_HS_CONN_HANDLE_NONE) ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
}

// ---- GATT: find the HID service, its reports, their CCCDs; subscribe ----

static void subscribeNext(uint16_t conn);

static int subscribeCb(uint16_t conn, const struct ble_gatt_error* err,
                       struct ble_gatt_attr* /*attr*/, void* /*arg*/) {
  if (err->status == 0) s_subscribed++;
  else                  noteError("cccd", err->status);
  subscribeNext(conn);
  return 0;
}

static void subscribeNext(uint16_t conn) {
  static const uint8_t kNotifyOn[2] = { 0x01, 0x00 };
  while (s_subIdx < s_chrCount) {
    const HidChr& c = s_chrs[s_subIdx++];
    bool isInput = (c.uuid16 == UUID_HID_REPORT || c.uuid16 == UUID_BOOT_KB_INPUT);
    if (!isInput || !(c.props & BLE_GATT_CHR_PROP_NOTIFY) || c.cccdHandle == 0) continue;
    int rc = ble_gattc_write_flat(conn, c.cccdHandle, kNotifyOn, sizeof(kNotifyOn),
                                  subscribeCb, nullptr);
    if (rc == 0) return;  // subscribeCb continues the chain
    noteError("cccd-write", rc);
  }
  if (s_subscribed > 0) {
    s_setupFails = 0;
    setState(BLE_PEDAL_READY);
  } else {
    dropLink("no-input-report", 0);
  }
}

static int dscCb(uint16_t conn, const struct ble_gatt_error* err,
                 uint16_t /*chrValHandle*/, const struct ble_gatt_dsc* dsc, void* /*arg*/) {
  if (err->status == 0 && dsc != nullptr) {
    if (ble_uuid_u16(&dsc->uuid.u) == UUID_CCCD) {
      // The CCCD belongs to the closest characteristic value below it.
      int owner = -1;
      for (int i = 0; i < s_chrCount; ++i) {
        if (s_chrs[i].valHandle < dsc->handle &&
            (owner < 0 || s_chrs[i].valHandle > s_chrs[owner].valHandle)) {
          owner = i;
        }
      }
      if (owner >= 0) s_chrs[owner].cccdHandle = dsc->handle;
    }
    return 0;
  }
  if (err->status == BLE_HS_EDONE) {
    s_subIdx = 0;
    s_subscribed = 0;
    subscribeNext(conn);
  } else {
    dropLink("disc-dsc", err->status);
  }
  return 0;
}

static int chrCb(uint16_t conn, const struct ble_gatt_error* err,
                 const struct ble_gatt_chr* chr, void* /*arg*/) {
  if (err->status == 0 && chr != nullptr) {
    if (s_chrCount < (int)(sizeof(s_chrs) / sizeof(s_chrs[0]))) {
      HidChr& c = s_chrs[s_chrCount++];
      c.valHandle  = chr->val_handle;
      c.cccdHandle = 0;
      c.uuid16     = ble_uuid_u16(&chr->uuid.u);
      c.props      = chr->properties;
    }
    return 0;
  }
  if (err->status == BLE_HS_EDONE) {
    int rc = ble_gattc_disc_all_dscs(conn, s_svcStart, s_svcEnd, dscCb, nullptr);
    if (rc != 0) dropLink("disc-dsc-start", rc);
  } else {
    dropLink("disc-chr", err->status);
  }
  return 0;
}

static int svcCb(uint16_t conn, const struct ble_gatt_error* err,
                 const struct ble_gatt_svc* svc, void* /*arg*/) {
  if (err->status == 0 && svc != nullptr) {
    s_svcStart = svc->start_handle;
    s_svcEnd   = svc->end_handle;
    return 0;
  }
  if (err->status == BLE_HS_EDONE && s_svcEnd != 0) {
    int rc = ble_gattc_disc_all_chrs(conn, s_svcStart, s_svcEnd, chrCb, nullptr);
    if (rc != 0) dropLink("disc-chr-start", rc);
  } else {
    dropLink("no-hid-service", err->status);
  }
  return 0;
}

static void startDiscovery(uint16_t conn) {
  s_chrCount = 0;
  s_svcStart = 0;
  s_svcEnd   = 0;
  ble_uuid16_t hid;
  hid.u.type = BLE_UUID_TYPE_16;
  hid.value  = UUID_HID_SERVICE;
  int rc = ble_gattc_disc_svc_by_uuid(conn, &hid.u, svcCb, nullptr);
  if (rc != 0) dropLink("disc-svc-start", rc);
}

// ---- GAP ----

static void startConnect() {
  int rc = ble_gap_connect(s_ownAddrType, &s_target, CONNECT_ATTEMPT_MS, nullptr,
                           gapEvent, nullptr);
  if (rc == 0) {
    setState(BLE_PEDAL_CONNECTING);
  } else {
    noteError("connect", rc);
    retryAfter(RETRY_MAX_MS / 10);
  }
}

static void rememberSeen(const struct ble_gap_disc_desc* d, bool hid,
                         const struct ble_hs_adv_fields& f) {
  const uint8_t* name = f.name;
  uint8_t nameLen = f.name_len;
  int n = s_seenCount;
  int slot = -1;
  for (int i = 0; i < n; ++i) {
    if (ble_addr_cmp(&s_seen[i].addr, &d->addr) == 0) { slot = i; break; }
  }
  if (slot < 0) {
    if (n < (int)(sizeof(s_seen) / sizeof(s_seen[0]))) {
      slot = n;
      s_seenCount = n + 1;
    } else {
      // Full: a crowded room shouldn't push the pedal out, so the weakest
      // non-HID entry makes way for anything stronger or HID.
      int weakest = -1;
      for (int i = 0; i < n; ++i) {
        if (s_seen[i].hid) continue;
        if (weakest < 0 || s_seen[i].rssi < s_seen[weakest].rssi) weakest = i;
      }
      if (weakest < 0 || (!hid && s_seen[weakest].rssi >= d->rssi)) return;
      slot = weakest;
    }
    s_seen[slot].addr = d->addr;
    s_seen[slot].hid = false;
    s_seen[slot].appearance = 0;
    s_seen[slot].mfg = 0xFFFF;
    s_seen[slot].name[0] = '\0';
  }
  s_seen[slot].rssi = d->rssi;
  if (hid) s_seen[slot].hid = true;
  if (f.appearance_is_present) s_seen[slot].appearance = f.appearance;
  if (f.mfg_data != nullptr && f.mfg_data_len >= 2) {
    s_seen[slot].mfg = (uint16_t)(f.mfg_data[0] | (f.mfg_data[1] << 8));
  }
  if (name != nullptr && nameLen > 0) {
    size_t len = nameLen < sizeof(s_seen[slot].name) - 1 ? nameLen : sizeof(s_seen[slot].name) - 1;
    memcpy(s_seen[slot].name, name, len);
    s_seen[slot].name[len] = '\0';
  }
}

static void onAdvert(const struct ble_gap_disc_desc* d) {
  struct ble_hs_adv_fields f;
  if (ble_hs_adv_parse_fields(&f, d->data, d->length_data) != 0) return;

  bool hid = false;
  for (int i = 0; i < f.num_uuids16; ++i) {
    if (ble_uuid_u16(&f.uuids16[i].u) == UUID_HID_SERVICE) hid = true;
  }
  // Appearance category 15 = HID (keyboard, mouse, generic, ...).
  if (f.appearance_is_present && (f.appearance & 0xFFC0) == 0x03C0) hid = true;
  rememberSeen(d, hid, f);

  if (!hid || !s_scanConnects) return;
  if (d->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
      d->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {
    return;  // not connectable
  }
  if (ble_gap_disc_cancel() != 0) return;
  s_target = d->addr;
  s_haveTarget = true;
  s_setupFails = 0;
  setState(BLE_PEDAL_IDLE);
  startConnect();
}

static int gapEvent(struct ble_gap_event* ev, void* /*arg*/) {
  switch (ev->type) {
    case BLE_GAP_EVENT_DISC:
      onAdvert(&ev->disc);
      return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
      if (s_state == BLE_PEDAL_SCANNING) setState(BLE_PEDAL_IDLE);
      return 0;

    case BLE_GAP_EVENT_CONNECT:
      if (ev->connect.status == 0) {
        s_conn = ev->connect.conn_handle;
        setState(BLE_PEDAL_SETUP);
        // HID over GATT wants an encrypted link before anything else: this
        // pairs and bonds the first time, re-encrypts with the stored key
        // afterwards.
        int rc = ble_gap_security_initiate(ev->connect.conn_handle);
        if (rc != 0) dropLink("security", rc);
      } else {
        // Attempt timed out or was cancelled: go round again.
        s_conn = BLE_HS_CONN_HANDLE_NONE;
        retryAfter(RETRY_MIN_MS);
        setState(BLE_PEDAL_IDLE);
      }
      return 0;

    case BLE_GAP_EVENT_DISCONNECT: {
      bool wasReady = (s_state == BLE_PEDAL_READY);
      s_conn = BLE_HS_CONN_HANDLE_NONE;
      if (wasReady) retryAfter(RETRY_MIN_MS);
      else          scheduleRetryAfterFailure();
      setState(BLE_PEDAL_IDLE);
      return 0;
    }

    case BLE_GAP_EVENT_ENC_CHANGE:
      if (ev->enc_change.status == 0) {
        startDiscovery(ev->enc_change.conn_handle);
      } else {
        // Our stored key no longer matches (pedal reset or paired elsewhere):
        // drop the bond so the next attempt pairs from scratch.
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(ev->enc_change.conn_handle, &desc) == 0) {
          ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        dropLink("encrypt", ev->enc_change.status);
      }
      return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
      // The pedal wants to pair again although we hold a bond for it.
      struct ble_gap_conn_desc desc;
      if (ble_gap_conn_find(ev->repeat_pairing.conn_handle, &desc) == 0) {
        ble_store_util_delete_peer(&desc.peer_id_addr);
      }
      return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
      uint8_t buf[16];
      uint16_t len = OS_MBUF_PKTLEN(ev->notify_rx.om);
      if (len > sizeof(buf)) len = sizeof(buf);
      if (len > 0 && os_mbuf_copydata(ev->notify_rx.om, 0, len, buf) == 0 && s_reportCb) {
        s_reportCb(buf, len);
      }
      return 0;
    }

    default:
      // Includes the pedal's connection-parameter requests: 0 accepts them.
      return 0;
  }
}

// ---- Bring-up ----

static void onSync() {
  ble_hs_util_ensure_addr(0);
  if (ble_hs_id_infer_auto(0, &s_ownAddrType) != 0) s_ownAddrType = BLE_OWN_ADDR_PUBLIC;

  // The bond store is the only persistence: whoever is bonded is the pedal.
  ble_addr_t peers[CONFIG_BT_NIMBLE_MAX_BONDS];
  int count = 0;
  if (ble_store_util_bonded_peers(peers, &count, CONFIG_BT_NIMBLE_MAX_BONDS) == 0 && count > 0) {
    s_target = peers[0];
    s_haveTarget = true;
  } else if (!s_haveTarget && !s_scanRequested) {
    // Nothing bonded yet: one pairing window, then quiet until asked again.
    s_scanConnects = true;
    s_scanRequested = true;
  }
  s_conn = BLE_HS_CONN_HANDLE_NONE;
  retryAfter(0);
  s_synced = true;
  setState(BLE_PEDAL_IDLE);
}

static void onReset(int reason) {
  noteError("host-reset", reason);
  s_synced = false;
  s_conn = BLE_HS_CONN_HANDLE_NONE;
  setState(BLE_PEDAL_STARTING);
}

static void hostTask(void* /*arg*/) {
  nimble_port_run();
  nimble_port_freertos_deinit();
}

// hostedInitBLE() talks to the C6 over SDIO and can take seconds, so it runs
// here instead of in setup().
static void initTask(void* /*arg*/) {
  if (!hostedInitBLE()) {
    noteError("hosted-init", -1);
    setState(BLE_PEDAL_FAILED);
    vTaskDelete(nullptr);
    return;
  }
  esp_err_t err = nimble_port_init();
  if (err != ESP_OK) {
    noteError("nimble-init", (int)err);
    setState(BLE_PEDAL_FAILED);
    vTaskDelete(nullptr);
    return;
  }
  ble_hs_cfg.reset_cb = onReset;
  ble_hs_cfg.sync_cb = onSync;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
  // Just-works bonding: a pedal has neither display nor keypad.
  ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
  ble_hs_cfg.sm_bonding = 1;
  ble_hs_cfg.sm_mitm = 0;
  ble_hs_cfg.sm_sc = 1;
  ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  ble_store_config_init();
  nimble_port_freertos_init(hostTask);
  vTaskDelete(nullptr);
}

// ---- Public API ----

void blePedalBegin(BlePedalReportCb cb) {
  if (s_state != BLE_PEDAL_OFF) return;
  s_reportCb = cb;
  setState(BLE_PEDAL_STARTING);
  if (xTaskCreatePinnedToCore(initTask, "ble_init", 8192, nullptr, 4, nullptr, 0) != pdPASS) {
    noteError("init-task", -1);
    setState(BLE_PEDAL_FAILED);
  }
}

static void startScan() {
  struct ble_gap_disc_params p = {};
  p.itvl = 160;              // 100 ms
  p.window = 80;             // 50 ms
  p.filter_policy = BLE_HCI_SCAN_FILT_NO_WL;
  p.limited = 0;
  p.passive = 0;             // active: scan responses carry the device name
  // The controller's duplicate filter keys on the address alone and drops the
  // scan response (the part with the name); rememberSeen() dedups instead.
  p.filter_duplicates = 0;
  s_seenCount = 0;
  int rc = ble_gap_disc(s_ownAddrType, PAIR_SCAN_MS, &p, gapEvent, nullptr);
  if (rc == 0) setState(BLE_PEDAL_SCANNING);
  else         noteError("scan", rc);
}

void blePedalService(uint32_t now) {
  BlePedalState st = s_state;
  if (!s_synced || st == BLE_PEDAL_OFF || st == BLE_PEDAL_STARTING || st == BLE_PEDAL_FAILED) return;

  if (!s_enabled) {
    // Unwind to IDLE and stay there.
    if (st == BLE_PEDAL_SCANNING) {
      ble_gap_disc_cancel();
      setState(BLE_PEDAL_IDLE);
    } else if (st == BLE_PEDAL_CONNECTING) {
      ble_gap_conn_cancel();
    } else if (st == BLE_PEDAL_SETUP || st == BLE_PEDAL_READY) {
      uint16_t conn = s_conn;
      if (conn != BLE_HS_CONN_HANDLE_NONE) ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
    }
    return;
  }

  if (s_scanRequested) {
    // A scan needs the radio to itself: unwind whatever is in progress and
    // start from IDLE on a later pass.
    if (st == BLE_PEDAL_CONNECTING) {
      ble_gap_conn_cancel();
    } else if (st == BLE_PEDAL_SETUP || st == BLE_PEDAL_READY) {
      uint16_t conn = s_conn;
      if (conn != BLE_HS_CONN_HANDLE_NONE) ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
    } else if (st == BLE_PEDAL_IDLE) {
      s_scanRequested = false;
      startScan();
    }
    return;
  }

  if (st == BLE_PEDAL_IDLE) {
    if (s_haveTarget && (int32_t)(now - s_nextConnectAt) >= 0) startConnect();
  } else if (st == BLE_PEDAL_SETUP && now - s_stateSince > SETUP_TIMEOUT_MS) {
    if (s_conn == BLE_HS_CONN_HANDLE_NONE) {
      setState(BLE_PEDAL_IDLE);
    } else {
      dropLink("setup-timeout", 0);
      s_stateSince = now;  // one terminate per timeout window
    }
  }
}

void blePedalSetEnabled(bool on) {
  if (on && !s_enabled && s_synced && !s_haveTarget) {
    // Same as a fresh start with nothing bonded: one pairing window.
    s_scanConnects = true;
    s_scanRequested = true;
  }
  if (!on) s_scanRequested = false;
  s_enabled = on;
}

bool blePedalHasPedal() { return s_haveTarget; }

void blePedalPair() {
  if (s_state == BLE_PEDAL_OFF || s_state == BLE_PEDAL_FAILED) return;
  s_haveTarget = false;
  if (s_synced) ble_store_clear();
  s_scanConnects = true;
  s_scanRequested = true;
}

void blePedalScanOnly() {
  if (s_state == BLE_PEDAL_OFF || s_state == BLE_PEDAL_FAILED) return;
  s_scanConnects = false;
  s_scanRequested = true;
}

void blePedalForget() {
  s_haveTarget = false;
  if (s_synced) ble_store_clear();
  s_scanRequested = false;
  BlePedalState st = s_state;
  if (st == BLE_PEDAL_SCANNING) {
    ble_gap_disc_cancel();
    setState(BLE_PEDAL_IDLE);
  } else if (st == BLE_PEDAL_CONNECTING) {
    ble_gap_conn_cancel();
  } else if (st == BLE_PEDAL_SETUP || st == BLE_PEDAL_READY) {
    uint16_t conn = s_conn;
    if (conn != BLE_HS_CONN_HANDLE_NONE) ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
  }
}

BlePedalState blePedalState() { return s_state; }

const char* blePedalStateLabel(BlePedalState s) {
  switch (s) {
    case BLE_PEDAL_OFF:        return "OFF";
    case BLE_PEDAL_STARTING:   return "STARTING";
    case BLE_PEDAL_FAILED:     return "FAILED";
    case BLE_PEDAL_IDLE:       return "IDLE";
    case BLE_PEDAL_SCANNING:   return "SCANNING";
    case BLE_PEDAL_CONNECTING: return "CONNECTING";
    case BLE_PEDAL_SETUP:      return "SETUP";
    case BLE_PEDAL_READY:      return "READY";
  }
  return "?";
}

static void formatAddr(const ble_addr_t& a, char* out, size_t outSize) {
  snprintf(out, outSize, "%02X:%02X:%02X:%02X:%02X:%02X/%u",
           a.val[5], a.val[4], a.val[3], a.val[2], a.val[1], a.val[0], (unsigned)a.type);
}

void blePedalDescribe(char* out, size_t outSize) {
  char peer[24] = "-";
  if (s_haveTarget) formatAddr(s_target, peer, sizeof(peer));
  snprintf(out, outSize, "ble=%s ble_peer=%s ble_err=%s:%d",
           blePedalStateLabel(s_state), peer,
           s_lastErrAt[0] ? s_lastErrAt : "-", s_lastErr);
}

void blePedalPrintSeen(Print& out) {
  int n = s_seenCount;
  for (int i = 0; i < n; ++i) {
    char addr[24];
    formatAddr(s_seen[i].addr, addr, sizeof(addr));
    out.printf("BLE SEEN %s rssi=%d hid=%d app=%04x mfg=%04x name=%s\n",
               addr, (int)s_seen[i].rssi, s_seen[i].hid ? 1 : 0,
               (unsigned)s_seen[i].appearance, (unsigned)s_seen[i].mfg,
               s_seen[i].name[0] ? s_seen[i].name : "-");
  }
  out.printf("OK BLE SEEN %d\n", n);
}
