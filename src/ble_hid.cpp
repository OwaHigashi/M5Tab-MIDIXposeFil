// BLE HID host implementation.  Uses the BLE library bundled with the
// arduino-esp32 core (which is NimBLE-backed on ESP32-P4/Tab5), so this
// compiles the same way as on every other ESP32 variant.
#include "ble_hid.h"

#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLESecurity.h>
#include <BLEUUID.h>
#include "nvs_flash.h"

// Standard HID-over-GATT UUIDs.
static const BLEUUID kHidService((uint16_t)0x1812);
static const BLEUUID kReportChar((uint16_t)0x2A4D);
static const BLEUUID kBootKbInputChar((uint16_t)0x2A22);

static BLE_HID_CALLBACK g_cb = nullptr;
static BT_STATUS g_status = BT_UNINITIALIZED;

static BLEClient* g_client = nullptr;
static BLEAdvertisedDevice* g_target = nullptr;
static volatile bool g_doConnect = false;
static volatile bool g_doScan = false;
static uint32_t g_lastScanRestart = 0;

// ---- Notification handler ----
static void onNotify(BLERemoteCharacteristic* chr,
                     uint8_t* data, size_t len, bool /*isNotify*/)
{
  if (g_cb) g_cb(data, len);
}

// ---- Client callbacks ----
class HidClientCb : public BLEClientCallbacks {
  void onConnect(BLEClient* /*c*/) override {
    Serial.println("[BLE_HID] client connected");
  }
  void onDisconnect(BLEClient* /*c*/) override {
    Serial.println("[BLE_HID] client disconnected");
    g_status = BT_DISCONNECTED;
    g_doScan = true;
  }
};

// ---- Scan callbacks ----
// Optional MAC-direct connect target.  If set, any advertiser that matches
// this address is accepted regardless of whether it advertises the HID UUID
// — useful for peripherals that hide their HID service in the scan response
// or omit it entirely.  Zeroed = disabled (UUID/appearance filter only).
static uint8_t g_targetMac[6] = {0};
static bool    g_targetMacSet = false;

static bool macEquals(const uint8_t* a, const uint8_t* b) {
  for (int i = 0; i < 6; i++) if (a[i] != b[i]) return false;
  return true;
}

class HidScanCb : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    // --- Verbose log of every advertisement we see, deduped per MAC.  This
    // is how you figure out whether a given pedal is visible over BLE at
    // all; each MAC is re-logged at most once every 5 seconds.
    static uint8_t  lastLogMac[8][6] = {{0}};
    static uint32_t lastLog[8]       = {0};
    static uint8_t  lastLogIdx       = 0;
    uint32_t now = millis();
    BLEAddress addr = dev.getAddress();
    uint8_t raw[6];
    memcpy(raw, addr.getNative(), 6);

    int slot = -1;
    for (int i = 0; i < 8; i++) {
      if (macEquals(lastLogMac[i], raw)) { slot = i; break; }
    }
    bool emit;
    if (slot < 0) {
      slot = lastLogIdx;
      lastLogIdx = (lastLogIdx + 1) & 7;
      memcpy(lastLogMac[slot], raw, 6);
      emit = true;
    } else {
      emit = (now - lastLog[slot]) > 5000;
    }
    if (emit) {
      lastLog[slot] = now;
      String name = dev.getName().c_str();
      int uuids = dev.haveServiceUUID() ? dev.getServiceUUIDCount() : 0;
      uint16_t ap = dev.haveAppearance() ? dev.getAppearance() : 0;
      Serial.printf("[BLE_SCAN] %s  rssi=%d  name='%s'  uuids=%d  appearance=0x%04x\n",
                    addr.toString().c_str(), dev.getRSSI(), name.c_str(),
                    uuids, ap);
      for (int i = 0; i < uuids; i++) {
        Serial.printf("            svc[%d] = %s\n",
                      i, dev.getServiceUUID(i).toString().c_str());
      }
    }

    // --- Match logic ---
    bool match = false;
    const char* reason = "";

    // 1) explicit MAC target always wins.
    if (g_targetMacSet) {
      uint8_t peer[6];
      memcpy(peer, addr.getNative(), 6);
      if (macEquals(peer, g_targetMac)) { match = true; reason = "mac"; }
    }
    // 2) advertiser exposes the 0x1812 HID service UUID.
    if (!match && dev.haveServiceUUID()) {
      for (int i = 0; i < dev.getServiceUUIDCount(); i++) {
        if (dev.getServiceUUID(i).equals(kHidService)) {
          match = true; reason = "svc"; break;
        }
      }
    }
    // 3) advertiser uses a HID appearance code (0x03C0..0x03C4).
    if (!match && dev.haveAppearance()) {
      uint16_t ap = dev.getAppearance();
      if ((ap & 0xFFC0) == 0x03C0) { match = true; reason = "ap"; }
    }
    if (!match) return;

    Serial.printf("[BLE_HID] pedal candidate (%s): %s name='%s'\n",
                  reason, addr.toString().c_str(), dev.getName().c_str());

    BLEDevice::getScan()->stop();
    if (g_target) { delete g_target; g_target = nullptr; }
    g_target = new BLEAdvertisedDevice(dev);
    g_doConnect = true;
    g_status = BT_CONNECTING;
  }
};

static HidClientCb s_clientCb;
static HidScanCb   s_scanCb;

// ---- Connect + subscribe ----
// Walks the HID service characteristics and registers for notify on every one
// that supports it.  For a keyboard pedal that's the Boot Keyboard Input
// Report (0x2A22) and/or one of the 0x2A4D Report characteristics.
static bool connectAndSubscribe() {
  if (!g_target) return false;
  Serial.printf("[BLE_HID] connecting to %s\n",
                g_target->getAddress().toString().c_str());

  if (!g_client) {
    g_client = BLEDevice::createClient();
    g_client->setClientCallbacks(&s_clientCb);
  }
  if (!g_client->connect(g_target)) {
    Serial.println("[BLE_HID] connect() failed");
    return false;
  }
  g_client->setMTU(256);

  BLERemoteService* hid = g_client->getService(kHidService);
  if (!hid) {
    Serial.println("[BLE_HID] peer has no 0x1812 HID service");
    g_client->disconnect();
    return false;
  }

  // Enumerate every characteristic under the HID service and subscribe to all
  // notifiables.  HID-over-GATT requires an encrypted link; touching a
  // secured characteristic kicks off bonding in NimBLE.
  auto* chars = hid->getCharacteristics();  // fetches from peer on first call
  int subscribed = 0;
  if (chars) {
    for (auto& kv : *chars) {
      BLERemoteCharacteristic* c = kv.second;
      if (!c || !c->canNotify()) continue;
      c->setAuth(ESP_GATT_AUTH_REQ_MITM);  // Bluedroid; NimBLE ignores
      c->registerForNotify(onNotify);
      subscribed++;
      Serial.printf("[BLE_HID] subscribed %s\n",
                    c->getUUID().toString().c_str());
    }
  }
  if (subscribed == 0) {
    Serial.println("[BLE_HID] no notifiable HID characteristics");
    g_client->disconnect();
    return false;
  }

  g_status = BT_CONNECTED;
  return true;
}

// ---- Public API ----
bool ble_hid_begin(const char* localName, BLE_HID_CALLBACK cb) {
  if (g_status != BT_UNINITIALIZED) return true;
  g_cb = cb;

  // NVS is needed for bond storage.  No-op if already initialised.
  esp_err_t nvs = nvs_flash_init();
  if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }

  BLEDevice::init(localName ? localName : "M5Tab5-MIDI");

  // Just-works bonding: HID pedals generally do not have a display or
  // keyboard, so NoInputNoOutput / no-MITM is the correct capability set.
  auto* sec = new BLESecurity();
  sec->setAuthenticationMode(true /*bond*/, false /*mitm*/, true /*sc*/);
  sec->setCapability(ESP_IO_CAP_NONE);

  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(&s_scanCb, false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(90);

  g_status = BT_DISCONNECTED;
  ble_hid_start_scan();
  return true;
}

void ble_hid_start_scan() {
  if (g_status == BT_CONNECTED || g_status == BT_CONNECTING) return;
  Serial.println("[BLE_HID] start scan");
  g_status = BT_SCANNING;
  // IMPORTANT: the 2-arg start(duration, is_continue) BLOCKS the caller
  // (on NimBLE the 2-arg overload waits on an internal semaphore — with
  // duration=0 that is forever).  Pass an explicit nullptr completion
  // callback to pick the non-blocking 3-arg overload instead.
  void (*noCb)(BLEScanResults) = nullptr;
  BLEDevice::getScan()->start(0, noCb, false);
  g_lastScanRestart = millis();
}

BT_STATUS ble_hid_status() { return g_status; }

void ble_hid_service() {
  if (g_doConnect) {
    g_doConnect = false;
    if (!connectAndSubscribe()) {
      g_status = BT_DISCONNECTED;
      g_doScan = true;
    }
  }
  if (g_doScan && g_status == BT_DISCONNECTED) {
    g_doScan = false;
    ble_hid_start_scan();
  }
  // If a scan went idle without finding anything, nudge it after 30 s.
  if (g_status == BT_SCANNING && (millis() - g_lastScanRestart) > 30000) {
    BLEDevice::getScan()->stop();
    g_status = BT_DISCONNECTED;
    ble_hid_start_scan();
  }
}

// ---- Async init helper ----
// BLEDevice::init() can take a long time to complete over the ESP-Hosted
// transport to the C6, and it runs heavy constructors for NimBLE.  Running
// it from setup() freezes the UI until init returns.  Defer the whole init
// to a short-lived worker task so the main loop stays responsive.
struct BeginArgs { const char* name; BLE_HID_CALLBACK cb; };
static void ble_init_task(void* pv) {
  auto* args = (BeginArgs*)pv;
  ble_hid_begin(args->name, args->cb);
  delete args;
  vTaskDelete(nullptr);
}
void ble_hid_begin_async(const char* localName, BLE_HID_CALLBACK cb) {
  auto* args = new BeginArgs{ localName, cb };
  xTaskCreatePinnedToCore(ble_init_task, "ble_init", 8192, args, 4, nullptr, 0);
}

void ble_hid_forget_bonds() {
  nvs_flash_erase();
  nvs_flash_init();
  Serial.println("[BLE_HID] bonds cleared — reboot recommended");
}

void ble_hid_set_target_mac(const uint8_t* mac6) {
  if (!mac6) {
    g_targetMacSet = false;
    Serial.println("[BLE_HID] MAC target cleared");
    return;
  }
  memcpy(g_targetMac, mac6, 6);
  g_targetMacSet = true;
  Serial.printf("[BLE_HID] MAC target set: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac6[0], mac6[1], mac6[2], mac6[3], mac6[4], mac6[5]);
}
