// BLE HID host for a page-turner style foot pedal.
// Replaces the M5Core2 Bluedroid L2CAP implementation — the ESP32-P4 has no
// classic BT, so the pedal must advertise as a BLE HID (service 0x1812).
// The wire format used by these pedals is the standard 8-byte HID keyboard
// report (mod, reserved, key1..key6), identical to what the Classic code
// stripped out of the 0xa1-0x01 prefixed L2CAP payload.
#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

enum BT_STATUS {
  BT_UNINITIALIZED,
  BT_DISCONNECTED,
  BT_SCANNING,
  BT_CONNECTING,
  BT_CONNECTED,
};

// Report is exactly `len` bytes as delivered by the pedal.  For a keyboard
// pedal this is the 8-byte boot-keyboard report.
typedef void (*BLE_HID_CALLBACK)(const uint8_t* report, size_t len);

// Initialise the stack.  Safe to call once from setup().
bool ble_hid_begin(const char* localName, BLE_HID_CALLBACK cb);

// Same as ble_hid_begin but runs the init on a short-lived worker task so
// setup() doesn't block while the BLE stack negotiates with the C6 radio.
void ble_hid_begin_async(const char* localName, BLE_HID_CALLBACK cb);

// Start scanning for HID advertisers.  Called automatically after begin.
void ble_hid_start_scan();

// Current connection status (for the on-screen indicator).
BT_STATUS ble_hid_status();

// Call regularly from loop() — drives reconnect and scan restart.
void ble_hid_service();

// Forget any paired devices — useful when pairing a new pedal.
void ble_hid_forget_bonds();

// Optional: restrict scanning to a specific peripheral MAC.  Pass any 6-byte
// address to match only that device; pass nullptr to clear the filter and
// fall back to matching by HID service UUID / appearance.
void ble_hid_set_target_mac(const uint8_t* mac6);
