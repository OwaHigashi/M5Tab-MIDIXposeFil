// BLE HID foot pedal (page-turner style) as a second wireless input.
//
// Tab5's radio is the ESP32-C6 behind ESP-Hosted, BLE only. This talks to the
// NimBLE host directly instead of going through the Arduino BLE classes, and
// it never scans in the background: the radio only scans for a bounded time
// when a pairing is requested, and otherwise just waits for the bonded pedal
// with a directed connect. (The earlier BLE pedal code kept a near-continuous
// scan running with no pedal present and reset the board within minutes.)
#pragma once

#include <Arduino.h>

enum BlePedalState : uint8_t {
  BLE_PEDAL_OFF,         // stack never started
  BLE_PEDAL_STARTING,    // bringing up the C6 link and NimBLE
  BLE_PEDAL_FAILED,      // bring-up failed; stays down until reboot
  BLE_PEDAL_IDLE,        // up, no bonded pedal and no pairing in progress
  BLE_PEDAL_SCANNING,    // pairing scan running
  BLE_PEDAL_CONNECTING,  // waiting for the pedal to show up
  BLE_PEDAL_SETUP,       // link up: encrypting, discovering, subscribing
  BLE_PEDAL_READY,       // input reports are flowing
};

// One HID input report exactly as the pedal notified it (no Report ID byte).
// Runs on the NimBLE host task: no UI, no Serial.
typedef void (*BlePedalReportCb)(const uint8_t* data, size_t len);

// Start the stack on a worker task; returns immediately. Once it is up it
// reconnects to the bonded pedal on its own, or runs one pairing window if
// nothing is bonded yet. A second call is a no-op.
void blePedalBegin(BlePedalReportCb cb);

// Drive reconnects and timeouts. Call from loop(); cheap when idle.
void blePedalService(uint32_t now);

// Off: drop the link and stop reconnecting (bond and stack stay). On: resume.
void blePedalSetEnabled(bool on);

// A pedal is bonded (whether or not it is connected right now).
bool blePedalHasPedal();

// Forget the current pedal and take the first HID peripheral that advertises
// within the pairing window.
void blePedalPair();

// Scan for the same window without connecting, to see what is in range.
void blePedalScanOnly();

// Drop the bond and stop reconnecting.
void blePedalForget();

BlePedalState blePedalState();
const char* blePedalStateLabel(BlePedalState s);

// "state=... peer=..." for STATUS.
void blePedalDescribe(char* out, size_t outSize);

// Devices seen by the last scan, one per line.
void blePedalPrintSeen(Print& out);
