#pragma once

#include <M5Unified.h>
#include <AudioOutput.h>

class AudioOutputM5Speaker : public AudioOutput {
 public:
  explicit AudioOutputM5Speaker(m5::Speaker_Class* speaker,
                                uint8_t virtual_channel = 0)
      : _speaker(speaker), _virtualChannel(virtual_channel) {}

  bool begin(void) override { return true; }

  bool ConsumeSample(int16_t sample[2]) override {
    if (_bufferIndex + 1 < kBufferSamples) {
      _buffers[_activeBuffer][_bufferIndex++] = sample[0];
      _buffers[_activeBuffer][_bufferIndex++] = sample[1];
      return true;
    }
    // Buffer full. M5.Speaker queues at most two chunks per channel and
    // playRaw() blocks (sleeping) until a slot frees up. Don't hand it a
    // third one: report "not consumed" so the decoder returns to its task,
    // which sleeps a tick and retries. Keeps the decoder mutex from being
    // held while blocked and lets the UI's stop/start get in promptly.
    if (_speaker && _speaker->isPlaying(_virtualChannel) >= 2) return false;
    flush();
    return false;
  }

  void flush(void) override {
    if (_bufferIndex == 0 || _speaker == nullptr) return;
    _speaker->playRaw(_buffers[_activeBuffer], _bufferIndex, hertz, true, 1,
                      _virtualChannel);
    _lastPlayedBuffer = _activeBuffer;
    _activeBuffer = (_activeBuffer + 1) % 3;
    _bufferIndex = 0;
  }

  bool stop(void) override {
    flush();
    if (_speaker) _speaker->stop(_virtualChannel);
    return true;
  }

  const int16_t* getBuffer(void) const { return _buffers[_lastPlayedBuffer]; }

  size_t getBufferLength(void) const { return kBufferSamples; }

 private:
  static constexpr size_t kBufferSamples = 1536;

  m5::Speaker_Class* _speaker = nullptr;
  uint8_t _virtualChannel = 0;
  int16_t _buffers[3][kBufferSamples] = {};
  size_t _bufferIndex = 0;
  size_t _activeBuffer = 0;
  size_t _lastPlayedBuffer = 0;
};
