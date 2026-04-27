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
