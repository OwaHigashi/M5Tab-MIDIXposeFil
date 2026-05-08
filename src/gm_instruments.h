#pragma once
#include <stdint.h>

// GM Level 1 instrument names (Program 0..127), as exposed by M5 Unit MIDI
// (SAM2695). Matches the table used in M5Core2-MIDIXposeFilBTUM so the SRC
// playback mode stays consistent across devices.
static const char* const kGmInstrumentNames[128] = {
  "Grand Piano 1", "Bright Piano 2", "El Grd Piano 3", "Honky-Tonk Piano",
  "Electric Piano 1", "Electric Piano 2", "Harpsichord", "Clavi",
  "Celesta", "Glockenspiel", "Music Box", "Vibraphone",
  "Marimba", "Xylophone", "Tubular Bells", "Santur",
  "Drawbar Organ", "Percussive Organ", "Rock Organ", "Church Organ",
  "Reed Organ", "Accordion French", "Harmonica", "Tango Accordion",
  "Acoustic Guitar Nylon", "Acoustic Guitar Steel", "Acoustic Guitar Jazz", "Acoustic Guitar Clean",
  "Acoustic Guitar Muted", "Overdriven Guitar", "Distortion Guitar", "Guitar Harmonics",
  "Acoustic Bass", "Finger Bass", "Picked Bass", "Fretless Bass",
  "Slap Bass 1", "Slap Bass 2", "Synth Bass 1", "Synth Bass 2",
  "Violin", "Viola", "Cello", "Contrabass",
  "Tremolo Strings", "Pizzicato Strings", "Orchestral Harp", "Timpani",
  "String Ensemble 1", "String Ensemble 2", "Synth Strings 1", "Synth Strings 2",
  "Choir Aahs", "Voice Oohs", "Synth Voice", "Orchestra Hit",
  "Trumpet", "Trombone", "Tuba", "Muted Trumpet",
  "French Horn", "Brass Section", "Synth Brass 1", "Synth Brass 2",
  "Soprano Sax", "Alto Sax", "Tenor Sax", "Baritone Sax",
  "Oboe", "English Horn", "Bassoon", "Clarinet",
  "Piccolo", "Flute", "Recorder", "Pan Flute",
  "Blown Bottle", "Shakuhachi", "Whistle", "Ocarina",
  "Lead 1 Square", "Lead 2 Sawtooth", "Lead 3 Calliope", "Lead 4 Chiff",
  "Lead 5 Charang", "Lead 6 Voice", "Lead 7 Fifths", "Lead 8 Bass + Lead",
  "Pad 1 Fantasia", "Pad 2 Warm", "Pad 3 PolySynth", "Pad 4 Choir",
  "Pad 5 Bowed", "Pad 6 Metallic", "Pad 7 Halo", "Pad 8 Sweep",
  "FX 1 Rain", "FX 2 Soundtrack", "FX 3 Crystal", "FX 4 Atmosphere",
  "FX 5 Brightness", "FX 6 Goblins", "FX 7 Echoes", "FX 8 Sci-Fi",
  "Sitar", "Banjo", "Shamisen", "Koto",
  "Kalimba", "Bag Pipe", "Fiddle", "Shanai",
  "Tinkle Bell", "Agogo", "Steel Drums", "Woodblock",
  "Taiko Drum", "Melodic Tom", "Synth Drum", "Reverse Cymbal",
  "Guitar Fret Noise", "Breath Noise", "Seashore", "Bird Tweet",
  "Telephone Ring", "Helicopter", "Applause", "Gunshot"
};

inline const char* gmInstrumentName(uint8_t program) {
  return (program < 128) ? kGmInstrumentNames[program] : "Unknown";
}

// GM Level 1 drum-kit Program Change targets for MIDI Channel 10.
// Program numbers are the canonical GM-compatible values; SAM2695 maps
// missing variants down to "Standard Kit", so picking any of these is safe.
struct DrumKitEntry { uint8_t program; const char* name; };
static const DrumKitEntry kGmDrumKits[] = {
  {  0, "Standard Kit"   },
  {  8, "Room Kit"       },
  { 16, "Power Kit"      },
  { 24, "Electronic Kit" },
  { 25, "TR-808 Kit"     },
  { 32, "Jazz Kit"       },
  { 40, "Brush Kit"      },
  { 48, "Orchestra Kit"  },
  { 56, "Sound FX Kit"   },
};
static constexpr int kGmDrumKitCount =
    (int)(sizeof(kGmDrumKits) / sizeof(kGmDrumKits[0]));

// Returns the human-readable kit name for the given program number, or
// "Standard Kit" if not exactly one of the catalog entries (matches SAM2695
// fallback behaviour).
inline const char* gmDrumKitName(uint8_t program) {
  for (int i = 0; i < kGmDrumKitCount; ++i) {
    if (kGmDrumKits[i].program == program) return kGmDrumKits[i].name;
  }
  return "Standard Kit";
}

// Returns the index in kGmDrumKits[] for a given program, or 0 (Standard) if
// not found.
inline int gmDrumKitIndex(uint8_t program) {
  for (int i = 0; i < kGmDrumKitCount; ++i) {
    if (kGmDrumKits[i].program == program) return i;
  }
  return 0;
}
