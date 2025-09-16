// ESP32 Bluetooth A2DP Sink with 3-Band EQ to PCM5102 (I2S)
#include <Arduino.h>
#include "BluetoothA2DPSink.h"
#include <driver/i2s.h>
#include <math.h>
#include <Preferences.h>
#include <math.h>

#define I2S_NUM I2S_NUM_0
// Overall output makeup gain in dB (applied after EQ). Increase to taste.
#define OUTPUT_GAIN_DB 3.0f

// UART for display ESP32
#define UART_RX 16
#define UART_TX 17
HardwareSerial SerialUART(1);

BluetoothA2DPSink a2dp_sink;
Preferences prefs;

char lastTitle[64] = "?";
char lastArtist[64] = "?";
bool playing = false;

// Pairing PIN (4-6 digits). Change if you want a different code.
static const int kPairingPin = 1968;

// Simple flag to auto-confirm pairing PIN shortly after boot or when requested
volatile bool allowPinConfirm = true;

// 3-Band EQ Implementation
enum EQBand { BAND_BASS = 0, BAND_MID = 1, BAND_TREBLE = 2 };
float eqGains[3] = {0.0f, 0.0f, 0.0f}; // Bass, Mid, Treble gains in dB
const float eqMinGain = -12.0f;
const float eqMaxGain = 12.0f;
int currentEQBand = 0;
uint16_t currentSampleRate = 44100; // track actual A2DP sample rate

// EQ processing helpers
float preGainLin = 1.0f; // pre-attenuation to preserve headroom when boosting
float postGainLin = 1.0f; // post-EQ makeup gain for overall loudness
inline float dbToLin(float dB) { return powf(10.0f, dB / 20.0f); }
inline bool eqIsFlat() {
  const float th = 0.05f; // ~0.05dB considered zero
  return fabsf(eqGains[0]) < th && fabsf(eqGains[1]) < th && fabsf(eqGains[2]) < th;
}

// Optional gentle soft-clipper to catch any residual peaks without harsh distortion
inline float softClip(float x) {
  // Piecewise soft clip: linear in -0.5..0.5, quadratic kinks outside, then clamp
  const float limit = 0.98f;
  if (x > limit) return limit + 0.5f * (x - limit) / (1.0f + (x - limit) * (x - limit));
  if (x < -limit) return -limit + 0.5f * (x + limit) / (1.0f + (x + limit) * (x + limit));
  return x;
}

// Simple RBJ Biquad filter implementation
struct BiquadRBJ {
  float b0=1.0f, b1=0.0f, b2=0.0f, a1=0.0f, a2=0.0f; // normalized (a0=1)
  float z1=0.0f, z2=0.0f; // Direct Form I (transposed) state

  inline float process(float x) {
    float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }

  inline void reset() { z1 = 0.0f; z2 = 0.0f; }

  static inline float dB2A(float dB) { return powf(10.0f, dB / 40.0f); }

  void setPeaking(float fs, float f0, float Q, float dBgain) {
    if (Q <= 0) Q = 0.707f;
    const float A = dB2A(dBgain);
    const float w0 = 2.0f * (float)M_PI * (f0 / fs);
    const float alpha = sinf(w0) / (2.0f * Q);
    const float cosw0 = cosf(w0);

    float b0n = 1.0f + alpha * A;
    float b1n = -2.0f * cosw0;
    float b2n = 1.0f - alpha * A;
    float a0n = 1.0f + alpha / A;
    float a1n = -2.0f * cosw0;
    float a2n = 1.0f - alpha / A;

    b0 = b0n / a0n;
    b1 = b1n / a0n;
    b2 = b2n / a0n;
    a1 = a1n / a0n;
    a2 = a2n / a0n;
    reset();
  }

  void setLowShelf(float fs, float f0, float S, float dBgain) {
    if (S <= 0) S = 1.0f;
    const float A = dB2A(dBgain);
    const float w0 = 2.0f * (float)M_PI * (f0 / fs);
    const float cosw0 = cosf(w0);
    const float sinw0 = sinf(w0);
    const float alpha = sinw0 / 2.0f * sqrtf((A + 1.0f / A) * (1.0f / S - 1.0f) + 2.0f);
    const float twoSqrtAalpha = 2.0f * sqrtf(A) * alpha;

    float b0n =      A*((A+1.0f) - (A-1.0f)*cosw0 + twoSqrtAalpha);
    float b1n =  2.0f * A*((A-1.0f) - (A+1.0f)*cosw0);
    float b2n =      A*((A+1.0f) - (A-1.0f)*cosw0 - twoSqrtAalpha);
    float a0n =          (A+1.0f) + (A-1.0f)*cosw0 + twoSqrtAalpha;
    float a1n = -2.0f *     ((A-1.0f) + (A+1.0f)*cosw0);
    float a2n =          (A+1.0f) + (A-1.0f)*cosw0 - twoSqrtAalpha;

    b0 = b0n / a0n;
    b1 = b1n / a0n;
    b2 = b2n / a0n;
    a1 = a1n / a0n;
    a2 = a2n / a0n;
    reset();
  }

  void setHighShelf(float fs, float f0, float S, float dBgain) {
    if (S <= 0) S = 1.0f;
    const float A = dB2A(dBgain);
    const float w0 = 2.0f * (float)M_PI * (f0 / fs);
    const float cosw0 = cosf(w0);
    const float sinw0 = sinf(w0);
    const float alpha = sinw0 / 2.0f * sqrtf((A + 1.0f / A) * (1.0f / S - 1.0f) + 2.0f);
    const float twoSqrtAalpha = 2.0f * sqrtf(A) * alpha;

    float b0n =      A*((A+1.0f) + (A-1.0f)*cosw0 + twoSqrtAalpha);
    float b1n = -2.0f * A*((A-1.0f) + (A+1.0f)*cosw0);
    float b2n =      A*((A+1.0f) + (A-1.0f)*cosw0 - twoSqrtAalpha);
    float a0n =          (A+1.0f) - (A-1.0f)*cosw0 + twoSqrtAalpha;
    float a1n =  2.0f *     ((A-1.0f) - (A+1.0f)*cosw0);
    float a2n =          (A+1.0f) - (A-1.0f)*cosw0 - twoSqrtAalpha;

    b0 = b0n / a0n;
    b1 = b1n / a0n;
    b2 = b2n / a0n;
    a1 = a1n / a0n;
    a2 = a2n / a0n;
    reset();
  }
};

// Filters for each band (stereo)
BiquadRBJ bassL, bassR;     // Low shelf ~100 Hz
BiquadRBJ midL, midR;       // Peaking ~1 kHz
BiquadRBJ trebleL, trebleR; // High shelf ~8 kHz

void updateEQFilters() {
  // Configure filters with explicit sample rate (RBJ cookbook)
  const double fs = (double)currentSampleRate;
  bassL.setLowShelf(fs, 100.0, 1.0, eqGains[BAND_BASS]);
  bassR.setLowShelf(fs, 100.0, 1.0, eqGains[BAND_BASS]);
  midL.setPeaking(fs, 1000.0, 1.0, eqGains[BAND_MID]);
  midR.setPeaking(fs, 1000.0, 1.0, eqGains[BAND_MID]);
  trebleL.setHighShelf(fs, 8000.0, 1.0, eqGains[BAND_TREBLE]);
  trebleR.setHighShelf(fs, 8000.0, 1.0, eqGains[BAND_TREBLE]);
  
  // Compute pre-attenuation headroom equal to the maximum positive boost
  float maxBoost = eqGains[0];
  if (eqGains[1] > maxBoost) maxBoost = eqGains[1];
  if (eqGains[2] > maxBoost) maxBoost = eqGains[2];
  // Limit headroom to at most 6 dB; rely on soft-clipper for rare transients beyond this
  float headroomDb = (maxBoost > 0.0f) ? fminf(maxBoost, 6.0f) : 0.0f;
  preGainLin = dbToLin(-headroomDb);
  // Fixed makeup gain after EQ for overall volume
  postGainLin = dbToLin(OUTPUT_GAIN_DB);

  Serial.printf("[EQ] Updated filters - Bass: %.1fdB, Mid: %.1fdB, Treble: %.1fdB\n", 
                eqGains[BAND_BASS], eqGains[BAND_MID], eqGains[BAND_TREBLE]);
}

// Update when A2DP reports a new sample rate (e.g., 44100 or 48000)
void onSampleRateChanged(uint16_t rate) {
  currentSampleRate = rate ? rate : 44100;
  Serial.printf("[A2DP] Sample rate changed: %u Hz\n", currentSampleRate);
  updateEQFilters();
}

void saveEQSettings() {
  prefs.begin("eq", false);
  for (int i = 0; i < 3; i++) {
    char key[8];
    snprintf(key, sizeof(key), "band%d", i);
    prefs.putFloat(key, eqGains[i]);
  }
  prefs.end();
}

void loadEQSettings() {
  prefs.begin("eq", true);
  for (int i = 0; i < 3; i++) {
    char key[8];
    snprintf(key, sizeof(key), "band%d", i);
    eqGains[i] = prefs.getFloat(key, 0.0f);
  }
  prefs.end();
  updateEQFilters();
}

void sendDisplayMetadata() {
  SerialUART.print("T=");
  SerialUART.print(lastTitle);
  SerialUART.print("\n");
  SerialUART.print("A=");
  SerialUART.print(lastArtist);
  SerialUART.print("\n");
  SerialUART.print("P=");
  SerialUART.print(playing ? "1" : "0");
  SerialUART.print("\n");
}

void sendEQValues() {
  // Send current EQ values to display
  for (int i = 0; i < 3; i++) {
    SerialUART.print("EQ=");
    SerialUART.print(i);
    SerialUART.print(",");
    SerialUART.print(eqGains[i], 1);
    SerialUART.print("\n");
  }
  SerialUART.print("EQBAND=");
  SerialUART.print(currentEQBand);
  SerialUART.print("\n");
}

void btAudioStateCallback(esp_a2d_audio_state_t state, void * obj) {
  playing = (state == ESP_A2D_AUDIO_STATE_STARTED);
  sendDisplayMetadata();
}

void btMetadataCallback(uint8_t id, const uint8_t *text) {
  if (id == 0x01) { // Title
    strncpy(lastTitle, (const char*)text, sizeof(lastTitle) - 1);
    lastTitle[sizeof(lastTitle) - 1] = '\0';
  } else if (id == 0x02) { // Artist
    strncpy(lastArtist, (const char*)text, sizeof(lastArtist) - 1);
    lastArtist[sizeof(lastArtist) - 1] = '\0';
  }
  sendDisplayMetadata();
}

// Audio data callback with EQ processing
void IRAM_ATTR audioDataCallback(const uint8_t *data, uint32_t len) {
  // If EQ is effectively flat, skip processing for efficiency
  if (eqIsFlat()) {
    return;
  }
  // Convert byte data to 16-bit samples
  int16_t *samples = (int16_t*)data;
  uint32_t sampleCount = len / 4; // 4 bytes per stereo sample (2x 16-bit)
  
  for (uint32_t i = 0; i < sampleCount; i++) {
    // Extract left and right channels
    float leftSample = samples[i * 2] / 32768.0f;     // Convert to float [-1.0, 1.0]
    float rightSample = samples[i * 2 + 1] / 32768.0f;
    // Pre-attenuation headroom to avoid clipping with boosts
    leftSample *= preGainLin;
    rightSample *= preGainLin;
    
  // Apply 3-band EQ processing
  leftSample = trebleL.process(midL.process(bassL.process(leftSample)));
  rightSample = trebleR.process(midR.process(bassR.process(rightSample)));
  // Post-EQ makeup gain for overall loudness
  leftSample *= postGainLin;
  rightSample *= postGainLin;
    
    // Optional soft-clip safety
    leftSample = softClip(leftSample);
    rightSample = softClip(rightSample);
    // Convert back to 16-bit and clamp
    samples[i * 2] = (int16_t)(fmax(-32768, fmin(32767, leftSample * 32767.0f)));
    samples[i * 2 + 1] = (int16_t)(fmax(-32768, fmin(32767, rightSample * 32767.0f)));
  }
}


// Button GPIOs
#define BTN_PLAY_PAUSE 32
#define BTN_NEXT 33
#define BTN_PREV 23
#define BTN_EQ 5
#define BTN_BRIGHTNESS 4  // New brightness button

unsigned long lastPlayBtnMillis = 0;
unsigned long lastNextBtnMillis = 0;
unsigned long lastPrevBtnMillis = 0;
unsigned long lastEqBtnMillis = 0;
unsigned long lastBrightnessBtnMillis = 0;
const unsigned long debounceMs = 250;

// EQ mode state
bool eqMode = false;

void setup() {
  Serial.begin(115200);
  SerialUART.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
  delay(200);
  Serial.println("[A2DP] ESP32 Bluetooth to I2S with 3-Band EQ");

  pinMode(BTN_PLAY_PAUSE, INPUT_PULLUP);
  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(BTN_PREV, INPUT_PULLUP);
  pinMode(BTN_EQ, INPUT_PULLUP);
  pinMode(BTN_BRIGHTNESS, INPUT_PULLUP);

  // Load EQ settings from preferences
  loadEQSettings();

  // I2S pin config (adjust as needed for your wiring)
  i2s_pin_config_t my_pins = {
    .bck_io_num = 21,
    .ws_io_num = 25,
    .data_out_num = 22,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  a2dp_sink.set_pin_config(my_pins);

  // I2S config for PCM5102
  i2s_config_t my_i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 12,
    .dma_buf_len = 256,
    .use_apll = true,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  a2dp_sink.set_i2s_config(my_i2s_config);

  a2dp_sink.set_volume(127); // 0-127: set to max; adjust phone volume as needed
  a2dp_sink.set_on_audio_state_changed_post(btAudioStateCallback, nullptr);
  a2dp_sink.set_avrc_metadata_callback(btMetadataCallback);
  // Enable audio processing AND keep I2S output active (let library manage I2S)
  a2dp_sink.set_stream_reader(audioDataCallback, true);
  // Keep EQ filters aligned with the actual negotiated sample rate
  a2dp_sink.set_sample_rate_callback(onSampleRateChanged);

  // Enable auto-reconnect with limited retries and a small delay before first attempt
  a2dp_sink.set_auto_reconnect(true, 10);     // try up to 10 times
  a2dp_sink.set_reconnect_delay(1500);        // 1.5s delay before reconnect

  // Activate pairing PIN. The library will issue PIN callbacks we can confirm.
  a2dp_sink.activate_pin_code(true);

  // Start sink (device will be discoverable/connectable)
  a2dp_sink.start("ESP32 EQ PCM5102");
  Serial.println("[A2DP] Sink with EQ started. Pair and play from your phone.");
}

void loop() {
  // Button debounce
  static bool lastPlay = HIGH, lastNext = HIGH, lastPrev = HIGH, lastEq = HIGH, lastBrightness = HIGH;
  bool curPlay = digitalRead(BTN_PLAY_PAUSE);
  bool curNext = digitalRead(BTN_NEXT);
  bool curPrev = digitalRead(BTN_PREV);
  bool curEq = digitalRead(BTN_EQ);
  bool curBrightness = digitalRead(BTN_BRIGHTNESS);
  unsigned long now = millis();

  // If the stack requested a PIN recently, auto-confirm our fixed code
  static unsigned long lastPinCheck = 0;
  if (allowPinConfirm && (now - lastPinCheck > 250)) {
    lastPinCheck = now;
    int requested = a2dp_sink.pin_code();
    if (requested != 0) {
      // If the phone expects a numeric entry, pass our fixed code; otherwise just confirm
      a2dp_sink.confirm_pin_code(kPairingPin);
      // Stop spamming confirmations
      allowPinConfirm = false;
      Serial.printf("[BT] PIN requested -> confirming with %d\n", kPairingPin);
    } else {
      // Try a generic confirm in case of simple yes/no pairing
      a2dp_sink.confirm_pin_code();
    }
  }

  // --- UART receive: look for commands from display ---
  if (SerialUART.available()) {
    String line = SerialUART.readStringUntil('\n');
    if (line.startsWith("EQMODE=")) {
      if (line.endsWith("1")) {
        eqMode = true;
      } else if (line.endsWith("0")) {
        eqMode = false;
      }
    } else if (line.startsWith("EQ=")) {
      // EQ band adjustment: "EQ=BAND,GAIN" (e.g., "EQ=0,3.5")
      int commaPos = line.indexOf(',');
      if (commaPos > 0) {
        int band = line.substring(3, commaPos).toInt();
        float gain = line.substring(commaPos + 1).toFloat();
        if (band >= 0 && band < 3 && gain >= eqMinGain && gain <= eqMaxGain) {
          eqGains[band] = gain;
          updateEQFilters();
          saveEQSettings();
          Serial.printf("[EQ] Updated band %d to %.1fdB\n", band, gain);
        }
      }
    }
  }

  if (eqMode) {
    // --- EQ MODE BUTTONS ---
    // Play: cycle band
    if (curPlay == LOW && lastPlay == HIGH && now - lastPlayBtnMillis > debounceMs) {
      lastPlayBtnMillis = now;
      currentEQBand = (currentEQBand + 1) % 3; // Cycle through 0, 1, 2
      SerialUART.print("BTN=PLAY\n");
      Serial.printf("[EQ] Switched to band %d\n", currentEQBand);
    }
    // Next: +1dB
    if (curNext == LOW && lastNext == HIGH && now - lastNextBtnMillis > debounceMs) {
      lastNextBtnMillis = now;
      eqGains[currentEQBand] += 1.0f;
      if (eqGains[currentEQBand] > eqMaxGain) eqGains[currentEQBand] = eqMaxGain;
      updateEQFilters();
      saveEQSettings();
      SerialUART.print("BTN=NEXT\n");
      Serial.printf("[EQ] Band %d: %.1fdB\n", currentEQBand, eqGains[currentEQBand]);
    }
    // Prev: -1dB
    if (curPrev == LOW && lastPrev == HIGH && now - lastPrevBtnMillis > debounceMs) {
      lastPrevBtnMillis = now;
      eqGains[currentEQBand] -= 1.0f;
      if (eqGains[currentEQBand] < eqMinGain) eqGains[currentEQBand] = eqMinGain;
      updateEQFilters();
      saveEQSettings();
      SerialUART.print("BTN=PREV\n");
      Serial.printf("[EQ] Band %d: %.1fdB\n", currentEQBand, eqGains[currentEQBand]);
    }
    // EQ: exit EQ mode
    if (curEq == LOW && lastEq == HIGH && now - lastEqBtnMillis > debounceMs) {
      lastEqBtnMillis = now;
      SerialUART.print("BTN=EQ\n");
      SerialUART.print("EQMODE=0\n");
      eqMode = false;
    }
    lastPlay = curPlay;
    lastNext = curNext;
    lastPrev = curPrev;
    lastEq = curEq;
    lastBrightness = curBrightness;
    return;
  }

  // --- NORMAL MODE BUTTONS ---
  if (curPlay == LOW && lastPlay == HIGH && now - lastPlayBtnMillis > debounceMs) {
    if (playing) {
      a2dp_sink.pause();
    } else {
      a2dp_sink.play();
    }
    lastPlayBtnMillis = now;
  }
  if (curNext == LOW && lastNext == HIGH && now - lastNextBtnMillis > debounceMs) {
    a2dp_sink.next();
    lastNextBtnMillis = now;
  }
  if (curPrev == LOW && lastPrev == HIGH && now - lastPrevBtnMillis > debounceMs) {
    a2dp_sink.previous();
    lastPrevBtnMillis = now;
  }
  // EQ: enter EQ mode
  if (curEq == LOW && lastEq == HIGH && now - lastEqBtnMillis > debounceMs) {
    lastEqBtnMillis = now;
    SerialUART.print("BTN=EQ\n");
    SerialUART.print("EQMODE=1\n");
    eqMode = true;
    currentEQBand = 0; // Start with bass
    sendEQValues(); // Send current EQ settings to display
  }
  // Brightness: cycle brightness levels
  if (curBrightness == LOW && lastBrightness == HIGH && now - lastBrightnessBtnMillis > debounceMs) {
    lastBrightnessBtnMillis = now;
    SerialUART.print("BTN=DIM\n");
    Serial.println("[BRIGHTNESS] Button pressed");
  }
  lastPlay = curPlay;
  lastNext = curNext;
  lastPrev = curPrev;
  lastEq = curEq;
  lastBrightness = curBrightness;
}
