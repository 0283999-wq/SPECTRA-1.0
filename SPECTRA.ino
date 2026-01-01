/*
  SPECTRA - ESP32 GC9A01 + DFPlayer Mini MP3 firmware (single-file sketch)
  ----------------------------------------------------------------------
  Mini README:
  - Board: ESP32 Dev Module (NodeMCU style)
  - Required libraries (install via Library Manager):
      * Adafruit GFX Library
      * Adafruit GC9A01A
      * DFRobotDFPlayerMini (by DFRobot)
      * Preferences (built into ESP32 core)
  - Hardware: GC9A01 240x240 SPI (pins fixed per spec), DFPlayer Mini on HW UART, touch + mechanical buttons, ADC battery.
  - No WiFi/Bluetooth used. Rotation left at default (0). Pins unchanged.
  - Background bitmap header: vinyl_assets.h placed alongside this sketch.

  vinyl_assets.h aliasing notes (critical):
  - If your header exports image_data_Image, the alias maps VINYL_UI_BITMAP to image_data_Image automatically.
  - If your header exports vinyl_ui_bitmap, the alias maps VINYL_UI_BITMAP to vinyl_ui_bitmap automatically.
  - If neither exists, you'll see: #error "vinyl_ui bitmap symbol not found; export image_data_Image or vinyl_ui_bitmap".
  Use VINYL_UI_BITMAP consistently; no manual edits needed if header provides one of the above.
*/

#include <Arduino.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <DFRobotDFPlayerMini.h>
#include <time.h>
#include <pgmspace.h> // keep compatibility with converter-generated headers using PROGMEM
#include "vinyl_assets.h"

// ---------------------------------------------------------------------------
// vinyl_assets.h aliasing (see comments above)
#if defined(vinyl_ui_bitmap)
  #define VINYL_UI_BITMAP vinyl_ui_bitmap
  #if defined(vinyl_ui_bitmap_WIDTH)
    #define VINYL_UI_WIDTH vinyl_ui_bitmap_WIDTH
    #define VINYL_UI_HEIGHT vinyl_ui_bitmap_HEIGHT
  #else
    #define VINYL_UI_WIDTH 240
    #define VINYL_UI_HEIGHT 240
  #endif
#elif defined(image_data_Image)
  #define VINYL_UI_BITMAP image_data_Image
  #if defined(image_data_Image_WIDTH)
    #define VINYL_UI_WIDTH image_data_Image_WIDTH
    #define VINYL_UI_HEIGHT image_data_Image_HEIGHT
  #else
    #define VINYL_UI_WIDTH 240
    #define VINYL_UI_HEIGHT 240
  #endif
#else
  #error "vinyl_ui bitmap symbol not found; export image_data_Image or vinyl_ui_bitmap"
#endif
// If your header exports image_data_Image, the alias maps VINYL_UI_BITMAP to image_data_Image automatically.
// If it exports vinyl_ui_bitmap, the alias maps VINYL_UI_BITMAP to vinyl_ui_bitmap automatically.
// If neither exists, you'll see: #error 'vinyl_ui bitmap symbol not found; export image_data_Image or vinyl_ui_bitmap'.

// ---------------------------------------------------------------------------
// Pin map (do not change)
static const uint8_t PIN_TFT_CS   = 17;
static const uint8_t PIN_TFT_DC   = 16;
static const uint8_t PIN_TFT_RST  = 5;
static const uint8_t PIN_TFT_MOSI = 23;
static const uint8_t PIN_TFT_SCK  = 18;

static const uint8_t PIN_DF_RX = 27; // ESP32 RX (to DFPlayer TX)
static const uint8_t PIN_DF_TX = 26; // ESP32 TX (to DFPlayer RX)

static const uint8_t PIN_PLAY  = 32; // touch, active HIGH
static const uint8_t PIN_NEXT  = 33; // touch, active HIGH
static const uint8_t PIN_PREV  = 25; // touch, active HIGH
static const uint8_t PIN_VOL_DN = 21; // mechanical, INPUT_PULLUP active LOW
static const uint8_t PIN_VOL_UP = 22; // mechanical, INPUT_PULLUP active LOW

static const uint8_t PIN_BATT = 35; // ADC

// ---------------------------------------------------------------------------
// Forward declarations
class ButtonManager;
class DFPlayerWrapper;
class BatteryManager;
class ClockManager;
class DisplayManager;

// ---------------------------------------------------------------------------
// Helpers
static const uint16_t COLOR_BG = 0x0000;       // black
static const uint16_t COLOR_CYAN = 0x07FF;     // cyan/greenish
static const uint16_t COLOR_AMBER = 0xFEC0;    // amber/orange
static const uint16_t COLOR_RED = 0xF800;      // red
static const uint16_t COLOR_GRAY = 0x4208;     // dark gray
static const uint16_t COLOR_WHITE = 0xFFFF;

enum class UiState { MAIN_UI, BT_UI, SET_TIME, ERROR_UI };

struct ButtonEvent {
  uint8_t id;
  bool pressed = false;
  bool released = false;
  bool tapped = false;
  bool held = false;
};

// ---------------------------------------------------------------------------
// Button management with debounce + hold
class Button {
public:
  Button(uint8_t pin, bool activeHigh, uint32_t debounceMs, uint32_t holdMs, uint8_t id)
    : _pin(pin), _activeHigh(activeHigh), _debounceMs(debounceMs), _holdMs(holdMs), _id(id) {}

  void begin(bool usePullup=false) {
    pinMode(_pin, usePullup ? INPUT_PULLUP : INPUT);
    _lastStable = _readRaw();
    _lastReading = _lastStable;
    _lastChangeMs = millis();
  }

  ButtonEvent update() {
    ButtonEvent ev; ev.id = _id;
    bool raw = _readRaw();
    uint32_t now = millis();

    if (raw != _lastReading) {
      _lastReading = raw;
      _lastChangeMs = now;
    }

    if ((now - _lastChangeMs) >= _debounceMs) {
      if (_lastStable != _lastReading) {
        _lastStable = _lastReading;
        if (_lastStable) {
          ev.pressed = true;
          _pressTime = now;
        } else {
          ev.released = true;
          if (_pressed && !_heldFlag) ev.tapped = true;
          _pressed = false;
          _heldFlag = false;
        }
      }
    }

    if (_lastStable && !_heldFlag && _pressed && (now - _pressTime >= _holdMs)) {
      _heldFlag = true;
      ev.held = true;
    }

    // latch pressed flag
    if (ev.pressed) _pressed = true;

    return ev;
  }

  bool isPressed() const { return _pressed; }

private:
  bool _readRaw() const {
    int v = digitalRead(_pin);
    return _activeHigh ? (v == HIGH) : (v == LOW);
  }

  uint8_t _pin;
  bool _activeHigh;
  uint32_t _debounceMs;
  uint32_t _holdMs;
  uint8_t _id;
  bool _lastStable = false;
  bool _lastReading = false;
  uint32_t _lastChangeMs = 0;
  uint32_t _pressTime = 0;
  bool _pressed = false;
  bool _heldFlag = false;
};

class ButtonManager {
public:
  enum ButtonId { BTN_PLAY, BTN_NEXT, BTN_PREV, BTN_VOL_DN, BTN_VOL_UP };

  void begin() {
    _buttons[BTN_PLAY] = new Button(PIN_PLAY, true, 30, 3000, BTN_PLAY);
    _buttons[BTN_NEXT] = new Button(PIN_NEXT, true, 30, 800, BTN_NEXT);
    _buttons[BTN_PREV] = new Button(PIN_PREV, true, 30, 800, BTN_PREV);
    _buttons[BTN_VOL_DN] = new Button(PIN_VOL_DN, false, 35, 1200, BTN_VOL_DN);
    _buttons[BTN_VOL_UP] = new Button(PIN_VOL_UP, false, 35, 1200, BTN_VOL_UP);

    _buttons[BTN_PLAY]->begin(false);
    _buttons[BTN_NEXT]->begin(false);
    _buttons[BTN_PREV]->begin(false);
    _buttons[BTN_VOL_DN]->begin(true);
    _buttons[BTN_VOL_UP]->begin(true);
  }

  void update(void (*handler)(const ButtonEvent&)) {
    for (int i = 0; i < BTN_COUNT; ++i) {
      ButtonEvent ev = _buttons[i]->update();
      if (ev.pressed || ev.released || ev.tapped || ev.held) handler(ev);
    }
  }

  bool bothVolumePressed() const {
    return _buttons[BTN_VOL_DN]->isPressed() && _buttons[BTN_VOL_UP]->isPressed();
  }

private:
  static const int BTN_COUNT = 5;
  Button* _buttons[BTN_COUNT];
};

// ---------------------------------------------------------------------------
// DFPlayer wrapper
class DFPlayerWrapper {
public:
  bool begin() {
    Serial1.begin(9600, SERIAL_8N1, PIN_DF_RX, PIN_DF_TX);
    if (!_df.begin(Serial1)) {
      _error = true;
      return false;
    }
    _df.setTimeOut(500);
    _df.EQ(DFPLAYER_EQ_NORMAL);
    _df.outputDevice(DFPLAYER_DEVICE_SD);
    setVolume(_volume);
    return true;
  }

  void setVolume(uint8_t v) {
    _volume = constrain(v, (uint8_t)0, (uint8_t)30);
    _df.volume(_volume);
  }

  uint8_t volume() const { return _volume; }

  void playTrack(uint16_t idx) {
    _track = idx;
    _df.playMp3Folder(idx);
    _playing = true;
    _trackStartMs = millis();
  }

  void playPauseToggle() {
    if (_playing) {
      _df.pause();
      _playing = false;
    } else {
      _df.start();
      _playing = true;
    }
  }

  void next() {
    _df.next();
    _track++;
    _trackStartMs = millis();
    _playing = true;
  }

  void prev() {
    _df.previous();
    if (_track > 1) _track--;
    _trackStartMs = millis();
    _playing = true;
  }

  bool isPlaying() const { return _playing; }
  uint16_t track() const { return _track; }
  bool hasError() const { return _error; }
  uint32_t elapsedMs() const { return millis() - _trackStartMs; }

private:
  DFRobotDFPlayerMini _df;
  bool _playing = false;
  bool _error = false;
  uint8_t _volume = 18;
  uint16_t _track = 1;
  uint32_t _trackStartMs = 0;
};

// ---------------------------------------------------------------------------
// Battery manager
class BatteryManager {
public:
  void begin() {
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_BATT, ADC_11db);
    _lastUpdate = millis();
    _filtered = readRawVoltage();
  }

  void update() {
    uint32_t now = millis();
    if (now - _lastUpdate < 200) return;
    _lastUpdate = now;
    float v = readRawVoltage();
    _filtered = _filtered * 0.85f + v * 0.15f;
  }

  float voltage() const { return _filtered * _gain + _offset; }

  uint8_t percent() const {
    float v = voltage();
    // simple piecewise map 3.0-4.2V
    if (v <= 3.0f) return 0;
    if (v >= 4.2f) return 100;
    return (uint8_t)constrain((v - 3.0f) * (100.0f / 1.2f), 0.0f, 100.0f);
  }

  uint16_t color() const {
    uint8_t p = percent();
    if (p > 40) return COLOR_CYAN; // greenish
    if (p > 15) return COLOR_AMBER;
    return COLOR_RED;
  }

  void setCalibration(float gain, float offset) { _gain = gain; _offset = offset; }

private:
  float readRawVoltage() {
    int raw = analogRead(PIN_BATT);
    float v = ((float)raw / 4095.0f) * 3.3f; // ADC ref
    v *= 2.0f; // divider
    return v;
  }

  float _filtered = 0;
  float _gain = 1.0f;
  float _offset = 0.0f;
  uint32_t _lastUpdate = 0;
};

// ---------------------------------------------------------------------------
// Clock manager
class ClockManager {
public:
  void begin(Preferences& prefs) {
    parseCompileTime();
    _prefs = &prefs;
    _timeOffset = prefs.getInt("timeOffset", 0);
    _lastMillis = millis();
  }

  void update() {
    uint32_t now = millis();
    uint32_t delta = now - _lastMillis;
    _lastMillis = now;
    _current += delta / 1000;
    _millisCarry += delta % 1000;
    if (_millisCarry >= 1000) {
      _current += 1;
      _millisCarry -= 1000;
    }
  }

  void adjustMinutes(int delta) { _timeOffset += delta * 60; }
  void adjustHours(int delta) { _timeOffset += delta * 3600; }

  void save() {
    if (_prefs) _prefs->putInt("timeOffset", _timeOffset);
  }

  void getTime(uint8_t& hh, uint8_t& mm, uint8_t& ss) const {
    int32_t t = _current + _timeOffset;
    if (t < 0) t = (24*3600 + (t % (24*3600))) % (24*3600);
    t %= (24*3600);
    hh = t / 3600;
    mm = (t / 60) % 60;
    ss = t % 60;
  }

  String formatted() const {
    uint8_t h,m,s; getTime(h,m,s);
    char buf[9];
    snprintf(buf, sizeof(buf), "%02u:%02u:%02u", h,m,s);
    return String(buf);
  }

private:
  void parseCompileTime() {
    // __DATE__ format: "Mmm dd yyyy", __TIME__: "hh:mm:ss"
    const char* date = __DATE__;
    const char* time = __TIME__;
    char monthStr[4];
    int day, year;
    sscanf(date, "%3s %d %d", monthStr, &day, &year);
    int month = monthToInt(monthStr);
    int hh, mm, ss;
    sscanf(time, "%d:%d:%d", &hh, &mm, &ss);
    struct tm t = {};
    t.tm_year = year - 1900;
    t.tm_mon = month - 1;
    t.tm_mday = day;
    t.tm_hour = hh;
    t.tm_min = mm;
    t.tm_sec = ss;
    time_t compiled = mktime(&t);
    _current = compiled;
  }

  int monthToInt(const char* m) {
    static const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    const char* p = strstr(months, m);
    if (!p) return 1;
    return (p - months) / 3 + 1;
  }

  Preferences* _prefs = nullptr;
  time_t _current = 0;
  int32_t _timeOffset = 0;
  uint32_t _lastMillis = 0;
  uint32_t _millisCarry = 0;
};

// ---------------------------------------------------------------------------
// Display manager
class DisplayManager {
public:
  void begin() {
    _tft = new Adafruit_GC9A01A(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_MOSI, PIN_TFT_SCK, PIN_TFT_RST);
    _tft->begin();
    _tft->fillScreen(COLOR_BG);
    drawBackground();
    _lastOverlayMs = millis();
  }

  void drawBackground() {
    // Center the vinyl asset if its exported dimensions are not exactly 240x240.
    int16_t x = (240 - VINYL_UI_WIDTH) / 2;
    int16_t y = (240 - VINYL_UI_HEIGHT) / 2;
    _tft->fillScreen(COLOR_BG);
    _tft->drawRGBBitmap(x, y, VINYL_UI_BITMAP, VINYL_UI_WIDTH, VINYL_UI_HEIGHT);
    _backgroundDrawn = true;
  }

  void drawMainUI(const DFPlayerWrapper& player, const BatteryManager& batt, const ClockManager& clock, UiState state, bool highlight=false) {
    if (!_backgroundDrawn) drawBackground();
    uint32_t now = millis();
    if (now - _lastOverlayMs < 150 && !highlight) return;
    _lastOverlayMs = now;

    uint16_t accent = highlight ? COLOR_WHITE : COLOR_CYAN;

    // Track
    char trackStr[16];
    snprintf(trackStr, sizeof(trackStr), "TRACK %04u", player.track());
    overlayBanner(trackStr, 5, accent);

    // State + progress
    _tft->setTextColor(accent, COLOR_BG);
    _tft->setTextSize(2);
    _tft->setCursor(10, 40);
    _tft->print(player.isPlaying() ? "PLAY" : "PAUSE");

    uint32_t elapsed = player.elapsedMs();
    const uint32_t estDuration = 180000; // fallback 3 minutes
    uint32_t displayElapsed = min(elapsed, estDuration);
    uint32_t remaining = estDuration - displayElapsed;
    drawTimeField("ELAP", displayElapsed/1000, 120, 40, accent);
    drawTimeField("LEFT", remaining/1000, 120, 60, accent);

    // Volume
    drawBar("VOL", player.volume(), 30, 90, 30, accent);

    // Battery
    drawBattery(batt, 150, 90);

    // Clock
    _tft->setTextColor(COLOR_CYAN, COLOR_BG);
    _tft->setTextSize(2);
    _tft->setCursor(10, 200);
    _tft->print(clock.formatted());
  }

  void drawBluetoothUI() {
    _tft->fillScreen(COLOR_BG);
    _tft->drawCircle(120, 120, 70, COLOR_CYAN);
    _tft->fillCircle(120, 120, 50, COLOR_GRAY);
    _tft->fillTriangle(120, 40, 150, 100, 90, 100, COLOR_CYAN);
    _tft->drawTriangle(120, 200, 160, 150, 80, 150, COLOR_AMBER);
    _tft->setTextColor(COLOR_CYAN, COLOR_BG);
    _tft->setTextSize(2);
    _tft->setCursor(35, 15);
    _tft->print("BLUETOOTH MODE");
    _tft->setCursor(45, 220);
    _tft->print("AUDIO ROUTE EXTERNAL");
  }

  void drawSetTimeUI(const ClockManager& clock, bool adjustHours) {
    _tft->fillScreen(COLOR_BG);
    _tft->setTextColor(COLOR_CYAN, COLOR_BG);
    _tft->setTextSize(2);
    _tft->setCursor(40, 20);
    _tft->print("SET TIME");

    uint8_t h,m,s; clock.getTime(h,m,s);
    char buf[9]; snprintf(buf, sizeof(buf), "%02u:%02u:%02u", h,m,s);
    _tft->setTextSize(3);
    _tft->setCursor(45, 100);
    _tft->print(buf);

    _tft->drawRect(40, adjustHours ? 95 : 130, 80, 35, COLOR_AMBER);
    _tft->drawRect(130, adjustHours ? 95 : 130, 80, 35, COLOR_AMBER);
    _tft->setTextSize(1);
    _tft->setCursor(30, 200);
    _tft->print("VOL+/- adjust, NEXT swap, PLAY save");
  }

  void drawError(const char* msg) {
    _tft->fillScreen(COLOR_BG);
    _tft->setTextColor(COLOR_RED, COLOR_BG);
    _tft->setTextSize(2);
    _tft->setCursor(20, 100);
    _tft->print("PLAYER ERR");
    _tft->setTextSize(1);
    _tft->setCursor(20, 130);
    _tft->print(msg);
  }

private:
  void overlayBanner(const char* txt, int y, uint16_t color) {
    _tft->fillRect(0, y-2, 240, 18, COLOR_BG);
    _tft->setTextSize(2);
    _tft->setTextColor(color, COLOR_BG);
    _tft->setCursor(10, y);
    _tft->print("SPECTRA ");
    _tft->print(txt);
  }

  void drawTimeField(const char* label, uint32_t seconds, int x, int y, uint16_t color) {
    char buf[6];
    uint8_t mm = seconds / 60;
    uint8_t ss = seconds % 60;
    snprintf(buf, sizeof(buf), "%02u:%02u", mm, ss);
    _tft->setTextSize(1);
    _tft->setTextColor(COLOR_GRAY, COLOR_BG);
    _tft->setCursor(x, y);
    _tft->print(label);
    _tft->setTextColor(color, COLOR_BG);
    _tft->setCursor(x, y+12);
    _tft->setTextSize(2);
    _tft->print(buf);
  }

  void drawBar(const char* label, uint8_t value, int x, int y, int width, uint16_t color) {
    _tft->setTextColor(COLOR_GRAY, COLOR_BG);
    _tft->setTextSize(1);
    _tft->setCursor(x, y);
    _tft->print(label);
    _tft->drawRect(x, y+12, width, 12, COLOR_GRAY);
    int filled = map(value, 0, 30, 0, width-2);
    _tft->fillRect(x+1, y+13, filled, 10, color);
    _tft->setCursor(x + width + 5, y+12);
    _tft->setTextColor(color, COLOR_BG);
    _tft->print(value);
  }

  void drawBattery(const BatteryManager& batt, int x, int y) {
    uint8_t pct = batt.percent();
    uint16_t color = batt.color();
    _tft->setTextColor(COLOR_GRAY, COLOR_BG);
    _tft->setTextSize(1);
    _tft->setCursor(x, y);
    _tft->print("BATT");
    _tft->drawRect(x, y+12, 50, 12, COLOR_GRAY);
    int filled = map(pct, 0, 100, 0, 48);
    _tft->fillRect(x+1, y+13, filled, 10, color);
    _tft->fillRect(x+52, y+14, 4, 8, COLOR_GRAY);

    _tft->setCursor(x, y+28);
    _tft->setTextColor(color, COLOR_BG);
    char buf[16];
    snprintf(buf, sizeof(buf), "%3u%%", pct);
    _tft->print(buf);
    _tft->setCursor(x, y+40);
    snprintf(buf, sizeof(buf), "%.2fV", batt.voltage());
    _tft->print(buf);
  }

  Adafruit_GC9A01A* _tft = nullptr;
  bool _backgroundDrawn = false;
  uint32_t _lastOverlayMs = 0;
};

// ---------------------------------------------------------------------------
// Globals
Preferences prefs;
ButtonManager buttons;
DFPlayerWrapper player;
BatteryManager battery;
ClockManager clockMgr;
DisplayManager displayMgr;
UiState uiState = UiState::MAIN_UI;
bool adjustHours = true;
uint32_t btHoldStart = 0;
uint32_t overlayFlashUntil = 0;

// ---------------------------------------------------------------------------
void handleButton(const ButtonEvent& ev) {
  // BT hold detection (both volume buttons)
  if (ev.pressed && buttons.bothVolumePressed()) {
    if (btHoldStart == 0) btHoldStart = millis();
  }
  if (!buttons.bothVolumePressed()) btHoldStart = 0;

  switch (ev.id) {
    case ButtonManager::BTN_PLAY:
      if (uiState == UiState::SET_TIME) {
        if (ev.tapped || ev.held) {
          clockMgr.save();
          uiState = UiState::MAIN_UI;
          displayMgr.drawBackground();
        }
      } else {
        if (ev.held) {
          uiState = UiState::SET_TIME;
          adjustHours = true;
          displayMgr.drawSetTimeUI(clockMgr, adjustHours);
        } else if (ev.tapped) {
          player.playPauseToggle();
          overlayFlashUntil = millis() + 300;
        }
      }
      break;
    case ButtonManager::BTN_NEXT:
      if (uiState == UiState::SET_TIME) {
        adjustHours = !adjustHours;
        displayMgr.drawSetTimeUI(clockMgr, adjustHours);
      } else if (ev.tapped) {
        player.next();
        overlayFlashUntil = millis() + 300;
      }
      break;
    case ButtonManager::BTN_PREV:
      if (uiState == UiState::MAIN_UI && ev.tapped) { player.prev(); overlayFlashUntil = millis() + 300; }
      break;
    case ButtonManager::BTN_VOL_DN:
      if (uiState == UiState::SET_TIME) {
        if (ev.tapped || ev.held) {
          adjustHours ? clockMgr.adjustHours(-1) : clockMgr.adjustMinutes(-1);
          displayMgr.drawSetTimeUI(clockMgr, adjustHours);
        }
      } else if (ev.tapped || ev.held) {
        uint8_t v = player.volume();
        if (v > 0) player.setVolume(v-1);
        prefs.putUChar("volume", player.volume());
        overlayFlashUntil = millis() + 200;
      }
      break;
    case ButtonManager::BTN_VOL_UP:
      if (uiState == UiState::SET_TIME) {
        if (ev.tapped || ev.held) {
          adjustHours ? clockMgr.adjustHours(1) : clockMgr.adjustMinutes(1);
          displayMgr.drawSetTimeUI(clockMgr, adjustHours);
        }
      } else if (ev.tapped || ev.held) {
        uint8_t v = player.volume();
        if (v < 30) player.setVolume(v+1);
        prefs.putUChar("volume", player.volume());
        overlayFlashUntil = millis() + 200;
      }
      break;
  }
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  prefs.begin("spectra", false);
  buttons.begin();

  battery.begin();
  battery.setCalibration(1.0f, 0.0f); // adjust if needed

  clockMgr.begin(prefs);

  displayMgr.begin();

  uint8_t savedVol = prefs.getUChar("volume", 18);
  uint16_t savedTrack = prefs.getUShort("lastTrack", 1);

  if (!player.begin()) {
    uiState = UiState::ERROR_UI;
    displayMgr.drawError("DFPlayer init failed");
    return;
  }

  player.setVolume(savedVol);
  player.playTrack(savedTrack);
  overlayFlashUntil = millis() + 600;
}

// ---------------------------------------------------------------------------
void loop() {
  uint32_t now = millis();
  buttons.update(handleButton);
  clockMgr.update();
  battery.update();

  // check BT hold
  if (buttons.bothVolumePressed() && btHoldStart == 0) {
    btHoldStart = now;
  }
  if (!buttons.bothVolumePressed()) {
    btHoldStart = 0;
  }
  if (btHoldStart && buttons.bothVolumePressed() && now - btHoldStart > 2000) {
    uiState = (uiState == UiState::BT_UI) ? UiState::MAIN_UI : UiState::BT_UI;
    btHoldStart = 0;
    if (uiState == UiState::BT_UI) {
      displayMgr.drawBluetoothUI();
    } else {
      displayMgr.drawBackground();
    }
  }

  if (uiState == UiState::ERROR_UI) return;

  if (uiState == UiState::SET_TIME) {
    // already drawn; updates via button handler
    return;
  }

  if (uiState == UiState::BT_UI) {
    // nothing dynamic here; could add subtle animation if desired
    return;
  }

  // MAIN UI
  bool highlight = now < overlayFlashUntil;
  displayMgr.drawMainUI(player, battery, clockMgr, uiState, highlight);

  // persist track periodically
  static uint32_t lastSave = 0;
  if (now - lastSave > 5000) {
    prefs.putUShort("lastTrack", player.track());
    lastSave = now;
  }
}

