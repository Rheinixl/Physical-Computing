#include <FastLED.h>
#include <AccelStepper.h>

// LED setup and variables
#define DATA_PIN    6
#define BRIGHTNESS  10

#define NATIVE_W 32
#define NATIVE_H 8
#define MAJOR_ORDER_ROWS 0
#define SERPENTINE_MAJOR 1
#define LEDS_PER_PANEL (NATIVE_W * NATIVE_H) // 256

#define PANELS 4
#define NUM_LEDS (LEDS_PER_PANEL * PANELS)
CRGB leds[NUM_LEDS];

// 30 FPS
unsigned long lastFrameMs = 0;
const unsigned long FRAME_MS = 33;

// 4 screene vertically placed next to each other
#define PANEL_LOG_W 8
#define PANEL_LOG_H 32
#define WIDTH  (PANEL_LOG_W * PANELS)  // 32
#define HEIGHT (PANEL_LOG_H)           // 32

static const uint8_t panelOrder[4] = {3,2,1,0};
// the second and 4th screens are upside down
static const bool panelFlip180[4]  = {true, false, true, false};

// translate pixel
uint16_t idxInPanelFromUV(int u, int v) {
  if (MAJOR_ORDER_ROWS) {
    if (SERPENTINE_MAJOR && (v & 1)) return v * NATIVE_W + (NATIVE_W - 1 - u);
    return v * NATIVE_W + u;
  } else {
    if (SERPENTINE_MAJOR && (u & 1)) return u * NATIVE_H + (NATIVE_H - 1 - v);
    return u * NATIVE_H + v;
  }
}

// 32*32 to physical pixel
void setPixel(int x, int y, const CRGB &c) {
  if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;

  int panelLR = x / PANEL_LOG_W;      // 0..3 (从左到右)
  int lx = x % PANEL_LOG_W;           // 0..7
  int ly = y;                         // 0..31

  if (panelFlip180[panelLR]) {
    lx = (PANEL_LOG_W - 1) - lx;
    ly = (PANEL_LOG_H - 1) - ly;
  }

  // (8x32)->(32x8)
  int u = ly; // 0..31
  int v = lx; // 0..7

  int physPanel = panelOrder[panelLR];
  uint16_t idx = physPanel * LEDS_PER_PANEL + idxInPanelFromUV(u, v);
  leds[idx] = c;
}

void clearAll() { fill_solid(leds, NUM_LEDS, CRGB::Black); }

// fb cache
CRGB fb[WIDTH * HEIGHT];

inline void fbClear() { fill_solid(fb, WIDTH * HEIGHT, CRGB::Black); }
inline void fbSet(int x, int y, const CRGB &c) {
  if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
  fb[y * WIDTH + x] = c;
}
inline CRGB fbGet(int x, int y) {
  if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return CRGB::Black;
  return fb[y * WIDTH + x];
}

void renderFB() {
  clearAll();
  for (int y = 0; y < HEIGHT; y++) {
    for (int x = 0; x < WIDTH; x++) {
      setPixel(x, y, fbGet(x, y));
    }
  }
}

bool screenOff = false;

void screenPowerOff() {
  screenOff = true;
  clearAll();
  FastLED.show();
}

void screenPowerOn() {
  screenOff = false;
}

// rainbow rain animation
// every column one rain drop
int16_t rainY[WIDTH];
uint8_t rainHue[WIDTH];
uint8_t rainSpeed[WIDTH];
uint8_t rainLen[WIDTH];

void initRainbowRain() {
  for (int x = 0; x < WIDTH; x++) {
    // put raindrop at random height and start falling
    rainY[x] = HEIGHT + random(0, HEIGHT);
    rainHue[x] = random8();
    rainSpeed[x] = 1 + random8(0, 2);    // 1~2
    rainLen[x] = 6 + random8(0, 8);      // 6~13
  }
}

void rainbowRainStepToFB() {
  fbClear();

  for (int x = 0; x < WIDTH; x++) {
    rainY[x] -= rainSpeed[x];

    if (rainY[x] + (int16_t)rainLen[x] < 0) {
      rainY[x] = HEIGHT + random(0, HEIGHT);
      rainHue[x] = random8();
      rainSpeed[x] = 1 + random8(0, 2);
      rainLen[x] = 6 + random8(0, 8);
    }

    for (int k = 0; k < rainLen[x]; k++) {
      int y = (int)rainY[x] + k;
      if (y < 0 || y >= HEIGHT) continue;

      uint8_t val = (k == 0) ? 255 : (uint8_t)max(0, 220 - k * 20);
      uint8_t hue = rainHue[x] + k * 6;
      fbSet(x, y, CHSV(hue, 255, val));
    }
  }
}


// enjoy rainbow background
void rainbowBackgroundToFB(unsigned long now) {
  // bigger now=flow of rainbow
  uint8_t baseHue = (uint8_t)(now >> 4); // 慢一点
  for (int y = 0; y < HEIGHT; y++) {
    for (int x = 0; x < WIDTH; x++) {
      uint8_t hue = baseHue + x * 6 + y * 3;
      // bgd not too bright
      uint8_t val = 55 + (sin8((x * 10) + (now >> 2)) >> 3); // ~55..86
      fbSet(x, y, CHSV(hue, 255, val));
    }
  }
}

// 5*7 font
struct Glyph { char c; uint8_t col[5]; };
const Glyph font[] PROGMEM = {
  {' ', {0x00,0x00,0x00,0x00,0x00}},
  {'0', {0x3E,0x45,0x49,0x51,0x3E}},
  {'1', {0x00,0x21,0x7F,0x01,0x00}},
  {'2', {0x21,0x43,0x45,0x49,0x31}},
  {'3', {0x42,0x41,0x51,0x69,0x46}},
  {'4', {0x0C,0x14,0x24,0x7F,0x04}},
  {'5', {0x72,0x51,0x51,0x51,0x4E}},
  {'6', {0x1E,0x29,0x49,0x49,0x06}},
  {'7', {0x40,0x47,0x48,0x50,0x60}},
  {'8', {0x36,0x49,0x49,0x49,0x36}},
  {'9', {0x30,0x49,0x49,0x4A,0x3C}},
  {'A', {0x3F,0x44,0x44,0x44,0x3F}},
  {'B', {0x7F,0x49,0x49,0x49,0x36}},
  {'C', {0x3E,0x41,0x41,0x41,0x22}},
  {'D', {0x7F,0x41,0x41,0x22,0x1C}},
  {'E', {0x7F,0x49,0x49,0x49,0x41}},
  {'F', {0x7F,0x48,0x48,0x48,0x40}},
  {'G', {0x3E,0x41,0x49,0x49,0x2E}},
  {'H', {0x7F,0x08,0x08,0x08,0x7F}},
  {'I', {0x00,0x41,0x7F,0x41,0x00}},
  {'J', {0x02,0x01,0x01,0x01,0x7E}},
  {'K', {0x7F,0x08,0x14,0x22,0x41}},
  {'L', {0x7F,0x01,0x01,0x01,0x01}},
  {'M', {0x7F,0x20,0x10,0x20,0x7F}},
  {'N', {0x7F,0x20,0x10,0x08,0x7F}},
  {'O', {0x3E,0x41,0x41,0x41,0x3E}},
  {'P', {0x7F,0x48,0x48,0x48,0x30}},
  {'Q', {0x3E,0x41,0x45,0x42,0x3D}},
  {'R', {0x7F,0x48,0x4C,0x4A,0x31}},
  {'S', {0x32,0x49,0x49,0x49,0x26}},
  {'T', {0x40,0x40,0x7F,0x40,0x40}},
  {'U', {0x7E,0x01,0x01,0x01,0x7E}},
  {'V', {0x7C,0x02,0x01,0x02,0x7C}},
  {'W', {0x7E,0x01,0x06,0x01,0x7E}},
  {'X', {0x63,0x14,0x08,0x14,0x63}},
  {'Y', {0x70,0x08,0x07,0x08,0x70}},
  {'Z', {0x43,0x45,0x49,0x51,0x61}},
};

bool getGlyph(char c, uint8_t outCols[5]) {
  if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';
  for (uint16_t i = 0; i < sizeof(font)/sizeof(font[0]); i++) {
    char fc = pgm_read_byte(&font[i].c);
    if (fc == c) {
      for (int k = 0; k < 5; k++) outCols[k] = pgm_read_byte(&font[i].col[k]);
      return true;
    }
  }
  for (int k = 0; k < 5; k++) outCols[k] = 0x00;
  return false;
}

void drawChar5x7FB(int x, int y, char c, const CRGB &color) {
  uint8_t cols[5];
  getGlyph(c, cols);
  for (int cx = 0; cx < 5; cx++) {
    uint8_t bits = cols[cx];
    for (int cy = 0; cy < 7; cy++) {
      if (bits & (1 << cy)) fbSet(x + cx, y + cy, color);
    }
  }
}

int textPixelLen(const char *s) { int n=0; while(*s){ n+=6; s++; } return n; }

void drawTextFB(int x, int y, const char *s, const CRGB &color) {
  int cx = x;
  while (*s) { drawChar5x7FB(cx, y, *s, color); cx += 6; s++; }
}

// centered found for start screen
void drawCenteredTextLineFB(int y, const char* s, const CRGB& color) {
  int w = textPixelLen(s);                 // 像素宽
  int x = (WIDTH - w) / 2;                 // centered
  drawTextFB(x, y, s, color);
}

void drawIdle3LinesCentered() {
  const char* L1 = "WAVE";

  const int lineH = 7;
  const int gap   = 1;
  const int totalH = lineH*3 + gap*2;

  int yBase = (HEIGHT - totalH) / 2; // measured from BOTTOM in coords

  // Because y increases UP, highest y is "top line"
  drawCenteredTextLineFB(yBase + 2*(lineH+gap), L1, CRGB::White); // top
}


void drawLineFB(int x0,int y0,int x1,int y1,const CRGB& c){
  int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
  int dy = -abs(y1-y0), sy = y0<y1 ? 1 : -1;
  int err = dx + dy;
  while(true){
    fbSet(x0,y0,c);
    if(x0==x1 && y0==y1) break;
    int e2 = 2*err;
    if(e2 >= dy){ err += dy; x0 += sx; }
    if(e2 <= dx){ err += dx; y0 += sy; }
  }
}

const unsigned long ARROW_BLINK_MS = 250;  // 250ms 翻转一次（= 2Hz 闪烁）

bool arrowBlinkOn(unsigned long now) {
  return ((now / ARROW_BLINK_MS) % 2) == 0;
}

void drawArrowToBottomRight(const CRGB& c) {
  int x1 = WIDTH - 2;
  int y1 = 1;                 // bottom

  int head = 10;               // size of the right-angle
  int shaft = 15;             // length of diagonal

  int x0 = x1 - shaft;
  int y0 = y1 + shaft;        // up-left

  // diagonal line
  drawLineFB(x0, y0, x1, y1, c);

  // right-angle tip
  for (int i = 0; i < head; i++) fbSet(x1 - i, y1, c);  // left from tip
  for (int i = 0; i < head; i++) fbSet(x1, y1 + i, c);  // up from tip
}




// UI
void drawPanelBorder(int panelLR, const CRGB &c) {
  int x0 = panelLR * PANEL_LOG_W;
  int x1 = x0 + PANEL_LOG_W - 1;
  int y0 = 0;
  int y1 = HEIGHT - 1;
  for (int x = x0; x <= x1; x++) { fbSet(x, y0, c); fbSet(x, y1, c); }
  for (int y = y0; y <= y1; y++) { fbSet(x0, y, c); fbSet(x1, y, c); }
}
void fillPanel(int panelLR, const CRGB &c) {
  int x0 = panelLR * PANEL_LOG_W;
  for (int y = 0; y < HEIGHT; y++) {
    for (int x = x0; x < x0 + PANEL_LOG_W; x++) fbSet(x, y, c);
  }
}

// motion sensor
#define PIR_PIN 12
bool lastPir = false;
unsigned long lastTriggerMs = 0;
const unsigned long TRIGGER_COOLDOWN_MS = 2500; //change this if motion sensor doesn't cool down and continuously trigger

// step motors
static const long STEPS_PER_REV = 4096;

const uint8_t motorPins[4][4] = {
  {50, 51, 52, 53},
  {46, 47, 48, 49},
  {42, 43, 44, 45},
  {38, 39, 40, 41}
};

AccelStepper stepper0(AccelStepper::HALF4WIRE, motorPins[0][0], motorPins[0][2], motorPins[0][1], motorPins[0][3]);
AccelStepper stepper1(AccelStepper::HALF4WIRE, motorPins[1][0], motorPins[1][2], motorPins[1][1], motorPins[1][3]);
AccelStepper stepper2(AccelStepper::HALF4WIRE, motorPins[2][0], motorPins[2][2], motorPins[2][1], motorPins[2][3]);
AccelStepper stepper3(AccelStepper::HALF4WIRE, motorPins[3][0], motorPins[3][2], motorPins[3][1], motorPins[3][3]);

AccelStepper* steppers[4] = { &stepper0, &stepper1, &stepper2, &stepper3 };

void motorsSleepAll() {
  for (int i = 0; i < 4; i++) steppers[i]->disableOutputs();
}

//state machine
enum State {
  ST_IDLE,
  ST_READY_TEXT,
  ST_SPIN,
  ST_RESULT,
  ST_DISPENSE,
  ST_COOLDOWN
};

State state = ST_IDLE;
unsigned long stateStartMs = 0;

const CRGB prizeColor[4] = { CRGB::Red, CRGB::Yellow, CRGB::Orange, CRGB::Blue };
const char* prizeName[4] = { "RED", "YELLOW", "ORANGE", "BLUE" };

int spinIndex = 0;
int targetIndex = 0;
long spinTicksRemaining = 0;
unsigned long lastSpinTickMs = 0;
unsigned long spinIntervalMs = 60;

int dispenseMotor = 0;
int dispensePhase = 0;

// IDLE 滚动字 not in use
int idleScrollX = WIDTH;
const char* IDLE_MSG  = "WAVE TO PLAY  ";
const char* READY_MSG = "READY";

const unsigned long ENJOY_MS = 3000;

void startLottery() {
  state = ST_READY_TEXT;
  stateStartMs = millis();

  targetIndex = random(0, 4);
  spinIndex   = random(0, 4);

  long base = (24 + random(0, 6) * 4);
  int offset = (targetIndex - spinIndex + 4) % 4;
  spinTicksRemaining = base + offset;

  spinIntervalMs = 60;
  lastSpinTickMs = millis();
}

long stepsForAngle(float angleDeg) {
  return lround((STEPS_PER_REV * angleDeg) / 360.0);
}

void startDispense(int motorIdx) {
  dispenseMotor = motorIdx;
  dispensePhase = 0;

  AccelStepper* m = steppers[dispenseMotor];
  m->enableOutputs();
  m->setMaxSpeed(900.0);
  m->setAcceleration(2500.0);
  m->move(stepsForAngle(100));
}

void updateDispense() {
  AccelStepper* m = steppers[dispenseMotor];
  m->run();

  if (m->distanceToGo() == 0) {
    if (dispensePhase == 0) {
      dispensePhase = 1;
      m->move(stepsForAngle(0));
    } else {
      m->disableOutputs();
      screenPowerOn();
      state = ST_COOLDOWN;
      stateStartMs = millis();
    }
  }
}

// setup and loop
void setup() {
  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT);

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  clearAll();
  FastLED.show();

  randomSeed(analogRead(A0));

  for (int i = 0; i < 4; i++) {
    steppers[i]->setCurrentPosition(0);
    steppers[i]->disableOutputs();
  }

  initRainbowRain();

  fbClear();
  lastFrameMs = millis();
}

void loop() {
  unsigned long now = millis();

  // doFrame：固定帧率推进显示
  bool doFrame = (now - lastFrameMs >= FRAME_MS);
  if (doFrame) lastFrameMs = now;

  // motion sensor trigger
  bool pirNow = (digitalRead(PIR_PIN) == HIGH);
  bool rising = (pirNow && !lastPir);
  lastPir = pirNow;

  if (state == ST_IDLE && rising && (now - lastTriggerMs > TRIGGER_COOLDOWN_MS)) {
    lastTriggerMs = now;
    startLottery();
  }

  // motor moves regardless of refreshes or clears
  if (state == ST_DISPENSE) {
    updateDispense();
  }

  // 显示：只在 doFrame 时刷新
  if (!doFrame) return;

  switch (state) {
    case ST_IDLE: {
      rainbowRainStepToFB();

      drawIdle3LinesCentered();
      if (arrowBlinkOn(now)) {
        drawArrowToBottomRight(CRGB::White);
      }

      motorsSleepAll();
    } break;

    case ST_READY_TEXT: {
      fbClear();
      int msgW = textPixelLen(READY_MSG);
      // int x = (WIDTH - msgW) / 2;
      int x = 1;
      int y = (HEIGHT / 2) - 3;
      drawTextFB(x, y, READY_MSG, CRGB::White);

      if (now - stateStartMs > 800) {
        state = ST_SPIN;
        stateStartMs = now;
      }
      motorsSleepAll();
    } break;

    case ST_SPIN: {
      fbClear();
      if (now - lastSpinTickMs >= spinIntervalMs) {
        lastSpinTickMs = now;
        spinIndex = (spinIndex + 1) % 4;
        spinTicksRemaining--;

        if (spinTicksRemaining < 16) spinIntervalMs += 12;
        else spinIntervalMs += 2;
        if (spinIntervalMs > 220) spinIntervalMs = 220;

        if (spinTicksRemaining <= 0) {
          state = ST_RESULT;
          stateStartMs = now;
        }
      }

      drawPanelBorder(spinIndex, CRGB::White);
      drawTextFB(1, 2, "CATCH IT", CRGB::White);

      motorsSleepAll();
    } break;

    case ST_RESULT: {
      fbClear();
      int win = targetIndex;

      fillPanel(win, prizeColor[win]);
      drawPanelBorder(win, CRGB::White);

      if (now - stateStartMs > 800) {
        screenPowerOff();
        state = ST_DISPENSE;
        stateStartMs = now;
        startDispense(win);
      }
    } break;

    case ST_DISPENSE: {
      fbClear();
      int win = targetIndex;
      drawPanelBorder(win, CRGB::White);
      drawTextFB(1, 2, "DISPENSING", CRGB::White);
    } break;

    case ST_COOLDOWN: {
      rainbowBackgroundToFB(now);

      drawTextFB(1, (HEIGHT/2)-3, "ENJOY!", CRGB::White);

      if (now - stateStartMs > ENJOY_MS) {
        state = ST_IDLE;
        stateStartMs = now;
      }
      motorsSleepAll();
    } break;
  }

  if (!screenOff) {
    renderFB();
    FastLED.show();
  }
}
