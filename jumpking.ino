// Matrix Midterm game
//Jump King

#include <avr/io.h>
#include <util/delay.h>

//PINS
const uint8_t ROW_PINS[8] = {A0, A1, A2, A3, A4, A5, 8, 9};
const uint8_t SWITCH_PIN  = 10;   // button (to GND)

//low level display setup
static inline void allRowsOff() {
  for (uint8_t i = 0; i < 8; i++) digitalWrite(ROW_PINS[i], HIGH);
}
static inline void selectRow(uint8_t r) {
  digitalWrite(ROW_PINS[r], LOW);
}
//Show one full refresh (~8 ms total)
static inline void showFrameOnce(const uint8_t rows[8]) {
  for (uint8_t r = 0; r < 8; r++) {
    PORTD = 0x00;
    allRowsOff();
    selectRow(r);
    PORTD = rows[r];      // HIGH bits = on
    _delay_ms(1);
  }
}

//frame helpers
static inline void clearFB(uint8_t fb[8]) { for (uint8_t r=0; r<8; ++r) fb[r] = 0; }

static inline void setPixelScreen(uint8_t fb[8], int8_t sx, int8_t sy) {
  if (sx>=0 && sx<8 && sy>=0 && sy<8) fb[sy] |= (1 << sx);
}



//World & Camera
static const uint8_t MAP_W = 8;
static const uint8_t MAP_H = 40;
static int16_t camY = MAP_H - 8;     // start showing the bottom 8 rows only (upward scroll only)

// Platforms are in WORLD coordinates (y: 0=top, MAP_H-1=bottom)
struct Platform { uint8_t y, x0, x1; };

static const Platform PLATS[] = {
  {MAP_H-1, 0, 7},    // ground
  {MAP_H-5,  1, 2},
  {MAP_H-9,  4, 6},
  {MAP_H-13, 2, 3},
  {MAP_H-17, 5, 6},
  {MAP_H-21, 1, 2},
  {MAP_H-25, 6, 7},
  {MAP_H-29, 3, 5},
  {MAP_H-33, 0, 1},
};
static const uint8_t N_PLATS = sizeof(PLATS)/sizeof(PLATS[0]);

static inline bool isSolidWorld(int8_t x, int16_t y) {
  if (x < 0 || x >= MAP_W || y < 0 || y >= MAP_H) return false;
  for (uint8_t i = 0; i < N_PLATS; ++i) {
    if (y == PLATS[i].y && x >= PLATS[i].x0 && x <= PLATS[i].x1) return true;
  }
  return false;
}

// Draw visible slice of platforms according to camY
static inline void drawPlatforms(uint8_t fb[8]) {
  for (uint8_t i=0; i<N_PLATS; ++i) {
    int16_t wy = PLATS[i].y;
    int8_t sy = wy - camY;             // world→screen
    if (sy < 0 || sy > 7) continue;    // not visible
    for (uint8_t x = PLATS[i].x0; x <= PLATS[i].x1; ++x)
      setPixelScreen(fb, x, sy);
  }
}
static inline bool isSolid(int8_t x, int8_t y){
  for (uint8_t i=0;i<N_PLATS;i++)
    if (y==PLATS[i].y && x>=PLATS[i].x0 && x<=PLATS[i].x1) return true;
  return false;
}

//Game
static int8_t  pX = 3;
static int16_t pY = MAP_H - 1;   // start on ground
static int8_t  vX = 0, vY = 0;
static int8_t  facing = 1;       // +1 right, -1 left
static bool    onGround = true;
static bool gameWon = false;
static bool    btnPrev = false;
static uint16_t holdFrames = 0;

static const uint8_t  FRAMES_PER_TICK  = 10;  // ~6*8ms ≈ 48ms per tick
static const uint16_t LONG_PRESS_TICKS = 6;  // ~300ms
static const uint16_t MAX_CHARGE_TICKS = 25; // ~1.2s cap
static const int8_t   GRAVITY = 1;
static const int8_t   MAX_FALL_V = 2;
static const int8_t   HOP_VX = 1;

static inline bool btnPressed(){ return digitalRead(SWITCH_PIN)==LOW; }
static int8_t chargeToVy(uint16_t t){
  if (t < LONG_PRESS_TICKS) return 0;
  if (t > MAX_CHARGE_TICKS) t = MAX_CHARGE_TICKS;
  if (t < LONG_PRESS_TICKS + 6)  return -1;
  if (t < LONG_PRESS_TICKS + 12) return -2;
  return -3;
}

static void showWinMessage() {
  // For simplicity, just flash the screen a few times
  for (uint8_t k = 0; k < 6; ++k) {
    uint8_t fb[8];
    for (uint8_t i=0;i<8;i++) fb[i] = (k % 2 == 0) ? 0xFF : 0x00; // all ON / all OFF
    for (uint8_t t=0;t<50;t++) showFrameOnce(fb);
  }
}

static void tickGame() {
  if (gameWon) {
    showWinMessage();
    while (1); // stop game forever
  }
  //INPUT
  bool pressed = btnPressed();
  if (pressed) holdFrames++;
  if (!pressed && btnPrev) {
    // on release
    if (holdFrames < LONG_PRESS_TICKS) {
      // short tap → flip facing
      facing = -facing;
    } else if (onGround) {
      // long press → jump
      vY = chargeToVy(holdFrames); // negative = up
      vX = HOP_VX * facing;
      onGround = false;
    }
    holdFrames = 0;
  }
  btnPrev = pressed;

  //PHYSICS (WORLD)
  if (!onGround) {
    // horizontal
    int8_t nx = pX + vX;
    if (nx < 0 || nx >= MAP_W) { vX = -vX; nx = pX + vX; }
    pX = nx;

    // vertical
    pY += vY;
    if (vY < MAX_FALL_V) vY += GRAVITY;

    // landing checks (moving downward)
    if (pY >= MAP_H) pY = MAP_H - 1;
    if (vY >= 0) {
      if (isSolidWorld(pX, pY)) {
        pY -= 1; onGround = true; vY = 0; vX = 0;
      } else if (isSolidWorld(pX, pY + 1)) {
        onGround = true; vY = 0; vX = 0;
      }
    }
  } else {
    // walked off a ledge?
    if (!isSolidWorld(pX, pY + 1)) onGround = false;
  }
  if (pY <= 0 && !gameWon) {   // reached top row of world
    gameWon = true;
  }
  //CAMERA FOLLOW (bidirectional, smoothed)
  const int16_t TARGET_SCREEN_Y = 4;              // keep player near row 4 on screen
  int16_t targetCam = pY - TARGET_SCREEN_Y;

  if (targetCam < 0) targetCam = 0;
  int16_t maxCam = MAP_H - 8;
  if (targetCam > maxCam) targetCam = maxCam;

  // Move camera at most 1 row per tick toward target (follows up & down)
  if (targetCam > camY) camY++;
  else if (targetCam < camY) camY--;



  //RENDER (SCREEN)
  uint8_t fb[8]; clearFB(fb);
  drawPlatforms(fb);

  // charge bar on left edge (screen-space)
  if (holdFrames > 0) {
   uint8_t bars = (uint8_t)((holdFrames * 7) / MAX_CHARGE_TICKS);
   if (bars > 7) bars = 7;
   for (uint8_t i = 0; i <= bars; ++i) setPixelScreen(fb, 0, 7 - i);
  }

  // player in screen-space
  int8_t sY = (int8_t)(pY - camY);
  setPixelScreen(fb, pX, sY);

  // facing hint (screen-space)
  if (onGround && holdFrames == 0) {
    int8_t nx = pX + facing;
    if (nx>=0 && nx<8) setPixelScreen(fb, nx, sY);
  }

  // show one tick
  for (uint8_t i=0; i<FRAMES_PER_TICK; ++i) showFrameOnce(fb);

}

void setup() {
  // Columns (PORTD)
  DDRD = 0xFF;
  PORTD = 0x00;

  // Rows
  for (uint8_t i=0;i<8;i++){ pinMode(ROW_PINS[i], OUTPUT); digitalWrite(ROW_PINS[i], HIGH); }

  // Button
  pinMode(SWITCH_PIN, INPUT_PULLUP);
}

void loop() {
  tickGame();
}
