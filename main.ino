#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <TouchScreen.h>
#include <math.h>

// --- HARDWARE CONFIGURATION ---
MCUFRIEND_kbv tft;

// Touchscreen pins for most MCUFRIEND shields
uint8_t YP = A2;  // must be an analog pin for <TouchScreen.h>
uint8_t XM = A1;  // must be an analog pin for <TouchScreen.h>
uint8_t YM = 6;   // can be a digital pin
uint8_t XP = 7;   // can be a digital pin

// Resistance between X+ and X- (measure with multimeter or leave at 300)
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

// --- COLOR DEFINITIONS ---
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define DARKGREEN 0x03E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define GREY    0x8410
#define BROWN   0x9A60

// --- GAME CONSTANTS ---
const int SCREEN_W = 320;
const int SCREEN_H = 240;
const int BALL_RADIUS = 7;
const int BALL_COUNT = 7; 
const float FRICTION = 0.985;
const float STOP_SPEED = 0.15;
const int TABLE_MARGIN = 10;

// CONTROLS TWEAKS
const int MAX_POWER = 14;      // Increased from 10 for stronger shots
const int MAX_DRAG_DIST = 80;  // Decreased from 150 (shorter drag = max power)
const int TOUCH_DEBOUNCE = 5;  // Frames to wait before confirming release

// Pocket positions
const int POCKET_RADIUS = 11; // Visual size of the drawn pocket (reverted to 12)
const int CATCH_RADIUS = 14.5;  // TWEAKED: Invisible radius for potting calculation (much larger)
int pocketsX[6];
int pocketsY[6];

// --- TOUCH CALIBRATION ---
const int TS_LEFT = 907;
const int TS_RT   = 205;
const int TS_TOP  = 192;
const int TS_BOT  = 876;

// --- GAME STATE ---
enum GameState {
  STATE_AIMING,
  STATE_SHOOTING,
  STATE_MOVING
};
GameState currentState = STATE_AIMING;

struct Vector2 {
  float x;
  float y;
};

struct Ball {
  Vector2 pos;
  Vector2 vel;
  uint16_t color;
  bool active; // false if in pocket
  bool isCue;
};

Ball balls[BALL_COUNT];

// Touch interaction variables
bool isDragging = false;
Vector2 dragStart;
Vector2 dragCurrent;
int noTouchCount = 0; // For debouncing touch release

// --- FORWARD DECLARATIONS ---
void drawTable();
void redrawPockets();
void redrawBorders();
void setupBall(int index, int x, int y, uint16_t color);
void drawBall(int index, bool clear = false); 
void handleInput();
void updatePhysics();
void resetGame();

void setup() {
  Serial.begin(9600);
  
  uint16_t ID = tft.readID();
  if (ID == 0xD3D3) ID = 0x9481;
  tft.begin(ID);
  tft.setRotation(1); // Landscape
  
  // Define Pocket Locations
  pocketsX[0] = TABLE_MARGIN; pocketsY[0] = TABLE_MARGIN; // Top Left
  pocketsX[1] = SCREEN_W/2;   pocketsY[1] = TABLE_MARGIN; // Top Middle
  pocketsX[2] = SCREEN_W - TABLE_MARGIN; pocketsY[2] = TABLE_MARGIN; // Top Right
  pocketsX[3] = TABLE_MARGIN; pocketsY[3] = SCREEN_H - TABLE_MARGIN; // Bot Left
  pocketsX[4] = SCREEN_W/2;   pocketsY[4] = SCREEN_H - TABLE_MARGIN; // Bot Middle
  pocketsX[5] = SCREEN_W - TABLE_MARGIN; pocketsY[5] = SCREEN_H - TABLE_MARGIN; // Bot Right

  resetGame();
}

void resetGame() {
  tft.fillScreen(BLACK);
  drawTable();

  // Setup Cue Ball
  balls[0].pos.x = SCREEN_W / 4;
  balls[0].pos.y = SCREEN_H / 2;
  balls[0].vel.x = 0;
  balls[0].vel.y = 0;
  balls[0].color = WHITE;
  balls[0].active = true;
  balls[0].isCue = true;

  // Setup Object Balls
  int startX = (SCREEN_W * 3) / 4;
  int startY = SCREEN_H / 2;
  int idx = 1;
  uint16_t colors[] = {YELLOW, BLUE, RED, MAGENTA, CYAN, MAGENTA};
  
  setupBall(idx++, startX, startY, colors[0]);
  setupBall(idx++, startX + 13, startY - 7, colors[1]);
  setupBall(idx++, startX + 13, startY + 7, colors[2]);
  setupBall(idx++, startX + 26, startY - 14, colors[3]);
  setupBall(idx++, startX + 26, startY, colors[4]);
  setupBall(idx++, startX + 26, startY + 14, colors[5]);
  
  for(int i=0; i<BALL_COUNT; i++) {
    drawBall(i);
  }
}

void setupBall(int index, int x, int y, uint16_t color) {
  if (index >= BALL_COUNT) return;
  balls[index].pos.x = x;
  balls[index].pos.y = y;
  balls[index].vel.x = 0;
  balls[index].vel.y = 0;
  balls[index].color = color;
  balls[index].active = true;
  balls[index].isCue = false;
}

void loop() {
  if (currentState == STATE_AIMING) {
    handleInput();
  } else if (currentState == STATE_MOVING) {
    updatePhysics();
  }
  delay(10); 
}

void redrawBorders() {
  tft.fillRect(0, 0, SCREEN_W, TABLE_MARGIN, BROWN); // Top
  tft.fillRect(0, SCREEN_H - TABLE_MARGIN, SCREEN_W, TABLE_MARGIN, BROWN); // Bottom
  tft.fillRect(0, 0, TABLE_MARGIN, SCREEN_H, BROWN); // Left
  tft.fillRect(SCREEN_W - TABLE_MARGIN, 0, TABLE_MARGIN, SCREEN_H, BROWN); // Right
}

void redrawPockets() {
  for(int i=0; i<6; i++) {
    tft.fillCircle(pocketsX[i], pocketsY[i], POCKET_RADIUS, BLACK); // Uses POCKET_RADIUS (12)
  }
}

void drawTable() {
  tft.fillRect(TABLE_MARGIN, TABLE_MARGIN, SCREEN_W - 2*TABLE_MARGIN, SCREEN_H - 2*TABLE_MARGIN, DARKGREEN);
  
  redrawBorders();
  redrawPockets();
}

void drawBall(int index, bool clear) {
  if (!balls[index].active) return;
  
  uint16_t c = clear ? DARKGREEN : balls[index].color;
  int x = (int)balls[index].pos.x;
  int y = (int)balls[index].pos.y;
  
  int radius = BALL_RADIUS;

  // IMPORTANT: When clearing the ball, use a slightly larger radius (R+1)
  // to ensure all residual colored pixels from movement/collision jitter are erased.
  if (clear) {
    radius += 1; 
  }

  tft.fillCircle(x, y, radius, c);

  // --- BORDER REDRAW FIX (Only required when clearing near border) ---
  // If we just erased a ball (clear=true) and it was near a border,
  // we might have accidentally painted Green over the Brown rail.
  if (clear) {
    // Check Top Border
    if (y - BALL_RADIUS <= TABLE_MARGIN) {
      tft.fillRect(x - BALL_RADIUS, 0, BALL_RADIUS * 2 + 1, TABLE_MARGIN, BROWN);
    }
    // Check Bottom Border
    if (y + BALL_RADIUS >= SCREEN_H - TABLE_MARGIN) {
      tft.fillRect(x - BALL_RADIUS, SCREEN_H - TABLE_MARGIN, BALL_RADIUS * 2 + 1, TABLE_MARGIN, BROWN);
    }
    // Check Left Border
    if (x - BALL_RADIUS <= TABLE_MARGIN) {
      tft.fillRect(0, y - BALL_RADIUS, TABLE_MARGIN, BALL_RADIUS * 2 + 1, BROWN);
    }
    // Check Right Border
    if (x + BALL_RADIUS >= SCREEN_W - TABLE_MARGIN) {
      tft.fillRect(SCREEN_W - TABLE_MARGIN, y - BALL_RADIUS, TABLE_MARGIN, BALL_RADIUS * 2 + 1, BROWN);
    }
  }
}

void handleInput() {
  TSPoint p = ts.getPoint();
  
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  // Pressure check
  if (p.z > 10 && p.z < 1000) {
    noTouchCount = 0; // Reset debounce counter because we have a touch

    int tx = map(p.y, TS_LEFT, TS_RT, 0, SCREEN_W);
    int ty = map(p.x, TS_TOP, TS_BOT, 0, SCREEN_H);

    if (!isDragging) {
      isDragging = true;
      dragStart.x = tx;
      dragStart.y = ty;
    } 
    
    dragCurrent.x = tx;
    dragCurrent.y = ty;
    
    float dx = dragStart.x - dragCurrent.x;
    float dy = dragStart.y - dragCurrent.y;
    
    // Visual limit on power line length
    float power = sqrt(dx*dx + dy*dy);
    // Draw aiming line
    tft.drawLine(balls[0].pos.x, balls[0].pos.y, balls[0].pos.x + dx, balls[0].pos.y + dy, RED);
    delay(5);
    tft.drawLine(balls[0].pos.x, balls[0].pos.y, balls[0].pos.x + dx, balls[0].pos.y + dy, DARKGREEN);

  } else {
    // No valid pressure detected
    if (isDragging) {
      noTouchCount++;
      
      // DEBOUNCE: Only confirm release if we haven't seen a touch for 'TOUCH_DEBOUNCE' frames
      if (noTouchCount >= TOUCH_DEBOUNCE) {
        isDragging = false;
        
        float dx = dragStart.x - dragCurrent.x;
        float dy = dragStart.y - dragCurrent.y;
        
        float power = sqrt(dx*dx + dy*dy);
        float angle = atan2(dy, dx);
        
        if (power > 10) { 
          if (power > MAX_DRAG_DIST) power = MAX_DRAG_DIST; 
          
          // Map inputs (0 to MAX_DRAG_DIST) to output force (0 to MAX_POWER)
          float shootPower = map(power, 0, MAX_DRAG_DIST, 0, MAX_POWER);
          
          balls[0].vel.x = cos(angle) * shootPower;
          balls[0].vel.y = sin(angle) * shootPower;
          
          currentState = STATE_MOVING;
        }
        noTouchCount = 0;
      }
    }
  }
}

void updatePhysics() {
  bool anyMoving = false;
  
  // 1. Erase balls using R+1 to ensure no artifacts are left
  for(int i=0; i<BALL_COUNT; i++) {
    if(balls[i].active && (abs(balls[i].vel.x) > 0.01 || abs(balls[i].vel.y) > 0.01)) {
      drawBall(i, true); // True triggers R+1 clear
    }
  }

  // 2. Physics
  for(int i=0; i<BALL_COUNT; i++) {
    if (!balls[i].active) continue;

    balls[i].pos.x += balls[i].vel.x;
    balls[i].pos.y += balls[i].vel.y;
    
    balls[i].vel.x *= FRICTION;
    balls[i].vel.y *= FRICTION;
    
    if (abs(balls[i].vel.x) < STOP_SPEED && abs(balls[i].vel.y) < STOP_SPEED) {
      balls[i].vel.x = 0;
      balls[i].vel.y = 0;
    } else {
      anyMoving = true;
    }

    // Wall Collisions (Bounce factor increased to -0.95 for less energy loss)
    if (balls[i].pos.x > SCREEN_W - TABLE_MARGIN - BALL_RADIUS) {
      balls[i].pos.x = SCREEN_W - TABLE_MARGIN - BALL_RADIUS;
      balls[i].vel.x *= -0.95; // TWEAKED
    }
    if (balls[i].pos.x < TABLE_MARGIN + BALL_RADIUS) {
      balls[i].pos.x = TABLE_MARGIN + BALL_RADIUS;
      balls[i].vel.x *= -0.95; // TWEAKED
    }
    if (balls[i].pos.y > SCREEN_H - TABLE_MARGIN - BALL_RADIUS) {
      balls[i].pos.y = SCREEN_H - TABLE_MARGIN - BALL_RADIUS;
      balls[i].vel.y *= -0.95; // TWEAKED
    }
    if (balls[i].pos.y < TABLE_MARGIN + BALL_RADIUS) {
      balls[i].pos.y = TABLE_MARGIN + BALL_RADIUS;
      balls[i].vel.y *= -0.95; // TWEAKED
    }

    // Pockets
    for(int p=0; p<6; p++) {
      float pdx = balls[i].pos.x - pocketsX[p];
      float pdy = balls[i].pos.y - pocketsY[p];
      // Use the larger CATCH_RADIUS for detection
      if (sqrt(pdx*pdx + pdy*pdy) < CATCH_RADIUS) { 
        balls[i].active = false;
        // Explicitly clear pocketed ball using R+2 (for safety)
        tft.fillCircle(balls[i].pos.x, balls[i].pos.y, BALL_RADIUS + 2, DARKGREEN); 
        
        // Redraw the pocket using the smaller POCKET_RADIUS (12)
        tft.fillCircle(pocketsX[p], pocketsY[p], POCKET_RADIUS, BLACK); 
        
        if (balls[i].isCue) {
          balls[i].pos.x = SCREEN_W/4;
          balls[i].pos.y = SCREEN_H/2;
          balls[i].vel.x = 0;
          balls[i].vel.y = 0;
          balls[i].active = true;
          drawBall(i);
        }
      }
    }
  }

  // 3. Collisions
  for (int i = 0; i < BALL_COUNT; i++) {
    for (int j = i + 1; j < BALL_COUNT; j++) {
      if (balls[i].active && balls[j].active) {
        float dx = balls[j].pos.x - balls[i].pos.x;
        float dy = balls[j].pos.y - balls[i].pos.y;
        float dist = sqrt(dx*dx + dy*dy);

        if (dist < BALL_RADIUS * 2) {
          float angle = atan2(dy, dx);
          float targetX = balls[i].pos.x + cos(angle) * BALL_RADIUS * 2;
          float targetY = balls[i].pos.y + sin(angle) * BALL_RADIUS * 2;
          float ax = (targetX - balls[j].pos.x) * 0.5; 
          float ay = (targetY - balls[j].pos.y) * 0.5;
          balls[i].pos.x -= ax;
          balls[i].pos.y -= ay;
          balls[j].pos.x += ax;
          balls[j].pos.y += ay;

          float nx = dx / dist;
          float ny = dy / dist;
          float tx = -ny;
          float ty = nx;

          float dpTan1 = balls[i].vel.x * tx + balls[i].vel.y * ty;
          float dpTan2 = balls[j].vel.x * tx + balls[j].vel.y * ty;
          float dpNorm1 = balls[i].vel.x * nx + balls[i].vel.y * ny;
          float dpNorm2 = balls[j].vel.x * nx + balls[j].vel.y * ny;

          float m1 = dpNorm2;
          float m2 = dpNorm1;

          balls[i].vel.x = tx * dpTan1 + nx * m1;
          balls[i].vel.y = ty * dpTan1 + ny * m1;
          balls[j].vel.x = tx * dpTan2 + nx * m2;
          balls[j].vel.y = ty * dpTan2 + ny * m2;
        }
      }
    }
  }

  // 4. Redraw Balls
  for(int i=0; i<BALL_COUNT; i++) {
    drawBall(i);
  }

  // 5. Redraw Static Elements (PERFORMANCE FIX)
  // Only redraw static elements once after all movement stops, preventing lag.
  if (!anyMoving) {
    redrawBorders(); 
    redrawPockets(); 
    currentState = STATE_AIMING;
  }
}
