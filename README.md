#Arduino Pool Game for MCUFRIEND TFT Shield

This project implements a simple, physics-based game of pool (billiards) designed specifically for an Arduino Mega (or Uno) paired with an MCUFRIEND TFT touchscreen shield.

1. Hardware Requirements

Arduino Board: Arduino Uno or Mega (Mega is recommended for better performance and memory).

Display: MCUFRIEND 2.4" - 3.5" TFT Display Shield (ILI9341, etc.).

Touch Screen: The shield must have a functional resistive touchscreen overlay.

Required Pin Configuration (as used in the code):
| Function | Pin (Analog/Digital) |
| :--- | :--- |
| Touch Y+ | A1 |
| Touch X- | A2 |
| Touch Y- | D7 |
| Touch X+ | D6 |

(Note: These are standard pins for most MCUFRIEND shields.)

2. Software Setup

The following libraries must be installed in your Arduino IDE:

Adafruit GFX Library (By Adafruit)

MCUFRIEND_kbv (By David Prentice)

TouchScreen (By Adafruit)

3. Game Features & Controls

Gameplay Overview

The game uses basic collision and friction physics for a simple billiards experience. The goal is to pocket the object balls (colored balls). If the cue ball (white) is pocketed, it automatically respawns at the starting position (the head spot).

Controls (Aiming & Shooting)

The game operates in two states:

STATE_AIMING:

Aiming: Press and drag on the screen (starting near the cue ball).

Power: The length of the drag determines the shot power. Dragging away from the intended direction sets the vector and power.

Shot: Release the touch to fire the cue ball. A strong red line shows the aimed direction.

STATE_MOVING:

The physics engine takes over. The balls move, collide, and slow down due to friction.

Input is ignored until all balls come to a complete stop.

Difficulty Tweak (Catch Radius)

To make the game easier, the actual area that "catches" the ball into a pocket (CATCH_RADIUS) is larger than the visible black pocket drawn on the screen (POCKET_RADIUS).

POCKET_RADIUS (Visual Size): 12

CATCH_RADIUS (Physics Size): 18

4. Key Configuration Constants

You can adjust these constants in the code to change the gameplay feel:

| Constant | Description | Value |
| FRICTION | Controls how quickly balls slow down (closer to 1.0 is less friction). | 0.985 |
| MAX_POWER | Maximum possible speed/force applied by a shot. | 14 |
| MAX_DRAG_DIST | The drag distance needed to achieve MAX_POWER. | 80 |
| BALL_RADIUS | Radius of all balls in pixels. | 6 |
