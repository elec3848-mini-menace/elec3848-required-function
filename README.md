# ELEC3848 Robot Mission Controller

An Arduino Mega firmware for a four-wheel mecanum-drive robot that autonomously
completes a fixed arena mission: navigate to a colour tile, identify it, turn
90 degrees in the correct direction, and park within target distance of the new
walls.

---

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | Arduino Mega 2560 |
| Drive | 4 × DC motors with quadrature encoders, mecanum wheels |
| Front ranging | 2 × HC-SR04 ultrasonic sensors (left & right) |
| Side ranging | 2 × HC-SR04 ultrasonic sensors (right-facing & left-facing) |
| Light centering | 2 × analogue photoresistor / light-sensor modules |
| Colour detection | TCS3200 RGB colour sensor |
| Display | SSD1306 128 × 32 OLED (I²C) |

---

## Software Dependencies

Install the following libraries via the Arduino Library Manager before
compiling:

- **Adafruit SSD1306** (≥ 2.5)
- **Adafruit GFX Library** (≥ 1.11)

---

## Project Structure

```
main/
├── main.ino          # setup(), loop(), and table-driven mission orchestrator
├── config_pins.h     # All pin assignments and tunable constants
├── behavior.h/.cpp   # Behavior API + central dispatcher
├── behavior_internal.h
├── behavior_move.cpp
├── behavior_align_wall.cpp
├── behavior_align_side.cpp
├── behavior_align_light.cpp
├── behavior_color_turn.cpp
├── behavior_post_turn.cpp
├── motor.h/.cpp      # Motor driver, encoder ISRs, and speed-sync service
├── sensors.h/.cpp    # Ultrasonic, light, and colour sensor drivers
└── oled.h/.cpp       # SSD1306 display helpers
```

### Module Responsibilities

**`config_pins.h`** — Single source of truth for pin numbers and mission
parameters.  Adjust the constants here to tune robot behaviour without editing
any other file.

**`behavior.*` split modules** — Each major behavior now lives in its own file
(`behavior_move.cpp`, `behavior_align_wall.cpp`, etc.). `behavior.cpp` only
dispatches updates based on active behavior state. The mission runner calls
`start*` APIs and uses `tickBehaviorUntilComplete()` rather than directly
managing `currentRobotState`.

**`motor.h/.cpp`** — Low-level motor driver.  `moveRobot(dir, pwm)` sets wheel
direction and speed; `motorSyncService()` (called every 1 ms) trims individual
PWM values using encoder inter-tick timing to keep all four wheels at the same
speed.

**`sensors.h/.cpp`** — Sensor abstractions.  `getUltrasonicCM()` triggers an
HC-SR04 and returns distance in cm (999 = out of range).  `getLightSensor()`
returns a calibrated 0–100 intensity value.  `detectFloorColor()` averages
three TCS3200 RGB readings and classifies the tile as green (1), red (2), or
unknown (0).

**`oled.h/.cpp`** — `oledShowText(line1, line2)` clears the 128×32 OLED and
prints up to two text lines, providing real-time status feedback.

---

## Mission Sequence

The `loop()` function runs a nine-step table of mission actions.
Each step starts one non-blocking behavior, then waits until that behavior
reports completion before moving to the next step:

| Step | Action |
|------|--------|
| 0 | Move forward `MOVE_FORWARD_DISTANCE_CM` cm (encoder-counted) |
| 1 | Align squarely with the front wall (ultrasonic angle + distance) |
| 2 | Align laterally with the right side wall to `TARGET_SIDE_WALL_DISTANCE_CM` |
| 3 | Centre over the overhead light source (light-sensor strafe) |
| 4 | Re-align with the front wall (final fine correction) |
| 5 | Detect floor colour and execute a 90-degree turn (CCW = green, CW = red) |
| 6 | Correct post-turn heading angle using the front ultrasonic sensors |
| 7 | Adjust forward/backward distance to `POST_TURN_FRONT_TARGET_CM` |
| 8 | Align with the new side wall to `POST_TURN_SIDE_TARGET_CM` |

Between every step there is a `SHORT_ACTION_DELAY` ms pause to let the OLED
update and any mechanical vibration settle.

---

## Tuning Parameters

All tunable values live in `config_pins.h`:

| Constant | Default | Description |
|----------|---------|-------------|
| `MAIN_SPEED` | 60 | Base PWM for forward/backward moves |
| `TURN_SPEED` | 35 | PWM during in-place turns |
| `STRAFE_SPEED` | 50 | PWM for lateral strafing |
| `TICKS_PER_CM` | 18.0 | **Tune:** encoder ticks per cm of wheel travel |
| `MOVE_FORWARD_DISTANCE_CM` | 25 | Initial forward move distance |
| `TARGET_WALL_DISTANCE_CM` | 21 | Front-wall parking distance (cm) |
| `TARGET_SIDE_WALL_DISTANCE_CM` | 63 | Initial side-wall distance (cm) |
| `TURN_90_DEGREE_ENCODER_TICKS` | 550 | **Tune:** ticks for a 90-degree spin |
| `TURN_90_TIMEOUT_MS` | 3000 | Safety timeout for encoder-based turn |
| `ALIGNMENT_CONFIRMATION_THRESHOLD` | 3 | Consecutive reads to confirm alignment |
| `ALIGN_TURN_DURATION_MS` | 130 | Duration of each corrective micro-turn |
| `POST_TURN_FRONT_TARGET_CM` | 15 | Front distance after turning |
| `POST_TURN_SIDE_TARGET_CM` | 21 | Side distance after turning |

---

## Building and Uploading

1. Open `main/main.ino` in the Arduino IDE (or VS Code with the Arduino
   extension).
2. Select **Board: Arduino Mega or Mega 2560** and the correct COM port.
3. Install the required libraries (see *Software Dependencies* above).
4. Click **Upload**.

Serial output at 115200 baud logs each mission step and sensor reading for
debugging.
Required function for ELEC3848 robot
