# BIOBUZZ Robot Code Guide (Maelstrom 2026-27)

All of our code is in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
**Every hardware name and every tunable number is in `RobotConstants.java`.** If something
needs changing, look there first.

---

## 1. Folder map

```
teamcode/
├── RobotConstants.java        ← ALL names + numbers (motor directions, speeds, positions)
├── subsystems/                ← one class per mechanism (no gamepad code in here)
│   ├── Drivetrain.java        mecanum drive, field-centric, IMU heading, motor reverse
│   ├── Odometry.java          position from the drive encoders + IMU
│   ├── Intake.java            1 motor, optional jam protection
│   ├── Elevator.java          1 motor, preset heights, holds position
│   ├── Transfer.java          feeder motor + flower servo + gate servo
│   └── Shooter.java           flywheel (velocity control) + optional servo
├── opmodes/                   ← what drivers run in a match
│   ├── DriveOnlyTeleOp.java   "BioBuzz Drive Only"
│   ├── BioBuzzTeleOp.java     "BioBuzz TeleOp" (full robot, hidden until wired)
│   ├── ParkAuto.java          "BioBuzz Auto - Park"
│   └── DriverControls.java    gamepad 1 driving, shared by every TeleOp
├── pedro/                     ← Pedro Pathing (needs a goBILDA Pinpoint)
│   ├── Constants.java         drivetrain + Pinpoint settings for Pedro
│   ├── Tuning.java            web tuner list (http://192.168.43.1:10158)
│   └── procedures/PinpointTuner.java
└── testing/                   ← setup + tuning tools, not for matches
    ├── HardwareCheck.java     "Hardware Check"
    └── tuning/
        ├── drivetrain/        "Drive Motor Directions"
        ├── pinpoint/          "Pinpoint 1 / 2 / 3"
        ├── shooter/           "Flywheel Tuning"
        ├── elevator/          "Elevator Tuning"
        └── servo/             "Servo Tuning"
```

---

## 2. Putting code on the robot

1. Turn the robot on and wait ~30 s.
2. Connect the laptop to the Control Hub, **either**:
   - **USB-C cable** into the Control Hub, **or**
   - **Wi-Fi**: join the Control Hub's network, then in Android Studio's **Terminal** run
     `adb connect 192.168.43.1:5555`
3. **File → Sync Project with Gradle Files** (needed after pulling new code or any `.gradle` change).
4. Top bar: **TeamCode** + **REV Robotics Control Hub** → press green **Run ▶**.
5. Wait for **"Install successfully finished."** The Driver Hub reconnects by itself.

---

## 3. Driver Hub configuration

**⋮ → Configure Robot → New (or Edit) → Scan → Control Hub Portal → Control Hub**.
Names are **case-sensitive** and must match exactly.

### Motors (Control Hub)
| Port | Type | Name |
|---|---|---|
| 0 | GoBILDA 5202/5203 series | `frontLeft` |
| 1 | GoBILDA 5202/5203 series | `backLeft` |
| 2 | GoBILDA 5202/5203 series | `backRight` |
| 3 | GoBILDA 5202/5203 series | `frontRight` |

Plug in all 4 encoder cables (odometry needs them).

### I2C
| Bus | Device | Name |
|---|---|---|
| 0 | REV internal IMU (already there by default) | `imu` |
| any free bus | goBILDA Pinpoint Odometry Computer | `pinpoint` *(only once you have one)* |

### Mechanisms (Expansion Hub, later)
| Kind | Name |
|---|---|
| Motor | `intake`, `elevator`, `transfer` (feeder), `shooter` |
| Servo | `flowerServo`, `gateServo`, `shooterServo` (optional) |

**Done → Save** as `BioBuzz`. The top of the Driver Hub should say `BioBuzz`.

---

## 4. Controller (PS5 DualSense)

1. Plug the controller into the **Driver Hub** with USB (not Bluetooth).
2. Hold **Options + Cross** → it becomes **gamepad 1**. (For gamepad 2: **Options + Circle**.)
3. A gamepad icon with **1** appears at the top of the screen.

### Gamepad 1: driving (every TeleOp)
| Control | Action |
|---|---|
| Left stick | Drive forward/back + strafe |
| Right stick X | Turn |
| **Triangle** | **Reverse drive**: back of robot becomes the front (rumbles) |
| **Create (Share)** | Field-centric ↔ robot-centric (rumbles) |
| **Options** | Reset heading: point robot away from you first |

### Gamepad 2: mechanisms (full "BioBuzz TeleOp" only)
| Control | Action |
|---|---|
| R1 / L1 (hold) | Intake in / reverse |
| D-pad up / right / down | Elevator UP / TRANSFER / DOWN |
| Left stick Y | Elevator fine adjust |
| R2 / L2 (hold) | Feeder into shooter / reverse |
| Cross | Toggle flower servo |
| Square | Toggle gate servo |
| Triangle | Shooter spin up / stop |
| Circle | Toggle shooter servo (if configured) |
| Options / Create | Shooter RPM + / − |
| PS button | Re-zero elevator at its current height |
| *(rumble)* | Shooter reached speed, ready to shoot |

### Autonomous "BioBuzz Auto - Park" (set during INIT, gamepad 1)
| Control | Action |
|---|---|
| D-pad up / down | Start delay ± 1 s (0–25 s) |
| D-pad left / right | Strafe toward left / right |

---

## 5. First-time setup order

Do these in order the first time, and again after any rebuild/rewire.

| # | OpMode | What to do | Copy result into |
|---|---|---|---|
| 1 | **Hardware Check** | Everything you've wired shows `found`. Turn each wheel by hand; its number should change. | Fix config names if `MISSING` |
| 2 | **Drive Motor Directions** | Robot on a box. Hold L1/R1/L2/R2 = FL/FR/BL/BR. Each wheel must spin to roll the robot **forward**. Cross flips the one you're holding. | `RobotConstants` → `*_REVERSED` |
| 3 | **BioBuzz Drive Only** | On the floor: stick forward = forward, right = strafe right, right stick right = turn clockwise. Encoders go **up** driving forward. | — |
| 4 | **Elevator Tuning** | Start fully down. Move with left stick; note ticks at each height. | `ELEVATOR_DOWN/TRANSFER/UP/MAX` |
| 5 | **Servo Tuning** | Square = next servo, D-pad right/left = move, Cross = step size. | `FLOWER_*`, `GATE_*`, `SHOOTER_SERVO_*` |
| 6 | **Flywheel Tuning** | Triangle = spin. Tune **F** first, then **P**. Then set the target RPM. | `SHOOTER_P/I/D/F`, `SHOOTER_TARGET_RPM` |
| 7 | **Pinpoint 1 - Pod Directions** | Push robot forward 12 in, then left 12 in, by hand. | `pedro/Constants` → `xPodDirection`, `yPodDirection` |
| 8 | **Pinpoint 2 - Pod Offsets** | Line robot up square, spin it 180° counter-clockwise by hand. | `pedro/Constants` → `xPodOffset`, `yPodOffset` |
| 9 | **Pinpoint 3 - Test** | Drive 48 in forward → X ≈ 48. Strafe 48 in left → Y ≈ 48. | — |

After each "copy into" step, **re-run ▶** from Android Studio so the robot gets the new numbers.

When every mechanism is wired and tuned, delete the `@Disabled` line in
`opmodes/BioBuzzTeleOp.java` to make **BioBuzz TeleOp** appear.

---

## 6. Optional drivetrain features (`RobotConstants`, Drivetrain section)

All are **off/normal** by default, so driving feels plain until you turn them on.

| Setting | Default | What it does |
|---|---|---|
| `FRONT_LEFT_REVERSED` … `BACK_RIGHT_REVERSED` | L side `true` | Quick per-motor reverse |
| `DRIVE_BRAKE` | `true` | Brake vs. coast on stick release |
| `DRIVE_MAX_POWER` | `1.0` | Top speed cap |
| `TURN_SCALE` | `1.0` | Lower = gentler turning |
| `DRIVE_INPUT_EXPONENT` | `1.0` | 2–3 = finer low-speed control |
| `STRAFE_CORRECTION` | `1.1` | Boosts strafe to match forward speed |
| `DRIVE_ACCEL_LIMIT` | `0` (off) | e.g. `4` = smooth starts/stops (anti-tip) |
| `HEADING_HOLD_ENABLED` | `false` | IMU keeps robot straight while strafing |
| `VOLTAGE_COMPENSATION` | `false` | Same speed on a low battery |
| `HUB_LOGO` / `HUB_USB` | `UP` / `FORWARD` | How the Control Hub is mounted. **Must be right** for field-centric and heading hold |

Mechanism reverse switches: `INTAKE_REVERSED`, `ELEVATOR_REVERSED`, `FEEDER_REVERSED`, `SHOOTER_REVERSED`.
Intake jam protection: `INTAKE_JAM_DETECTION` (off; check the amp limit first).

---

## 7. Troubleshooting

| Problem | Fix |
|---|---|
| "Unable to find a hardware device with name …" | That name in the Driver Hub config doesn't match `RobotConstants`. Fix spelling/capitals. |
| Robot spins when you push forward | One side's motors are backwards → **Drive Motor Directions**. |
| Strafes diagonally / one wheel fights | One motor reversed or two motor ports swapped in config. |
| Field-centric goes the wrong way | `HUB_LOGO` / `HUB_USB` don't match how the Control Hub is mounted. Press Options to reset heading facing away from you. |
| Encoder number stays 0 | Encoder cable unplugged at the motor or the hub. |
| Pinpoint OpModes say "NO PINPOINT" | Add `pinpoint` in the config (I2C) and restart the robot. |
| Red file names in Android Studio | Just new files not committed to git yet. Not an error. |
| Red underlines on `com.pedropathing` imports | **File → Sync Project with Gradle Files**, then **File → Invalidate Caches → Restart**. |
| "incompatible version of Kotlin" warning | Comes from the Pedro library being newer than Android Studio's Kotlin checker. Harmless; builds and runs fine. |
| Shooter reads negative RPM | Flip `SHOOTER_REVERSED`. |
| Elevator ticks go negative going up | Flip `ELEVATOR_REVERSED`. |
