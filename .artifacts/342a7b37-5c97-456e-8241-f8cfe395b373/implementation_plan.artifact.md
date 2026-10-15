# Implementation Plan - Flashing Green goBILDA LED Indicator When Target is Shootable

Implement visual green flashing LED indicator feedback on the goBILDA LED light (configured as Servo `"LED"`) when an AprilTag target is determined to be **SHOOTABLE** in `UNDERVisionAuto.java` and `AROUNDVisionAuto.java`.

## User Review Required

> [!IMPORTANT]
> **Hardware Map Setup**:
> This plan assumes your goBILDA LED driver is configured in your Driver Station Robot Configuration on a **Servo port** with the exact device name **`"LED"`**.

> [!NOTE]
> **PWM Green Color & Flashing**:
> For Servo-based LED drivers (like REV Blinkin or goBILDA PWM LED modules), `0.725` corresponds to Green. To make it **flash green**, the code will alternate the servo position between `0.725` (Green) and `0.0` (Off) every 200ms whenever `anyShootable` is `true`. When not shootable, the LED turns off (`0.0`).

## Proposed Changes

### Autonomous OpModes (`APAuto.Auto`)

#### [MODIFY] [UNDERVisionAuto.java](file:///C:/Users/krupa/StudioProjects/BIOBUZZ-practice/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/APAuto/Auto/UNDERVisionAuto.java)
* Declare `private Servo led;` and initialize `led = hardwareMap.get(Servo.class, "LED");` in `init()`.
* In `loop()`, when `anyShootable` is `true`, toggle `led.setPosition(0.725)` (Green) every 200ms using `System.currentTimeMillis()` to create a flashing green light effect.
* When `anyShootable` is `false`, set `led.setPosition(0.0)` (Off).

#### [MODIFY] [AROUNDVisionAuto.java](file:///C:/Users/krupa/StudioProjects/BIOBUZZ-practice/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/APAuto/Auto/AROUNDVisionAuto.java)
* Declare `private Servo led;` and initialize `led = hardwareMap.get(Servo.class, "LED");` in `init()`.
* In `loop()`, when `anyShootable` is `true`, toggle `led.setPosition(0.725)` (Green) every 200ms using `System.currentTimeMillis()`.
* When `anyShootable` is `false`, set `led.setPosition(0.0)` (Off).

---

## Verification Plan

### Automated Tests
* Run `analyze_file` on `UNDERVisionAuto.java` and `AROUNDVisionAuto.java` to verify 0 compilation errors.
* Build the project (`gradle_build("TeamCode:assembleDebug")`).

### Manual Verification
1. Verify the servo port in Driver Station configuration is named `"LED"`.
2. Point the webcam at an AprilTag facing up (`FACING_UP`).
3. Verify that the goBILDA LED light flashes green when `Shootable` is `YES` and turns off when `Shootable` is `NO`.
