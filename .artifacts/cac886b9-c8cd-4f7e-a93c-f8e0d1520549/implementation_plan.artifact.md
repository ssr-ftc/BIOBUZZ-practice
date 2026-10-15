# Fix PS4 Trigger Rotation and Right Stick Mapping

The robot rotates when you press the trigger because Android is mapping your **Left Trigger** to the same axis as your **Right Stick** (`right_stick_x`). This is a common PS4 controller issue on Android.

## User Review Required

> [!IMPORTANT]
> **Gamepad Mode**: Ensure your PS4 controller is not in a legacy mode.
> **Axis Isolation**: I am adding a "Safety Filter" that ignores trigger values when they overlap with the joystick.
> **Calibration**: I will add a **Start-time Calibration**. When you press **START**, do not touch the sticks or triggers. The code will "zero out" the phantom trigger values.

## Proposed Changes

### 1. Drivetrain Input Filtering
I will update the `loop()` to separate the trigger signal from the rotation signal.

#### [MODIFY] [testTeleop.java](file:///C:/Users/krupa/StudioProjects/BIOBUZZ-practice/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OFSB2/Teleop/testTeleop.java)
- **Calibration**: Add `static` variables to store the "resting" position of the joysticks.
- **Trigger Isolation**: If `left_trigger` is being read as `right_stick_x`, we will subtract it out.
- **Mecanum Smoothness**: Add a deadzone of `0.1` so the robot doesn't jitter.
- **Telemetry**: Add back the `Raw` vs `Processed` telemetry so you can see if the Right Stick is actually being detected.

### 2. Mecanum Normalization
- Instead of just `Range.clip`, I will add power normalization to ensure the robot doesn't lose its "path" when you turn and drive at the same time.

## Verification Plan

### Manual Verification
1. **Stationary Test**: Press **START** with hands off. Robot must stay still.
2. **Trigger Test**: Press Left Trigger. The robot should **only** slow down (multiplier), not rotate.
3. **Right Stick Test**: Move the Right Stick. Check the telemetry to see if `z` is changing. If not, I will help you find the correct axis name.
