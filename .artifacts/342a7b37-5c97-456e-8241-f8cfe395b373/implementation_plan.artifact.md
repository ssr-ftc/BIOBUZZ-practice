# Implementation Plan - Formatted Full Telemetry Layout in `carDashboard.java`

Structure telemetry in `carDashboard.java` so it displays in the exact requested order on the Driver Station screen:
1. **Shooter Status & Velocity** (Target & Current Velocity)
2. *Space*
3. **Car Dashboard** (Fuel Level, Distance/Odometer, Navigation Compass)
4. *Space*
5. **Driver & Operator Controls Guide**

## Proposed Changes

### TeleOp Utilities (`pedroPathing.templates.teleop`)

#### [MODIFY] [carDashboard.java](file:///C:/Users/krupa/StudioProjects/BIOBUZZ-practice/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/templates/teleop/carDashboard.java)
* Overload `update(depoLM0 depo)` to accept the shooter subsystem.
* Render telemetry in order:
  * **-----Shooter-----**: On/Off status, Target Velocity, Current Velocity (`depo.getVelocity()`)
  * *Space*
  * **-----Fuel-----**: Battery voltage gauge
  * *Space*
  * **-----Distance-----**: IPS speed, Session feet, All-time feet
  * *Space*
  * **-----Navigation-----**: Compass cardinal heading
  * *Space*
  * **-----Controls-----**: Driver 1 & Driver 2 button guide
* Call `telemetry.update()` once at the end of the frame.

#### [MODIFY] [drivePractice.java](file:///C:/Users/krupa/StudioProjects/BIOBUZZ-practice/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/templates/teleop/drivePractice.java)
* Remove syntax error in `start()` (`telemetry`).
* Remove duplicate `telemtry()` method from `drivePractice.java`.
* In `loop()`, call `dashboard.update(depo)`.

---

## Verification Plan

### Automated Tests
* Run `analyze_file` on `carDashboard.java` and `drivePractice.java` to confirm 0 compilation errors.

### Manual Verification
* Run `drivePractice` TeleOp on the robot.
* Verify Driver Station telemetry renders:
  `Shooter Velocity` $\rightarrow$ `Space` $\rightarrow$ `Dashboard (Fuel, Distance, Compass)` $\rightarrow$ `Space` $\rightarrow$ `Controls`.
