# Implementation Plan - Move Dashboard Telemetry to `carDashboard.java`

Encapsulate all dashboard telemetry, battery fuel gauge, compass navigation, and session/all-time odometer tracking into the `carDashboard` utility class so any TeleOp OpMode can easily display dashboard telemetry with a single `dashboard.update()` call.

## Proposed Changes

### TeleOp Utilities (`pedroPathing.templates.teleop`)

#### [MODIFY] [carDashboard.java](file:///C:/Users/krupa/StudioProjects/BIOBUZZ-practice/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/templates/teleop/carDashboard.java)
* Define the `carDashboard` class accepting `HardwareMap`, `Telemetry`, and `Follower` instances in its constructor.
* Implement `start()`, `update()`, `fuelLevel()`, `getCompass()`, and `stop()` methods.
* Automatically manage session feet traveled, all-time distance (persisted via `SharedPreferences`), speed (IPS), battery fuel level gauge, and compass direction.

#### [MODIFY] [drivePractice.java](file:///C:/Users/krupa/StudioProjects/BIOBUZZ-practice/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/templates/teleop/drivePractice.java)
* Instantiates `dashboard = new carDashboard(hardwareMap, telemetry, follower)` in `init()`.
* Calls `dashboard.start()` in `start()`.
* Calls `dashboard.update()` in `loop()` and `dashboard.stop()` in `stop()`.
* Removes duplicate inline telemetry, odometer, and fuel gauge methods to keep `drivePractice` clean and concise.

---

## Verification Plan

### Automated Tests
* Run `analyze_file` on `carDashboard.java` and `drivePractice.java` to confirm 0 compilation errors.

### Manual Verification
* Run `drivePractice` TeleOp on the robot.
* Confirm that battery fuel gauge, session feet, all-time feet, speed (IPS), and compass navigation display on the Driver Station telemetry screen.
