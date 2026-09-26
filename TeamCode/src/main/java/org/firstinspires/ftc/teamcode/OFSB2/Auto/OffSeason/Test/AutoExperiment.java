package org.firstinspires.ftc.teamcode.OFSB2.Auto.OffSeason.Test;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.pedropathing.api.Paths;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.OFSB2.Auto.Constants;
import org.firstinspires.ftc.teamcode.OFSB2.Subsystems.CustomFollower;

@Autonomous(name = "AutoExperiment", group = "Autonomous")
public class AutoExperiment extends OpMode {
    boolean selectedGamePadY = false;
    boolean selectedGamePadB = false;
    boolean pathStarted;
    boolean pathsBuilt = false;

    // --- MECHANISM HARDWARE ACTIVE (COMMENTED OUT) ---
    // DcMotorEx shooter;
    // DcMotor intake;
    // DcMotorEx leftPusher;
    // DcMotorEx rightPusher;

    CustomFollower follower;
    Timer pathTimer;
    Timer timer;
    StateMachine pathState;

    enum StateMachine {
        DRIVE_STARPOS_SHOOT_POS,
        SHOOT_TO_PREPICKUP,
        INTAKE,
        LAUNCH_PICKUP_1,
        SHOOT_TO_PREPICKUP2,
        INTAKE2,
        BUFFER,
        LAUNCH_PICKUP_2,
        SHOOT_TO_PREPICKUP3,
        INTAKE3,
        LAUNCH_PICKUP_3,
        FINISH
    }

    private Pose startPose, shootPose, preloadPose, intakePose, launchPose1;
    private Pose preloadPose2, intakePose2, buffer, launchPose2;
    private Pose preloadPose3, intakePose3, launchPose3;
    private Pose finishPose;
    private Path driveStartPosShootPos, shootToPrePickup, INTAKE,
            LAUNCH_PICKUP_1, SHOOT_TO_PREPICKUP2, INTAKEPOS2,
            Buffer, LAUNCHPOSE2, PRELOADPOSE3,
            INTAKEPOSE3, LAUNCHPOSE3, FINISH;

    @Override
    public void init() {
        pathState = StateMachine.DRIVE_STARPOS_SHOOT_POS;

        follower = new CustomFollower(hardwareMap, telemetry);
        follower.pedro.holdEnd.set(true);
        pathTimer = new Timer();
        timer = new Timer(); // Kept for pathing

        // --- MECHANISM MAPPING ACTIVE (COMMENTED OUT) ---
        // ... (Skipped mechanism init for brevity, same as your code) ...

        pathStarted = false;
    }

    @Override
    public void init_loop() {
        if (!pathsBuilt) {
            InitializePoseValues();
        }
    }

    @Override
    public void loop() {
        // This stays follower.update() because CustomFollower handles it
        follower.update();
        statePathUpdate();

        // handleIntake(); // Sensor / Intake logic active (COMMENTED OUT)

        telemetry.addData("STATE", pathState);

        // ADDED .pedro HERE
        Pose pose = follower.pedro.pose();
        if (pose != null) {
            telemetry.addData("x", pose.x());
            telemetry.addData("y", pose.y());
            telemetry.addData("heading", pose.heading());
        }

        // ADDED .pedro HERE
        telemetry.addData("followerBusy", follower.pedro.isBusy());
        telemetry.addData("pathTimer", pathTimer.seconds());
        telemetry.addData("Timer", timer.seconds());
        telemetry.update();
    }

    /* ---------------- INITIALIZE POSES ---------------- */
    private void InitializePoseValues() {
        if (gamepad1.y) {
            InitializePoseValues_GamePad_Y();
        }
        else if (gamepad1.b) {
            InitializePoseValues_GamePad_B();
        }
        else if (gamepad1.a) {
            InitializePoseValues_GamePad_A();
        }
        else if (gamepad1.x) {
            InitializePoseValues_GamePad_X();
        }
        else if (gamepad1.right_bumper && gamepad1.x) {
            telemetry.addLine("Backup selected ");
            telemetry.update();
        }
        else {
            telemetry.addLine("Select auto mode by pressing: " +
                    "X(Small blue) / Y(Big blue) / B(Big red) / A(Small red)");
            telemetry.update();
            return;
        }

        buildPaths();

        // ADDED .pedro HERE
        follower.pedro.setPose(startPose);
        pathsBuilt = true;
    }

    private void InitializePoseValues_GamePad_Y() {
        telemetry.addLine("Big blue selected");
        telemetry.update();

        startPose = new Pose(21.3509, 124.7842, Math.toRadians(138));
        shootPose = new Pose(49.45, 97.37, Math.toRadians(138));
        preloadPose = new Pose(47.4465, 85.4036, Math.toRadians(-180));
        intakePose = new Pose(24.3328, 91.5717, Math.toRadians(-180));
        launchPose1 = shootPose;
        preloadPose2 = new Pose(62.1203, 65.6352, Math.toRadians(-180));
        intakePose2 = new Pose(20.3180, 61.6804, Math.toRadians(-180));
        buffer = new Pose(54.12, 62.64, Math.toRadians(132));
        launchPose2 = shootPose;
        preloadPose3 = new Pose(61.2058, 37.8614, Math.toRadians(-180));
        intakePose3 = new Pose(19.4530, 43.6507, Math.toRadians(-180));
        launchPose3 = shootPose;
        finishPose = new Pose(35.1104, 71.4069, Math.toRadians(-180));
        selectedGamePadY = true;
    }

    private void InitializePoseValues_GamePad_B() {
        telemetry.addLine("Big red selected");
        telemetry.update();

        startPose = new Pose(120.64909390444811, 122.7841845140033, Math.toRadians(40));
        shootPose = new Pose(94.55354200988468, 93.36738056013179, Math.toRadians(40));
        preloadPose = new Pose(96.55354200988468, 87.4036243822076, Math.toRadians(0));

        intakePose = new Pose(135.0, 90.92, Math.toRadians(0));

        launchPose1 = new Pose(96.55354200988468, 97.36738056013179, Math.toRadians(40));
        preloadPose2 = new Pose(88.56, 58.64000000000001, Math.toRadians(0));

        intakePose2 = new Pose(135.0, 66.599999999999994, Math.toRadians(0));

        buffer = new Pose(86.88, 62.64, Math.toRadians(48));
        launchPose2 = new Pose(96.55354200988468, 95.36738056013179, Math.toRadians(40));
        preloadPose3 = new Pose(90.79423876953126, 37.861437866210935, Math.toRadians(0));

        intakePose3 = new Pose(135.0, 42.6, Math.toRadians(0));

        launchPose3 = new Pose(96.55354200988468, 95.36738056013179, Math.toRadians(40));
        finishPose = new Pose(108.88962108731467, 71.40691927512356, Math.toRadians(0));
        selectedGamePadB = true;
    }

    private void InitializePoseValues_GamePad_A() {
        telemetry.addLine("Small red selected");
        telemetry.update();

        startPose = new Pose(78.04444444444445, 8, Math.toRadians(90));
        shootPose = new Pose(77.68888888888888, 17.333333333333332, Math.toRadians(66));
        finishPose = new Pose(100.51111111111112, 25.77777777777764, Math.toRadians(0));
    }

    private void InitializePoseValues_GamePad_X() {
        telemetry.addLine("Small blue selected");
        telemetry.update();

        startPose = new Pose(65.95555555555555, 8, Math.toRadians(90));
        shootPose = new Pose(66.31111111111112, 17.333333333333332, Math.toRadians(114));
        finishPose = new Pose(48.48888888888888, 9.777777777777764, Math.toRadians(180));
    }

    private void buildPaths() {
        // 0. Common Path: Start -> Shoot Preloads
        driveStartPosShootPos = Paths.line(startPose, shootPose)
                .linear(startPose.heading(), shootPose.heading());

        if (selectedGamePadY || selectedGamePadB) {
            // 2. First Ball -> Shoot
            INTAKE = Paths.line(shootPose, intakePose)
                    .linear(shootPose.heading(), intakePose.heading());

            // 3. Shoot -> Second Ball
            LAUNCH_PICKUP_1 = Paths.line(intakePose, launchPose1)
                    .linear(intakePose.heading(), launchPose1.heading());

            // 5. Shoot -> Third Ball
            SHOOT_TO_PREPICKUP2 = Paths.line(launchPose1, preloadPose2)
                    .linear(launchPose1.heading(), preloadPose2.heading());

            INTAKEPOS2 = Paths.line(preloadPose2, intakePose2)
                    .linear(preloadPose2.heading(), intakePose2.heading());

            Buffer = Paths.line(intakePose2, buffer)
                    .linear(intakePose2.heading(), buffer.heading());

            LAUNCHPOSE2 = Paths.line(buffer, launchPose2)
                    .linear(buffer.heading(), launchPose2.heading());

            PRELOADPOSE3 = Paths.line(launchPose2, preloadPose3)
                    .linear(launchPose2.heading(), preloadPose3.heading());

            INTAKEPOSE3 = Paths.line(preloadPose3, intakePose3)
                    .linear(preloadPose3.heading(), intakePose3.heading());

            LAUNCHPOSE3 = Paths.line(intakePose3, launchPose3)
                    .linear(intakePose3.heading(), launchPose3.heading());

            FINISH = Paths.line(launchPose3, finishPose)
                    .linear(launchPose3.heading(), finishPose.heading());

        } else {
            FINISH = Paths.line(shootPose, finishPose)
                    .linear(shootPose.heading(), finishPose.heading());
        }
    }

    private void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARPOS_SHOOT_POS:
                timer.reset();
                boolean isSmallTriangle = !selectedGamePadY && !selectedGamePadB;
                startPath(driveStartPosShootPos);

                if (!follower.pedro.isBusy()) {
                    if (isSmallTriangle) {
                        setPathState(StateMachine.FINISH);
                    } else {
                        setPathState(StateMachine.SHOOT_TO_PREPICKUP);
                    }
                }
                break;

            case SHOOT_TO_PREPICKUP:
                startPath(shootToPrePickup);
                if (!follower.pedro.isBusy()) {
                    setPathState(StateMachine.INTAKE);
                }
                break;

            case INTAKE:
                startPath(INTAKE);
                if (!follower.pedro.isBusy()) {
                    setPathState(StateMachine.LAUNCH_PICKUP_1);
                }
                break;

            case LAUNCH_PICKUP_1:
                startPath(LAUNCH_PICKUP_1);
                if (!follower.pedro.isBusy()) {
                    setPathState(StateMachine.SHOOT_TO_PREPICKUP2);
                }
                break;

            case SHOOT_TO_PREPICKUP2:
                startPath(SHOOT_TO_PREPICKUP2);
                if (!follower.pedro.isBusy()) {
                    setPathState(StateMachine.INTAKE2);
                }
                break;

            case INTAKE2:
                startPath(INTAKEPOS2);
                if (!follower.pedro.isBusy()) {
                    setPathState(StateMachine.BUFFER);
                }
                break;

            case BUFFER:
                startPath(Buffer);
                if (!follower.pedro.isBusy()) {
                    setPathState(StateMachine.LAUNCH_PICKUP_2);
                }
                break;

            case LAUNCH_PICKUP_2:
                startPath(LAUNCHPOSE2);
                if (!follower.pedro.isBusy()) {
                    setPathState(StateMachine.SHOOT_TO_PREPICKUP3);
                }
                break;

            case SHOOT_TO_PREPICKUP3:
                startPath(PRELOADPOSE3);
                if (!follower.pedro.isBusy()) {
                    setPathState(StateMachine.INTAKE3);
                }
                break;

            case INTAKE3:
                startPath(INTAKEPOSE3);
                if (!follower.pedro.isBusy()) {
                    setPathState(StateMachine.LAUNCH_PICKUP_3);
                }
                break;

            case LAUNCH_PICKUP_3:
                startPath(LAUNCHPOSE3);
                if (!follower.pedro.isBusy()) {
                    setPathState(StateMachine.FINISH);
                }
                break;

            case FINISH:
                startPath(FINISH);
                break;

            default:
                telemetry.addLine("Error: Cannot Start State Machine");
                break;
        }
    }

    private void startPath(Path path) {
        if (!pathStarted) {
            follower.pedro.follow(path);
            pathStarted = true;
            pathTimer.reset();
        }
    }

    private void setPathState(StateMachine newState) {
        pathState = newState;
        pathStarted = false;
        pathTimer.reset();
    }
}
