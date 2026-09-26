package org.firstinspires.ftc.teamcode.OFSB2.Auto.OffSeason.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OFSB2.Auto.Constants;

@Autonomous(name = "AutoExperimentCurve", group = "Autonomous")
public class AutoExperimentCurve extends OpMode {
    boolean selectedGamePadY = false;
    boolean selectedGamePadB = false;
    boolean pathStarted;
    boolean pathsBuilt = false;

    Follower follower;
    Timer pathTimer;
    Timer timer;
    StateMachine pathState;

    enum StateMachine {
        DRIVE_STARPOS_SHOOT_POS,
        INTAKE,
        LAUNCH_PICKUP_1,
        INTAKE2,
        LAUNCH_PICKUP_2,
        INTAKE3,
        LAUNCH_PICKUP_3,
        GATE,
        LAUNCH_GATE,
        FINISH
    }

    private Pose startPose, shootPose, intakePose, launchPose1, intakePose2, launchPose2, intakePose3, launchPose3, gatePose, launchGatePose, finishPose;
    private PathChain driveStartPosShootPos, INTAKE,
            LAUNCH_PICKUP_1, INTAKEPOSE2, LAUNCHPOSE2,
            INTAKEPOSE3, LAUNCHPOSE3, GATEPOSE, LAUNCHGATEPOSE, FINISH;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(25);

        pathState = StateMachine.DRIVE_STARPOS_SHOOT_POS;

        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();
        timer = new Timer();

        pathStarted = false;
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (!pathsBuilt) {
            InitializePoseValues();
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("STATE", pathState);

        Pose pose = follower.getPose();
        if (pose != null) {
            telemetry.addData("x", pose.getX());
            telemetry.addData("y", pose.getY());
            telemetry.addData("heading", pose.getHeading());
        }

        telemetry.addData("followerBusy", follower.isBusy());
        telemetry.addData("pathTimer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Timer", timer.getElapsedTimeSeconds());

        telemetry.update();
    }

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
        else {
            telemetry.addLine("Select auto mode by pressing: " +
                    "X(Small blue) / Y(Big blue) / B(Big red) / A(Small red)");
            telemetry.update();
            return;
        }

        buildPaths();

        follower.setPose(startPose);
        pathsBuilt = true;
    }

    private void InitializePoseValues_GamePad_Y() {
        telemetry.addLine("Big blue selected");

        startPose = new Pose(20.7539489, 119.41190765, Math.toRadians(135));
        shootPose = new Pose(58.0631834, 82.76549211, Math.toRadians(180));
        intakePose = new Pose(18.5370595, 82.6992709, Math.toRadians(180));
        launchPose1 = new Pose(58.0631834, 82.7654921, Math.toRadians(-126));
        intakePose2 = new Pose(17.7089914945322, 70.28493317132447);
        launchPose2 = new Pose(58.0631834, 82.7654921, Math.toRadians(75));
        intakePose3 = new Pose(16.50546780072904, 52.73025516403404);
        launchPose3 = shootPose;
        gatePose = new Pose(17.06771633, 68.5968041, Math.toRadians(180));
        launchGatePose = new Pose(58.0631834, 82.7654921, Math.toRadians(180));
        finishPose = new Pose(23.161211296, 68.88116391, Math.toRadians(180));

        selectedGamePadY = true;
    }

    private void InitializePoseValues_GamePad_B() {
        telemetry.addLine("Big red selected");
        telemetry.update();

        startPose = new Pose(120.64909390444811, 122.7841845140033, Math.toRadians(40));
        shootPose = new Pose(94.55354200988468, 93.36738056013179, Math.toRadians(40));
        intakePose = new Pose(135.0, 90.92, Math.toRadians(0));
        launchPose1 = new Pose(96.55354200988468, 97.36738056013179, Math.toRadians(40));
        intakePose2 = new Pose(135.0, 66.599999999999994, Math.toRadians(0));
        launchPose2 = new Pose(96.55354200988468, 95.36738056013179, Math.toRadians(40));
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
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        if (selectedGamePadY || selectedGamePadB) {
            INTAKE = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, intakePose))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), intakePose.getHeading())
                    .build();

            LAUNCH_PICKUP_1 = follower.pathBuilder()
                    .addPath(new BezierLine(intakePose, launchPose1))
                    .setLinearHeadingInterpolation(intakePose.getHeading(), launchPose1.getHeading())
                    .build();

            INTAKEPOSE2 = follower.pathBuilder()
                    .addPath(new BezierCurve(launchPose1, new Pose(40.4777, 70.7287), intakePose2))
                    .setLinearHeadingInterpolation(launchPose1.getHeading(), intakePose2.getHeading())
                    .build();

            LAUNCHPOSE2 = follower.pathBuilder()
                    .addPath(new BezierCurve(intakePose2, new Pose(40.4777, 58.7287), launchPose2))
                    .setLinearHeadingInterpolation(intakePose2.getHeading(), launchPose2.getHeading())
                    .build();

            INTAKEPOSE3 = follower.pathBuilder()
                    .addPath(new BezierCurve(launchPose2, new Pose(45.7339, 52.5583), intakePose3))
                    .setLinearHeadingInterpolation(launchPose2.getHeading(), intakePose3.getHeading())
                    .build();

            LAUNCHPOSE3 = follower.pathBuilder()
                    .addPath(new BezierCurve(intakePose3, new Pose(45.7339, 34.5583), launchPose3))
                    .setLinearHeadingInterpolation(intakePose3.getHeading(), launchPose3.getHeading())
                    .build();

            GATEPOSE = follower.pathBuilder()
                    .addPath(new BezierLine(launchPose3, gatePose))
                    .setLinearHeadingInterpolation(launchPose3.getHeading(), gatePose.getHeading())
                    .build();

            LAUNCHGATEPOSE = follower.pathBuilder()
                    .addPath(new BezierLine(gatePose, launchGatePose))
                    .setLinearHeadingInterpolation(gatePose.getHeading(), launchGatePose.getHeading())
                    .build();

            FINISH = follower.pathBuilder()
                    .addPath(new BezierLine(gatePose, finishPose))
                    .setLinearHeadingInterpolation(launchPose3.getHeading(), finishPose.getHeading())
                    .build();

        } else {
            FINISH = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, finishPose))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), finishPose.getHeading())
                    .build();
        }
    }

    private void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARPOS_SHOOT_POS:
                timer.resetTimer();
                boolean isSmallTriangle = !selectedGamePadY && !selectedGamePadB;
                startPath(driveStartPosShootPos);

                if (!follower.isBusy()) {
                    if (isSmallTriangle) {
                        setPathState(StateMachine.FINISH);
                    } else {
                        setPathState(StateMachine.INTAKE);
                    }
                }
                break;

            case INTAKE:
                startPath(INTAKE);
                if (!follower.isBusy()) {
                    setPathState(StateMachine.LAUNCH_PICKUP_1);
                }
                break;

            case LAUNCH_PICKUP_1:
                startPath(LAUNCH_PICKUP_1);
                if (!follower.isBusy()) {
                    setPathState(StateMachine.INTAKE2);
                }
                break;

            case INTAKE2:
                startPath(INTAKEPOSE2);
                if (!follower.isBusy()) {
                    setPathState(StateMachine.LAUNCH_PICKUP_2);
                }
                break;

            case LAUNCH_PICKUP_2:
                startPath(LAUNCHPOSE2);
                if (!follower.isBusy()) {
                    setPathState(StateMachine.INTAKE3);
                }
                break;

            case INTAKE3:
                startPath(INTAKEPOSE3);
                if (!follower.isBusy()) {
                    setPathState(StateMachine.LAUNCH_PICKUP_3);
                }
                break;

            case LAUNCH_PICKUP_3:
                startPath(LAUNCHPOSE3);
                if (!follower.isBusy()) {
                    setPathState(StateMachine.GATE);
                }
                break;

            case GATE:
                startPath(GATEPOSE);
                if (!follower.isBusy()) {
                    setPathState(StateMachine.LAUNCH_GATE);
                }
                break;

            case LAUNCH_GATE:
                startPath(LAUNCHGATEPOSE);
                if (!follower.isBusy()) {
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

    private void startPath(PathChain path) {
        if (!pathStarted && path != null) {
            follower.followPath(path);
            pathStarted = true;
            pathTimer.resetTimer();
        }
    }

    private void setPathState(StateMachine newState) {
        pathState = newState;
        pathStarted = false;
        pathTimer.resetTimer();
    }
}
