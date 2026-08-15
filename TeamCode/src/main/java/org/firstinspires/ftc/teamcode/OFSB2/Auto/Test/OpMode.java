package org.firstinspires.ftc.teamcode.OFSB2.Auto.Test;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Pedro Pathing Test", group = "OFSB2")
public class OpMode extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // --- Trajectories ---
    public Pose startPose = new Pose(0, 0, 0);
    private PathChain toShoot, shootToSample1, sample1ToSample2, sample2ToReturnMid, returnMidToEnd;

    private enum AutoState {
        DRIVE_TO_SHOOT,
        DRIVE_TO_SAMPLE_1,
        DRIVE_TO_SAMPLE_2,
        DRIVE_TO_RETURN_MID,
        DRIVE_TO_END,
        DONE
    }

    private AutoState autoState;

    public void buildPaths() {
        toShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, new Pose(10, 10, 0)))
                .build();
        shootToSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(10, 10, 0), new Pose(20, 20, 0)))
                .build();
        sample1ToSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(20, 20, 0), new Pose(30, 30, 0)))
                .build();
        sample2ToReturnMid = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(30, 30, 0), new Pose(20, 20, 0)))
                .build();
        returnMidToEnd = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(20, 20, 0), new Pose(0, 0, 0)))
                .build();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
        setAutoState(AutoState.DRIVE_TO_SHOOT);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        updateAutoState();
        updateTelemetry();
    }

    private void updateAutoState() {
        switch (autoState) {
            case DRIVE_TO_SHOOT:
                handleDriveToShoot();
                break;
            case DRIVE_TO_SAMPLE_1:
                handleDriveToSample1();
                break;
            case DRIVE_TO_SAMPLE_2:
                handleDriveToSample2();
                break;
            case DRIVE_TO_RETURN_MID:
                handleDriveToReturnMid();
                break;
            case DRIVE_TO_END:
                handleDriveToEnd();
                break;
            case DONE:
                handleDone();
                break;
            default:
                telemetry.addLine("State machine not working");
                break;
        }
    }

    private void handleDriveToShoot() {
        follower.followPath(toShoot, true);
        setAutoState(AutoState.DRIVE_TO_SAMPLE_1);
    }

    private void handleDriveToSample1() {
        if (!follower.isBusy()) {
            follower.followPath(shootToSample1, true);
            setAutoState(AutoState.DRIVE_TO_SAMPLE_2);
        }
    }

    private void handleDriveToSample2() {
        if (!follower.isBusy()) {
            follower.followPath(sample1ToSample2, true);
            setAutoState(AutoState.DRIVE_TO_RETURN_MID);
        }
    }

    private void handleDriveToReturnMid() {
        if (!follower.isBusy()) {
            follower.followPath(sample2ToReturnMid, true);
            setAutoState(AutoState.DRIVE_TO_END);
        }
    }

    private void handleDriveToEnd() {
        if (!follower.isBusy()) {
            follower.followPath(returnMidToEnd, true);
            setAutoState(AutoState.DONE);
        }
    }

    private void handleDone() {
        if (!follower.isBusy()) {
            telemetry.addLine("Finished everything!");
        }
    }

    private void setAutoState(AutoState newState) {
        autoState = newState;
        pathTimer.resetTimer();
    }

    private void updateTelemetry() {
        telemetry.addData("auto state", autoState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
