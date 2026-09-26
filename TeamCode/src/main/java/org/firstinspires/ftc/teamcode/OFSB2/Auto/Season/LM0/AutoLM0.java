package org.firstinspires.ftc.teamcode.OFSB2.Auto.Season.LM0;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OFSB2.Auto.Constants;

@Autonomous(name = "AutoLM0", group = "Season")
public class AutoLM0 extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public final Pose start = new Pose(70.85831960461286, 134.72981878088962, Math.toRadians(90));
    public final Pose shootPos = new Pose(47.40449664771886, 70.25789384267753, Math.toRadians(90));
    public final Pose sample1 = new Pose(93.91480375429161, 40.852017956911546, Math.toRadians(360));
    public final Pose sample2 = new Pose(23.31136738056013, 47.088962108731465, Math.toRadians(55));
    public final Pose returnMid = new Pose(71.20195460911026, 99.05104624197372, Math.toRadians(55));
    public final Pose end = new Pose(70.85831960461286, 129.72981878088962, Math.toRadians(90));

    public PathChain toShoot;
    public PathChain shootToSample1;
    public PathChain sample1ToSample2;
    public PathChain sample2ToReturnMid;
    public PathChain returnMidToEnd;

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
                .addPath(new BezierLine(start, shootPos))
                .setLinearHeadingInterpolation(start.getHeading(), shootPos.getHeading())
                .build();
        shootToSample1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, sample1))
                .setLinearHeadingInterpolation(shootPos.getHeading(), sample1.getHeading())
                .build();
        sample1ToSample2 = follower.pathBuilder()
                .addPath(new BezierLine(sample1, sample2))
                .setLinearHeadingInterpolation(sample1.getHeading(), sample2.getHeading())
                .build();
        sample2ToReturnMid = follower.pathBuilder()
                .addPath(new BezierLine(sample2, returnMid))
                .setLinearHeadingInterpolation(sample2.getHeading(), returnMid.getHeading())
                .build();
        returnMidToEnd = follower.pathBuilder()
                .addPath(new BezierLine(returnMid, end))
                .setLinearHeadingInterpolation(returnMid.getHeading(), end.getHeading())
                .build();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(start);
        setAutoState(AutoState.DRIVE_TO_SHOOT);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setAutoState(autoState);
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
        follower.followPath(toShoot);
        setAutoState(AutoState.DRIVE_TO_SAMPLE_1);
    }

    private void handleDriveToSample1() {
        if (!follower.isBusy()) {
            follower.followPath(shootToSample1);
            setAutoState(AutoState.DRIVE_TO_SAMPLE_2);
        }
    }

    private void handleDriveToSample2() {
        if (!follower.isBusy()) {
            follower.followPath(sample1ToSample2);
            setAutoState(AutoState.DRIVE_TO_RETURN_MID);
        }
    }

    private void handleDriveToReturnMid() {
        if (!follower.isBusy()) {
            follower.followPath(sample2ToReturnMid);
            setAutoState(AutoState.DRIVE_TO_END);
        }
    }

    private void handleDriveToEnd() {
        if (!follower.isBusy()) {
            follower.followPath(returnMidToEnd);
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
        Pose pose = follower.getPose();
        if (pose != null) {
            telemetry.addData("x", pose.getX());
            telemetry.addData("y", pose.getY());
            telemetry.addData("heading", pose.getHeading());
        }
        telemetry.addData("path time", pathTimer.getElapsedTimeSeconds());
    }
}
