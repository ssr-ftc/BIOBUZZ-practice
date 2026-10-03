package org.firstinspires.ftc.teamcode.OFSB2.Auto.Season.LM0;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
    Pose start, sample1, shoot, sample2, done;
    private PathChain startToSample1, sample1ToShoot, shootToSample2, sample2ToDone;

    private Timer pathTimer, opModeTimer;

    private enum AutoState {
        STARTTOSAMPLE1,
        SAMPLE1TOSHOOT,
        SHOOTTOSAMPLE2,
        SAMPLE2TODONE,
        DONE
    }

    private AutoState autoState;

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        poses();
        buildPaths();
        follower.setPose(start);

        autoState = AutoState.STARTTOSAMPLE1;
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

    // --- THE SINGLE STATE MACHINE ---
    private void updateAutoState() {
        // If the robot is currently driving a path, wait here and do nothing
        if (follower.isBusy()) {
            return;
        }

        // When the current path finishes, trigger the next path and update the state to match
        switch (autoState) {
            case STARTTOSAMPLE1:
                // Finished startToSample1 -> drive sample1ToShoot
                follower.followPath(startToSample1, true);
                setPathState(AutoState.SAMPLE1TOSHOOT);
                break;

            case SAMPLE1TOSHOOT:
                // Finished sample1ToShoot -> drive shootToSample2
                follower.followPath(sample1ToShoot, true);
                setPathState(AutoState.SHOOTTOSAMPLE2);
                break;

            case SHOOTTOSAMPLE2:
                // Finished shootToSample2 -> drive sample2ToDone
                follower.followPath(shootToSample2, true);
                setPathState(AutoState.SAMPLE2TODONE);
                break;

            case SAMPLE2TODONE:
                // Finished sample2ToDone
                follower.followPath(sample2ToDone, true);
                setPathState(AutoState.DONE);
                break;

            case DONE:
                telemetry.addLine("Finished everything!");
                break;

            default:
                break;
        }
    }

    // Helper method to set the state and reset the timer cleanly
    private void setPathState(AutoState newState) {
        autoState = newState;
        pathTimer.resetTimer();
    }

    private void poses() {
        start = new Pose(84.35853658536585, 134.31463414634146, Math.toRadians(0));
        sample1 = new Pose(126.23658536585364, 134.70487804878047, Math.toRadians(0));
        shoot = new Pose(90.10487804878049, 36.74390243902439, Math.toRadians(-90));
        sample2 = new Pose(92.42017114914424, 18.267344504740876, Math.toRadians(-90));
        done = new Pose(138.7166915141034, 33.84915618104835, Math.toRadians(180));
    }

    private void buildPaths() {
        startToSample1 = follower.pathBuilder()
                .addPath(new BezierLine(start, sample1))
                .setLinearHeadingInterpolation(start.getHeading(), sample1.getHeading())
                .build();

        sample1ToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(sample1, new Pose(125.05243902439025, 65.78780487804879), new Pose(112.61829268292684, 29.90365853658537), shoot))
                .setLinearHeadingInterpolation(sample1.getHeading(), shoot.getHeading())
                .build();

        shootToSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot, new Pose(94.88536585365854, 25.947560975609758), sample2))
                .setLinearHeadingInterpolation(shoot.getHeading(), sample2.getHeading())
                .build();

        sample2ToDone = follower.pathBuilder()
                .addPath(new BezierCurve(sample2, new Pose(98.33658536585365, 38.026829268292694), done))
                .setLinearHeadingInterpolation(sample2.getHeading(), done.getHeading())
                .build();
    }

    private void updateTelemetry() {
        telemetry.addData("auto state", autoState);
        Pose pose = follower.getPose();
        if (pose != null) {
            telemetry.addData("x", pose.getX());
            telemetry.addData("y", pose.getY());
            telemetry.addData("heading (deg)", Math.toDegrees(pose.getHeading()));
        }
        telemetry.addData("path time", pathTimer.getElapsedTimeSeconds());
    }
}
