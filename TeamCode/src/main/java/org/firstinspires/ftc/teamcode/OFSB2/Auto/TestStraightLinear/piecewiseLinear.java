package org.firstinspires.ftc.teamcode.OFSB2.Auto.TestStraightLinear;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OFSB2.Auto.Constants;

@Autonomous(name = "piecewiseLinear", group = "Autonomous")
public class piecewiseLinear extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        START,
        DONE
    }

    private PathState pathState;

    private final Pose startingCoordinate = new Pose(129, 13, Math.toRadians(135));
    private final Pose path1complete = new Pose(13, 129, Math.toRadians(135));

    private PathChain startFinish;

    public void buildPaths() {
        startFinish = follower.pathBuilder()
                .addPath(new BezierLine(startingCoordinate, path1complete))
                // Commented out so they don't override the smooth acceleration in the loop:
                // .addParametricCallback(0.0, () -> follower.setMaxPower(0.25))
                // .addParametricCallback(0.3, () -> follower.setMaxPower(0.5))
                // .addParametricCallback(0.6, () -> follower.setMaxPower(0.75))
                // .addParametricCallback(0.9, () -> follower.setMaxPower(1))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0, 1, HeadingInterpolator.tangent)))
                /* new HeadingInterpolator.PiecewiseNode(
                        .5, 1, HeadingInterpolator.tangent)))

                 */


                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case START:
                follower.followPath(startFinish, true);
                setPathState(PathState.DONE);
                break;
            case DONE:
                if (!follower.isBusy()) {
                    telemetry.addLine("Fully Linear Piecewise Loop Finished!");
                }
                break;
            default:
                telemetry.addLine("State machine error");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startingCoordinate);

        pathState = PathState.START;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        // --- SMOOTH ACCELERATION LOGIC ADDED HERE ---
        if (follower.isBusy()) {
            double currentT = follower.getCurrentTValue();

            double startT = 0.4;
            double endT = 1.0;
            double startPower = 1.0;
            double endPower = 0.25;

            if (currentT >= startT && currentT <= endT) {
                double progress = (currentT - startT) / (endT - startT);
                double smoothPower = startPower + (progress * (endPower - startPower));
                follower.setMaxPower(smoothPower);
            } else if (currentT > endT) {
                follower.setMaxPower(endPower);
            }
        }
        // --------------------------------------------

        telemetry.addData("State", pathState);
        telemetry.addData("T Value", follower.getCurrentTValue());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}