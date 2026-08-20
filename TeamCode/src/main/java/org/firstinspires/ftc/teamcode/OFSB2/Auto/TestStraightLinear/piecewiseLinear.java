package org.firstinspires.ftc.teamcode.OFSB2.Auto.TestStraightLinear;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OFSB2.Subsystems.CustomFollower;

@Autonomous(name = "piecewiseLinear", group = "Autonomous")
public class piecewiseLinear extends OpMode {

    private CustomFollower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        START,
        FOLLOWING,
        DONE
    }

    private PathState pathState;

    private final Pose startingCoordinate =
            new Pose(129, 13, Math.toRadians(135));

    private final Pose path1complete =
            new Pose(13, 129, Math.toRadians(135));

    private PathChain startFinish;

    public void buildPaths() {
        startFinish = follower.pedro.pathBuilder()
                .addPath(new BezierLine(startingCoordinate, path1complete))
                .setHeadingInterpolation(

                HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, 1, HeadingInterpolator.tangent)))
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case START:
                follower.pedro.followPath(startFinish, true);

                    follower.acceleration(0, 0.5, 0, 1);
                    follower.acceleration(0.5, 1, 1, 0);

                    setPathState(PathState.FOLLOWING);


                break;

            case FOLLOWING:

                if (!follower.pedro.isBusy()) {
                    setPathState(PathState.DONE);
                }

                break;

            case DONE:
                    telemetry.addLine("Fully Linear Piecewise Loop Finished!");
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

        follower = new CustomFollower(hardwareMap, telemetry);

        buildPaths();
        follower.pedro.setPose(startingCoordinate);

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

        telemetry.addData("State", pathState);
        telemetry.addData("T Value", follower.pedro.getCurrentTValue());
        telemetry.addData("X", follower.pedro.getPose().getX());
        telemetry.addData("Y", follower.pedro.getPose().getY());
        telemetry.addData(
                "Heading (Deg)",
                Math.toDegrees(follower.pedro.getPose().getHeading())
        );
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}