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
        END,
        DONE
    }

    PathState pathState;

    private final Pose startingCoordinate = new Pose(72, 7, Math.toRadians(90));
    private final Pose path1complete = new Pose(72, 100, Math.toRadians(180));

    private PathChain start_finish; 

    public void buildPaths() {
        start_finish = follower.pathBuilder()
                .addPath(new BezierLine(startingCoordinate, path1complete))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0, .5, HeadingInterpolator.tangent),
                        new HeadingInterpolator.PiecewiseNode(
                                .5, 1, HeadingInterpolator.linear(Math.toRadians(90), Math.toRadians(180)))))
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case START:
                follower.followPath(start_finish, true);
                setPathState(PathState.DONE);
                break;
/*
            case END:
                // Safeguard: Ensure robot has moved past the start (T > 0.1) before allowed to finish
                if (follower.getCurrentTValue() > 0.1 && follower.atParametricEnd()) {
                    setPathState(PathState.DONE);
                }
                break;
                */
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

        telemetry.addData("State", pathState);
        telemetry.addData("T Value", follower.getCurrentTValue());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
