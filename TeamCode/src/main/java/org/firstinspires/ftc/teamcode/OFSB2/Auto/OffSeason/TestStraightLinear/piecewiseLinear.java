package org.firstinspires.ftc.teamcode.OFSB2.Auto.OffSeason.TestStraightLinear;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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
        FOLLOWING,
        DONE
    }

    private PathState pathState;

    private final Pose startingCoordinate = new Pose(129, 13, Math.toRadians(135));
    private final Pose path1complete = new Pose(13, 129, Math.toRadians(135));

    private PathChain startFinish;

    public void buildPaths() {
        startFinish = follower.pathBuilder()
                .addPath(new BezierLine(startingCoordinate, path1complete))
                .setTangentHeadingInterpolation()
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case START:
                follower.followPath(startFinish);
                setPathState(PathState.FOLLOWING);
                break;

            case FOLLOWING:
                if (!follower.isBusy()) {
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
        Pose pose = follower.getPose();
        if (pose != null) {
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading (Deg)", Math.toDegrees(pose.getHeading()));
        }
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
