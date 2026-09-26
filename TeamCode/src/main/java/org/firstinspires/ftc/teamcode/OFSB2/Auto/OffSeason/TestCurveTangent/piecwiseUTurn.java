package org.firstinspires.ftc.teamcode.OFSB2.Auto.OffSeason.TestCurveTangent;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OFSB2.Auto.Constants;

@Autonomous(name = "piecewiseUTurn", group = "Autonomous")
public class piecwiseUTurn extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        START_TO_END_LOOP,
        FOLLOWING1,
        DONE
    }

    private PathState pathState;
    private PathChain fullLoop;

    private final Pose startingCoordinate = new Pose(94.0, 10.0, Math.toRadians(90));

    public void buildPaths() {
        fullLoop = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                startingCoordinate,
                                new Pose(100.0, 121.0),
                                new Pose(81.0, 137.0),
                                new Pose(44.0, 132.0),
                                new Pose(47.0, 37.0)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case START_TO_END_LOOP:
                follower.followPath(fullLoop);
                setPathState(PathState.FOLLOWING1);
                break;

            case FOLLOWING1:
                if (!follower.isBusy()) {
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                telemetry.addLine("Autonomous Loop Finished Successfully!");
                break;

            default:
                telemetry.addLine("State machine logic error");
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

        pathState = PathState.START_TO_END_LOOP;
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
        telemetry.update();
    }
}
