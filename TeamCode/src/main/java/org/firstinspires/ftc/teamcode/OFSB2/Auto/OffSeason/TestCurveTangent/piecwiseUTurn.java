package org.firstinspires.ftc.teamcode.OFSB2.Auto.OffSeason.TestCurveTangent;

import org.firstinspires.ftc.teamcode.OFSB2.Subsystems.CustomFollower;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.api.Paths;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "piecewiseUTurn", group = "Autonomous")
public class piecwiseUTurn extends OpMode {

    private CustomFollower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        START_TO_END_LOOP,
        FOLLOWING1,
        FOLLOWING_LOOP,
        FOLLOWING2,
        DONE
    }

    private PathState pathState;
    private Path fullLoop;

    private final Pose startingCoordinate = new Pose(94.0, 10.0, Math.toRadians(90));

    public void buildPaths() {
        fullLoop = com.pedropathing.api.Paths.curve(
                        new Pose(94.0, 10.0, 0),
                        new Pose(100.0, 121.0, 0),
                        new Pose(81.0, 137.0, 0),
                        new Pose(44.0, 132.0, 0),
                        new Pose(47.0, 37.0, 0)
                )
                .tangent();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case START_TO_END_LOOP:
                follower.pedro.follow(fullLoop);

                follower.autoAcceleration(0.7, 80.0);

                setPathState(PathState.FOLLOWING1);
                break;

            case FOLLOWING1:
                if (!follower.pedro.isBusy()) {
                    setPathState(PathState.FOLLOWING_LOOP);
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
        pathTimer.reset();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = new CustomFollower(hardwareMap, telemetry);
        follower.pedro.holdEnd.set(true);

        buildPaths();

        follower.pedro.setPose(startingCoordinate);

        pathState = PathState.START_TO_END_LOOP;
    }

    @Override
    public void start() {
        opModeTimer.reset();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();

        statePathUpdate();

        telemetry.addData("State", pathState);

        telemetry.addData("T Value", follower.pedro.parametricCompletion());
        Pose pose = follower.pedro.pose();
        telemetry.addData("X", pose.x());
        telemetry.addData("Y", pose.y());
        telemetry.addData("Heading (Deg)", Math.toDegrees(pose.heading()));
        telemetry.update();
    }
}
