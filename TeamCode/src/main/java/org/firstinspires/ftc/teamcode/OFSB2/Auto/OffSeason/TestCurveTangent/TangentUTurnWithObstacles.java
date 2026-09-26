package org.firstinspires.ftc.teamcode.OFSB2.Auto.OffSeason.TestCurveTangent;

import org.firstinspires.ftc.teamcode.OFSB2.Subsystems.CustomFollower;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.api.Paths;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "TangentUTurnWithObstacles", group = "Autonomous")
public class TangentUTurnWithObstacles extends OpMode {

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

    private final Pose startingCoordinate = new Pose(102, 11, Math.toRadians(90));
    private final Pose endingCoordinate = new Pose(47, 45, 0);

    public void buildPaths() {
        fullLoop = com.pedropathing.api.Paths.curve(
                        startingCoordinate,       // Start
                        new Pose(107, 128, 0), //100,121
                        new Pose (72, 135, 0), // 81, 137
                        new Pose (42 , 131, 0),            // Control 3
                        endingCoordinate        // End
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
                    setPathState(PathState.DONE);
                }
                break;
            case DONE:
                if (!follower.pedro.isBusy()) {
                    telemetry.addLine("Autonomous Loop Finished Successfully!");
                }
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
