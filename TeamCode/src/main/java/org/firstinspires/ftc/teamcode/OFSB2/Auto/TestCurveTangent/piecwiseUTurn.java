package org.firstinspires.ftc.teamcode.OFSB2.Auto.TestCurveTangent;

// 1. ADDED IMPORT: Bring in your CustomFollower
import org.firstinspires.ftc.teamcode.OFSB2.Subsystems.CustomFollower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "piecewiseUTurn", group = "Autonomous")
public class piecwiseUTurn extends OpMode {

    // 2. CHANGED THIS: Declare it as your CustomFollower
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
    private PathChain fullLoop;

    private final Pose startingCoordinate = new Pose(94.0, 10.0, Math.toRadians(90));

    public void buildPaths() {
        // 3. ADDED .pedro: Route standard builder commands through the Pedro object
        fullLoop = follower.pedro.pathBuilder()
                .addPath(new BezierCurve(new Pose(94.0, 10.0), new Pose(100.0, 121.0), new Pose(81.0, 137.0), new Pose(44.0, 132.0), new Pose(47.0, 37.0)))
                .setTangentHeadingInterpolation()
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case START_TO_END_LOOP:
                // 4. ADDED .pedro: Tell Pedro to follow the path
                follower.pedro.followPath(fullLoop, true);

                // You put this in the exact right spot!
                follower.enableAutoPhysics(0.7, 80.0);

                setPathState(PathState.FOLLOWING1);
                break;

            case FOLLOWING1:
                // 5. ADDED .pedro: Check if Pedro is busy
                if (!follower.pedro.isBusy()) {
                    setPathState(PathState.FOLLOWING_LOOP);
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
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        // 6. CHANGED THIS: Initialize CustomFollower and pass in the telemetry
        follower = new CustomFollower(hardwareMap, telemetry);

        buildPaths();

        // 7. ADDED .pedro: Set the starting pose
        follower.pedro.setPose(startingCoordinate);

        pathState = PathState.START_TO_END_LOOP;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        // NOTE: We do NOT add .pedro here, because we want to run YOUR update loop!
        follower.update();

        statePathUpdate();

        telemetry.addData("State", pathState);

        // 8. ADDED .pedro: Route the getters through Pedro to get your data
        telemetry.addData("T Value", follower.pedro.getCurrentTValue());
        telemetry.addData("X", follower.pedro.getPose().getX());
        telemetry.addData("Y", follower.pedro.getPose().getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.pedro.getPose().getHeading()));
        telemetry.update();
    }
}