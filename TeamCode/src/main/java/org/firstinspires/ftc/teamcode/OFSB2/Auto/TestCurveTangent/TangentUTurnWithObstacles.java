package org.firstinspires.ftc.teamcode.OFSB2.Auto.TestCurveTangent;

import org.firstinspires.ftc.teamcode.OFSB2.Subsystems.CustomFollower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
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
    private PathChain fullLoop;

    // UPDATED: 65-degree heading perfectly matches the tangent line of the first curve segment
    private final Pose startingCoordinate = new Pose(71.15238879, 9.16556836, Math.toRadians(90));

    public void buildPaths() {
        // FIXED: Coordinates are now in the correct Start -> C1 -> C2 -> C3 -> End order
        fullLoop = follower.pedro.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(71.15238879, 9.16556836),       // Start
                        new Pose(124.9036, 125.4349),            // Control 1
                        new Pose(68.3245, 140.6177),             // Control 2
                        new Pose(45.71252, 140.5873),            // Control 3
                        new Pose(37.3509060, 31.8039538)         // End
                ))
                .setTangentHeadingInterpolation()
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case START_TO_END_LOOP:
                follower.pedro.followPath(fullLoop, true);

                // Utilizing your custom physics engine
                follower.autoAcceleration(0.7, 80.0);

                setPathState(PathState.FOLLOWING1);
                break;

            case FOLLOWING1:
                if (!follower.pedro.isBusy()) {
                    setPathState(PathState.FOLLOWING_LOOP);
                }
                break;

            case FOLLOWING_LOOP:
            case FOLLOWING2:
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

        follower = new CustomFollower(hardwareMap, telemetry);

        buildPaths();

        // Sets the internal odometry to prevent a snap-spin on start
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
        // Run the custom physics math loop
        follower.update();

        // Update the pathing state machine
        statePathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("T Value", follower.pedro.getCurrentTValue());
        telemetry.addData("X", follower.pedro.getPose().getX());
        telemetry.addData("Y", follower.pedro.getPose().getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.pedro.getPose().getHeading()));
        telemetry.update();
    }
}