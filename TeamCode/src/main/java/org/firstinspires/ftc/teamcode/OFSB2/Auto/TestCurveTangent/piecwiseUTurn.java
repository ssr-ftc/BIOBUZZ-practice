package org.firstinspires.ftc.teamcode.OFSB2.Auto.TestCurveTangent;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OFSB2.Auto.Constants;

@Autonomous(name = "PiecewiseTest", group = "Autonomous")
public class piecwiseUTurn extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        START_TO_END_LOOP,
        FOLLOWING_LOOP,
        DONE
    }

    PathState pathState;

    // --- COORDINATE DEFINITIONS ---
    private final Pose startingCoordinate = new Pose(18.8794038, 125.764044, Math.toRadians(330));
    private final Pose path1complete = new Pose(38.9827018, 55.3484349);
    private final Pose path2complete = new Pose(55.5683, 126.822446);
    private final Pose path3complete = new Pose(39.6266277, 104.602350);

    private PathChain fullLoop;

    public void buildPaths() {
        fullLoop = follower.pathBuilder(Constants.pathConstraints)
                // PATH 1: Complex S-Curve (5 points)
                .addPath(new BezierCurve(
                        startingCoordinate,
                        new Pose(71.7663, 96.3632),
                        new Pose(4.74546, 96.5288),
                        new Pose(35.11779, 79.0943),
                        path1complete
                ))
                // PATH 2: Large Loop (4 points)
                .addPath(new BezierCurve(
                        path1complete,
                        new Pose(55.5683, 30.8255),
                        new Pose(81.52341, 88.7543),
                        path2complete
                ))
                // PATH 3: Tight Curve (3 points)
                .addPath(new BezierCurve(
                        path2complete,
                        new Pose(47.6637, 140.2264),
                        path3complete
                ))
                // PATH 4: Return Loop (4 points)
                .addPath(new BezierCurve(
                        path3complete,
                        new Pose(10.91652, 76.6889),
                        new Pose(34.3444, 117.3792),
                        startingCoordinate
                ))
                // Piecewise Heading Strategy (Nose-steering then spinning)
                .setGlobalHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, 0.702, HeadingInterpolator.tangent),
                        new HeadingInterpolator.PiecewiseNode(0.702, 0.811,
                                HeadingInterpolator.linear(Math.toRadians(126), Math.toRadians(210))),
                        new HeadingInterpolator.PiecewiseNode(0.811, 1.0,
                                HeadingInterpolator.linear(Math.toRadians(210), Math.toRadians(330)))
                ))
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case START_TO_END_LOOP:
                follower.followPath(fullLoop, true);
                setPathState(PathState.FOLLOWING_LOOP);
                break;

            case FOLLOWING_LOOP:
                // Safeguard: Ensure robot has moved past the start (T > 0.1) before allowed to finish
                if (follower.getCurrentTValue() > 0.1 && follower.atParametricEnd()) {
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                if (!follower.isBusy()) {
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

        // Essential telemetry to track the loop progress
        telemetry.addData("State", pathState);
        telemetry.addData("T Value", follower.getCurrentTValue());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}
