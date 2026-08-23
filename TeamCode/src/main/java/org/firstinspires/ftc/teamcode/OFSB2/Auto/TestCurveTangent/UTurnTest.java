package org.firstinspires.ftc.teamcode.OFSB2.Auto.TestCurveTangent;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.OFSB2.Auto.Constants;

@Autonomous(name = "UTurnTest", group = "Autonomous")
public class UTurnTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        START_TO_FINISH,
        DONE
    }

    private PathState pathState;

    private final Pose startingCoordinate = new Pose(102, 11, Math.toRadians(90)); //94, 10
    private final Pose endingCoordinate = new Pose (47, 45, Math.toRadians(270)); //47, 45

    private PathChain start_finish;

    public void buildPaths() {
        // High-precision constraints (100 search limit)
        com.pedropathing.paths.PathConstraints precision = new com.pedropathing.paths.PathConstraints(0.995, 0.1, 0.1, 0.007, 100, 1.2, 100, 1);

        start_finish = follower.pathBuilder(precision)
                .addPath(new BezierCurve(startingCoordinate,
                        new Pose(107, 128), //100,121
                        new Pose (72, 135), // 81, 137
                        new Pose (42 , 131), // 44, 132
                        endingCoordinate))
                .setVelocityConstraint(30)
                .setTangentHeadingInterpolation()
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case START_TO_FINISH:
                follower.followPath(start_finish, true);
                setPathState(PathState.DONE);
                break;
            case DONE:
                if (!follower.isBusy()) {
                    telemetry.addLine("finished it");
                }
                break;
            default:
                telemetry.addLine("error somewehre in the code");
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
        follower.setCentripetalScaling(0.0015);
        follower.getConstants().setBEZIER_CURVE_SEARCH_LIMIT(100);

        buildPaths();
        follower.setPose(startingCoordinate);
        
        pathState = PathState.START_TO_FINISH;
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
