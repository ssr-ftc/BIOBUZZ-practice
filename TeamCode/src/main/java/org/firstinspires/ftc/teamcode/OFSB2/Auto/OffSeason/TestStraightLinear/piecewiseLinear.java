package org.firstinspires.ftc.teamcode.OFSB2.Auto.OffSeason.TestStraightLinear;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Timer;
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

    private Path startFinish;

    public void buildPaths() {
        startFinish = com.pedropathing.api.Paths.line(startingCoordinate, path1complete)
                .heading(Interpolator.piecewise().until(1, Interpolator.tangent));
    }

    public void statePathUpdate() {
        switch (pathState) {
            case START:
                follower.pedro.follow(startFinish);

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

        pathState = PathState.START;
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
        telemetry.addData(
                "Heading (Deg)",
                Math.toDegrees(pose.heading())
        );
        telemetry.addData("Path time", pathTimer.seconds());
        telemetry.update();
    }
}
