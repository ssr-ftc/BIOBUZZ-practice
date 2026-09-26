package org.firstinspires.ftc.teamcode.OFSB2.Auto.OffSeason.TestCurveTangent;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.api.Paths;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.OFSB2.Subsystems.CustomFollower;

@Autonomous(name = "UTurnTest", group = "Autonomous")
public class UTurnTest extends OpMode {
    private CustomFollower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        START_TO_FINISH,
        DONE
    }

    private PathState pathState;

    private final Pose startingCoordinate = new Pose(102, 11, Math.toRadians(90)); //94, 10
    private final Pose endingCoordinate = new Pose (47, 45, Math.toRadians(270)); //47, 45

    private Path start_finish;

    public void buildPaths() {
        start_finish = com.pedropathing.api.Paths.curve(startingCoordinate,
                        new Pose(107, 128, 0), //100,121
                        new Pose (72, 135, 0), // 81, 137
                        new Pose (42 , 131, 0), // 44, 132
                        endingCoordinate)
                .tangent();
    }
    public void statePathUpdate() {
        switch (pathState) {
            case START_TO_FINISH:
                follower.autoAcceleration(0.7, 80.0);
                follower.pedro.follow(start_finish);
                setPathState(PathState.DONE);
                break;
            case DONE:
                if (!follower.pedro.isBusy()) {
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

        pathState = PathState.START_TO_FINISH;
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
        telemetry.addData("Path time", pathTimer.seconds());
        telemetry.update();
    }
}
