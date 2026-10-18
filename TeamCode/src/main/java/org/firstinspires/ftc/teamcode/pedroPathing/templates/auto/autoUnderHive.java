package org.firstinspires.ftc.teamcode.pedroPathing.templates.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.depoLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.intakeLM0;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "UnderHive", group = "Autonomous")
public class autoUnderHive extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    //private DcMotorEx intake;
    private intakeLM0 intake;
    private depoLM0 depo;

    public enum PathState {
        STARTING_COORDINATE,
        END_PATH_1,
        END_PATH_2,
        WAIT_AT_PATH2,
        END_PATH_3,
        DONE
    }

    private PathState pathState;


    private final Pose startingCoordinate = new Pose(56, 9, Math.toRadians(-180));
    private final Pose endPath1 = new Pose(27, 9, Math.toRadians(-180));
    private final Pose endPath2 = new Pose(11.5, 9, Math.toRadians(-180));
    private final Pose endPath3 = new Pose(52, 114.5 , Math.toRadians(114));
    private final Pose endPath4 = new Pose(11 , 109 , Math.toRadians(0));  //11.109


    private PathChain start_path1, path1_path2, path2_path3, path3_path4;

    public void buildPaths() {
        start_path1 = follower.pathBuilder()
                .addPath(new BezierLine(startingCoordinate, endPath1))
                .setTangentHeadingInterpolation()
                .build();
        path1_path2 = follower.pathBuilder()
                .addPath(new BezierLine(endPath1, endPath2))
                .setTangentHeadingInterpolation()
                .build();
        path2_path3 = follower.pathBuilder()
                .addPath(new BezierCurve(endPath2,
                        /*
                        new Pose(78.5, 16), //82, 14
                        new Pose(59, 107), //58,104

                         */
                        new Pose(61.5,12), //61.5, 12
                        new Pose (48.5 ,27.5), //39,27.5
                        new Pose(63.5,109), //65.5, 109
                        endPath3))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        HeadingInterpolator.PiecewiseNode.linear(0, 0.313, endPath2.getHeading(), Math.toRadians(63)),
                        new HeadingInterpolator.PiecewiseNode(0.313, 0.741, HeadingInterpolator.tangent),
                        HeadingInterpolator.PiecewiseNode.linear(0.741, 1, Math.toRadians(105), endPath3.getHeading())
                ))
                .build();
        path3_path4 = follower.pathBuilder()
                .addPath(new BezierLine(endPath3, endPath4))
                .setTangentHeadingInterpolation()
                .build();
    }

    public boolean waitFor(double seconds) {
        return pathTimer.getElapsedTimeSeconds() > seconds;
    }

    public void statePathUpdate() {
        double timeElapsed = pathTimer.getElapsedTimeSeconds();

        switch (pathState) {
            case STARTING_COORDINATE:
                if (timeElapsed > 1) {
                    follower.followPath(start_path1, true);
                    setPathState(PathState.END_PATH_1);
                }
                break;
            case END_PATH_1:
                if (!follower.isBusy()) {
                    intake.turn_on_intake();
                    follower.followPath(path1_path2, 0.4, true);
                    setPathState(PathState.END_PATH_2);
                }
                break;
            case END_PATH_2:
                if (!follower.isBusy()) {
                    intake.turn_off_intake();
                    follower.followPath(path2_path3, true);
                    setPathState(PathState.WAIT_AT_PATH2);
                }
                break;
            case WAIT_AT_PATH2:
                if (!follower.isBusy()) {
                    setPathState(PathState.END_PATH_3);
                }
                break;
            case END_PATH_3:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(path3_path4, true);
                    setPathState(PathState.DONE);
                }
                break;
            case DONE:
                if (!follower.isBusy()) {
                    telemetry.addLine("finished");
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
        //intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake = new intakeLM0(hardwareMap);
        depo = new depoLM0(hardwareMap);
        buildPaths();
        follower.setPose(startingCoordinate);
        pathState = PathState.STARTING_COORDINATE;
    }
    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        intake.start();
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
