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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.depoLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.intakeLM0;

@Autonomous(name = "UnderHive", group = "Autonomous")
public class autoUnderHive extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer timers;
    private intakeLM0 intake;
    private depoLM0 depo;

    public enum PathState {
        SHOOT1,
        STARTING_COORDINATE,
        END_PATH_1,
        END_PATH_2,
        END_PATH_NEW,
        SHOOT2,
        NEARFLOWER,
        WAIT_AT_PATH2,
        END_PATH_3,
        DONE
    }

    private PathState pathState;

    private final Pose startingCoordinate = new Pose(56, 9, Math.toRadians(-180));
    private final Pose endPath1 = new Pose(27, 9, Math.toRadians(-180));
    private final Pose endPath2 = new Pose(13, 9, Math.toRadians(-180));
    private final Pose endPathnew = new Pose (56, 9, Math.toRadians(90));
    private final Pose flowerNear = new Pose (19,37.6, (145));
    private final Pose oppositFlower = new Pose(52, 114.5, Math.toRadians(114));
    private final Pose endPath4 = new Pose(11, 109, Math.toRadians(0));

    private PathChain start_path1, path1_path2, path2_shoot, shoot2_oppFlower, flower_park;

    public void buildPaths() {
        start_path1 = follower.pathBuilder()
                .addPath(new BezierLine(startingCoordinate, endPath1))
                .setTangentHeadingInterpolation()
                .build();
        path1_path2 = follower.pathBuilder()
                .addPath(new BezierLine(endPath1, endPath2))
                .setTangentHeadingInterpolation()
                .build();
        path2_shoot = follower.pathBuilder()
                .addPath(new BezierLine(endPath2,

                        //new Pose(61.5, 12),
                        //new Pose(48.5, 27.5),
                        //new Pose(63.5, 109),
                        endPathnew))
                /*
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        HeadingInterpolator.PiecewiseNode.linear(0, 0.313, endPath2.getHeading(), Math.toRadians(63)),
                        new HeadingInterpolator.PiecewiseNode(0.313, 0.741, HeadingInterpolator.tangent),
                        HeadingInterpolator.PiecewiseNode.linear(0.741, 1, Math.toRadians(105), endPath3.getHeading())
                ))
                 */
                .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                .build();
        shoot2_oppFlower = follower.pathBuilder()
                .addPath(new BezierCurve(endPathnew,
                        new Pose(60,69),
                        oppositFlower))
                .setTangentHeadingInterpolation()
                .build();
        flower_park = follower.pathBuilder()
                .addPath(new BezierLine(oppositFlower, endPath4))
                .setTangentHeadingInterpolation()
                .build();
    }

    public boolean waitFor(double seconds) {
        return pathTimer.getElapsedTimeSeconds() > seconds;
    }


    public void useDepo() {
        depo.run_using_pid();
        if (depo.reachedvelocity() && !timers.isRunning("depo")) {
            timers.start("depo");
            depo.open_hatch_2();
            intake.turn_on_intake();
        }
        if (timers.checkSecondsLast("depo", 2)) {
            depo.turn_off_deposit();
            depo.close_hatch();
            intake.turn_off_intake();
        }
    }


    public void statePathUpdate() {
        double timeElapsed = pathTimer.getElapsedTimeSeconds();
        switch (pathState) {
            case SHOOT1:
                if (timeElapsed > 1 && !depo.isDepositOn()) {
                    depo.turn_on_deposit(); 
                    setPathState(PathState.STARTING_COORDINATE);
                }
                break;
            case STARTING_COORDINATE:
                if (!depo.isDepositOn()) {
                    //depo.turn_off_deposit();
                    follower.followPath(start_path1, true);
                    setPathState(PathState.END_PATH_1);
                }
                break;
            case END_PATH_1:
                if (!follower.isBusy()) {
                    intake.turn_on_intake();
                    follower.followPath(path1_path2, 0.4,  true);
                    setPathState(PathState.END_PATH_2);
                }
                break;
            case END_PATH_2:
                if (!follower.isBusy()) {
                    //intake.turn_off_intake();
                    //follower.followPath(path2_shoot, true);
                    setPathState(PathState.END_PATH_NEW);
                }
                break;
                case END_PATH_NEW:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(path2_shoot, true);
                    setPathState(PathState.SHOOT2);
                }
                break;
            case SHOOT2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    depo.turn_on_deposit();
                    setPathState(PathState.WAIT_AT_PATH2);
                }
                break;
            case WAIT_AT_PATH2:
                if (!follower.isBusy() && !depo.isDepositOn()) {
                    follower.followPath(shoot2_oppFlower);
                    setPathState(PathState.END_PATH_3);
                }
                break;
            case END_PATH_3:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(flower_park, true);
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
        timers = new org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer();

        buildPaths();
        follower.setPose(startingCoordinate);
        pathState = PathState.SHOOT1;
    }
    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        intake.start();
        depo.start();
        timers.createNew("depo");
    }
    @Override
    public void loop() {
        follower.update();
        useDepo();
        statePathUpdate();
        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}
