package org.firstinspires.ftc.teamcode.pedroPathing.Auto;
//skib version 0.0.1
import com.pedropathing.follower.Follower;
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
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.turret;

@Autonomous(name = "UnderHive Akash", group = "Autonomous")
public class autoUnderHive_Akash extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer, stallTimer;
    private org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer timers;
    private intakeLM0 intake;
    private depoLM0 depo;
    private turret turret;

    public enum PathState {
        SHOOT1,
        PATH1,
        PATH2,
        PATH3,
        SHOOT2,
        PATH4,
        PATH4_INTAKE,
        PATH5,
        PATH6,
        SHOOT3,
        INTAKE_FLOWER,
        SHOOT4,
        PATH7,
        DONE
    }

    private PathState pathState;
    private boolean gardenHoldStarted = false;
    private boolean path4Started = false;
    private boolean path4FlowerDown = false;
    private boolean path6Approach = false;
    private boolean flowerPush = false;
    private double turretTargetDeg = 90;
    /** Heading 180 with the turret at 90 points at the first hive. Turret = robot heading - 90. */
    private static final double TURRET_FIELD_AIM_DEG = 90;
    /** Second hive, used for both shots at the far flower. */
    private static final double SECOND_HIVE_TURRET_DEG = 175;
    private static final double TURRET_TOLERANCE_DEG = 3.0;
    /** Flower starts down this long before the path 4 stop, based on current speed. */
    private static final double FLOWER_LEAD_SECONDS = 1.5;
    /** Stopped intake at the path 4 flower and at the far flower. */
    private static final double FLOWER_INTAKE_SECONDS = 2.0;
    /**
     * Last part of the drive to the far flower. 36 inches at this power is a little
     * over 1.5 seconds and slow enough that the flower piece does not slam in.
     */
    private static final double FAR_FLOWER_APPROACH_INCHES = 36;
    private static final double FAR_FLOWER_APPROACH_POWER = 0.3;
    /** Straight garden pass. Intake stays on for one second after the robot stops. */
    private static final double GARDEN_INTAKE_POWER = 0.4;
    private static final double GARDEN_HOLD_SECONDS = 1.0;
    /** Last 40% of the drive from the second shot to the near flower. */
    private static final double PATH4_SLOW_START = 0.6;
    private static final double PATH4_SLOW_POWER = 0.2;
    /**
     * The slow flower approach can stall a few inches short. After a short stall the
     * robot gets this much power to finish the drive onto the flower pose.
     */
    private static final double FLOWER_PUSH_POWER = 0.5;
    /** Slower than this, in inches per second, counts as stalled. */
    private static final double STALL_SPEED = 1.5;
    private static final double STALL_PUSH_SECONDS = 0.25;
    /** Still stalled this long while pushing means the robot is pressed against the flower. */
    private static final double STALL_AT_FLOWER_SECONDS = 0.75;

    // Visualizer poses are degrees. Pedro Pathing 2 stores heading in radians.
    private final Pose start = new Pose(56, 9, Math.toRadians(180));
    private final Pose path1End = new Pose(27, 9, Math.toRadians(180));
    private final Pose path2End = new Pose(12.5, 9, Math.toRadians(180));
    private final Pose path3End = new Pose(56, 9, Math.toRadians(150));
    private final Pose path4End = new Pose(16.1394, 54.5881, Math.toRadians(180));
    private final Pose path5End = new Pose(21.4, 54.2159, Math.toRadians(175.9538));
    private final Pose path6End = new Pose(43.3019, 118.2715, Math.toRadians(90));
    private final Pose path7End = new Pose(7.8428, 113.6069, Math.toRadians(0));

    private PathChain path1, path2, path3, path4, path5, path6, path7;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(start, path1End))
                .setTangentHeadingInterpolation()
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(path1End, path2End))
                .setTangentHeadingInterpolation()
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(path2End, path3End))
                .setLinearHeadingInterpolation(path2End.getHeading(), path3End.getHeading())
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(path3End, path4End))
                .setLinearHeadingInterpolation(path3End.getHeading(), path4End.getHeading())
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(path4End, path5End))
                .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                .build();

        // Straight line to the far flower. Heading goes 150 -> 60 over the first 40%, then 60 -> 90.
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(path5End, path6End))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        HeadingInterpolator.PiecewiseNode.linear(0, 0.4, Math.toRadians(150), Math.toRadians(60)),
                        HeadingInterpolator.PiecewiseNode.linear(0.4, 1, Math.toRadians(60), Math.toRadians(90))
                ))
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(path6End, path7End))
                .setLinearHeadingInterpolation(path6End.getHeading(), path7End.getHeading())
                .build();
    }

    public void useDepo() {
        depo.run_using_pid();
        if (depo.reachedvelocity() && !timers.isRunning("depo")) {
            timers.start("depo");
            depo.open_hatch(turret.getPositionDeg());
            intake.turn_on_intake();
        }
        if (timers.checkSecondsLast("depo", 2)) {
            depo.turn_off_deposit();
            depo.close_hatch();
            intake.turn_off_intake();
        }
    }

    public void statePathUpdate() {
        switch (pathState) {
            case SHOOT1:
                if (pathTimer.getElapsedTimeSeconds() > 1 && !depo.isDepositOn() && turretAimed()) {
                    depo.turn_on_deposit();
                    setPathState(PathState.PATH1);
                }
                break;
            case PATH1:
                if (!depo.isDepositOn()) {
                    follower.followPath(path1, true);
                    setPathState(PathState.PATH2);
                }
                break;
            case PATH2:
                if (!follower.isBusy()) {
                    intake.turn_on_intake();
                    follower.setMaxPower(GARDEN_INTAKE_POWER);
                    follower.followPath(path2, true);
                    setPathState(PathState.PATH3);
                }
                break;
            case PATH3:
                if (follower.isBusy()) {
                    break;
                }
                if (!gardenHoldStarted) {
                    pathTimer.resetTimer();
                    gardenHoldStarted = true;
                    break;
                }
                if (pathTimer.getElapsedTimeSeconds() > GARDEN_HOLD_SECONDS) {
                    intake.turn_off_intake();
                    follower.setMaxPower(1.0);
                    aimTurret(150);
                    follower.followPath(path3, true);
                    setPathState(PathState.SHOOT2);
                }
                break;
            case SHOOT2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1 && turretAimed()) {
                    depo.turn_on_deposit();
                    setPathState(PathState.PATH4);
                }
                break;
            case PATH4:
                if (!path4Started) {
                    if (!follower.isBusy() && !depo.isDepositOn()) {
                        follower.setMaxPower(1.0);
                        intake.turn_off_intake();
                        intake.flower_up();
                        follower.followPath(path4, true);
                        path4Started = true;
                    }
                    break;
                }
                boolean path4Slow = follower.getCurrentTValue() >= PATH4_SLOW_START;
                if (path4Slow && !flowerPush) {
                    follower.setMaxPower(PATH4_SLOW_POWER);
                }
                if (!path4FlowerDown && leadTimeReached(path4End, FLOWER_LEAD_SECONDS)) {
                    intake.flower_down();
                    path4FlowerDown = true;
                }
                if (reachedFlower(path4Slow)) {
                    intake.flower_down();
                    intake.turn_on_intake();
                    setPathState(PathState.PATH4_INTAKE);
                }
                break;
            case PATH4_INTAKE:
                if (pathTimer.getElapsedTimeSeconds() > FLOWER_INTAKE_SECONDS) {
                    intake.turn_off_intake();
                    intake.flower_up();
                    follower.setMaxPower(1.0);
                    follower.followPath(path5, true);
                    setPathState(PathState.PATH5);
                }
                break;
            case PATH5:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    flowerPush = false;
                    setTurretDeg(SECOND_HIVE_TURRET_DEG);
                    follower.followPath(path6, true);
                    setPathState(PathState.PATH6);
                }
                break;
            case PATH6:
                if (!path6Approach && distanceTo(path6End) <= FAR_FLOWER_APPROACH_INCHES) {
                    path6Approach = true;
                    intake.flower_down();
                    follower.setMaxPower(FAR_FLOWER_APPROACH_POWER);
                }
                if (!reachedFlower(path6Approach)) {
                    break;
                }
                follower.setMaxPower(1.0);
                if (!turretAimed()) {
                    break;
                }
                intake.turn_off_intake();
                intake.flower_up();
                depo.turn_on_deposit();
                setPathState(PathState.SHOOT3);
                break;
            case SHOOT3:
                if (depo.isDepositOn()) {
                    break;
                }
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    intake.flower_down();
                    intake.turn_on_intake();
                    setPathState(PathState.INTAKE_FLOWER);
                }
                break;
            case INTAKE_FLOWER:
                if (pathTimer.getElapsedTimeSeconds() > FLOWER_INTAKE_SECONDS) {
                    intake.flower_up();
                    depo.turn_on_deposit();
                    setPathState(PathState.SHOOT4);
                }
                break;
            case SHOOT4:
                if (!depo.isDepositOn() && pathTimer.getElapsedTimeSeconds() > 1) {
                    intake.flower_up();
                    intake.turn_off_intake();
                    follower.setMaxPower(1.0);
                    follower.followPath(path7, true);
                    setPathState(PathState.PATH7);
                }
                break;
            case PATH7:
                if (pathDone(path7End)) {
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

    /** Keeps the shooter on the first hive. Heading 180 uses turret 90. */
    private void aimTurret(double robotHeadingDeg) {
        setTurretDeg(robotHeadingDeg - TURRET_FIELD_AIM_DEG);
    }

    private void setTurretDeg(double degrees) {
        turretTargetDeg = degrees;
        turret.setPositionDeg(degrees);
    }

    private boolean turretAimed() {
        return Math.abs(turret.getPositionDeg() - turretTargetDeg) <= TURRET_TOLERANCE_DEG;
    }

    /**
     * Pedro only ends a path at t >= 0.99 and ignores a stall past t = 0.8, so a slow
     * approach that stalls a few inches short of the flower never finishes. During the
     * slow approach, a stall gets more power so Pedro can finish on the flower pose.
     * If the robot still cannot move while pushing, it is pressed against the flower.
     */
    private boolean reachedFlower(boolean slowApproach) {
        if (!follower.isBusy()) {
            return true;
        }
        if (!slowApproach || follower.getVelocity().getMagnitude() > STALL_SPEED) {
            stallTimer.resetTimer();
            return false;
        }
        if (!flowerPush) {
            if (stallTimer.getElapsedTimeSeconds() > STALL_PUSH_SECONDS) {
                flowerPush = true;
                follower.setMaxPower(FLOWER_PUSH_POWER);
                stallTimer.resetTimer();
            }
            return false;
        }
        if (stallTimer.getElapsedTimeSeconds() > STALL_AT_FLOWER_SECONDS) {
            follower.breakFollowing();
            return true;
        }
        return false;
    }

    private boolean pathDone(Pose target) {
        if (!follower.isBusy()) {
            return true;
        }
        double speed = follower.getVelocity().getMagnitude();
        if (follower.getCurrentTValue() >= 0.8 && speed < 2.0 && distanceTo(target) <= 8) {
            follower.breakFollowing();
            return true;
        }
        return false;
    }

    private double distanceTo(Pose target) {
        Pose pose = follower.getPose();
        double dx = target.getX() - pose.getX();
        double dy = target.getY() - pose.getY();
        return Math.hypot(dx, dy);
    }

    /** True once the robot is about {@code seconds} away from the pose at its current speed. */
    private boolean leadTimeReached(Pose target, double seconds) {
        double speed = follower.getVelocity().getMagnitude();
        if (speed < 8) {
            return distanceTo(target) < 12;
        }
        return distanceTo(target) <= speed * seconds;
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        stallTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setCentripetalScaling(0.0015);
        follower.getConstants().setBEZIER_CURVE_SEARCH_LIMIT(100);
        intake = new intakeLM0(hardwareMap);
        depo = new depoLM0(hardwareMap);
        turret = new turret(hardwareMap);
        timers = new org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer();

        buildPaths();
        follower.setPose(start);
        pathState = PathState.SHOOT1;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        intake.start();
        depo.start();
        turret.start();
        turret.ResetTurretEncoders();
        aimTurret(180);
        timers.createNew("depo");
    }

    @Override
    public void loop() {
        follower.update();
        turret.run_using_pid();
        useDepo();
        statePathUpdate();
        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Turret target", turretTargetDeg);
        telemetry.addData("Turret deg", turret.getPositionDeg());
        telemetry.update();
    }
}
