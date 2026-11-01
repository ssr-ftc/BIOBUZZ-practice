package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;

import static com.pedropathing.api.Paths.line;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0.depoLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0.intakeLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0.turret;

@Autonomous(name = "Half Auto", group = "Autonomous")
public class HalfAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer, stallTimer;
    private org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer timers;
    private intakeLM0 intake;
    private depoLM0 depo;
    private turret turret;

    public enum PathState {
        SHOOT_PRELOAD,
        DRIVE_GARDEN,
        DRIVE_GARDEN_SLOW,
        HOLD_GARDEN,
        SHOOT_NEAR,
        DRIVE_FLOWER1,
        HOLD_FLOWER1,
        PARK,
        DONE
    }

    private PathState pathState;
    private boolean gardenHoldStarted = false;
    private boolean flower1DriveStarted = false;
    private boolean flower1Down = false;
    private boolean flowerPush = false;
    private double turretTargetDeg = 90;
    /** Heading 180 with the turret at 90 points at the first hive. Turret = robot heading - 90. */
    private static final double TURRET_FIELD_AIM_DEG = 90;
    /** A shot counts as aimed once the turret is within this many degrees of the target. */
    private static final double TURRET_TOLERANCE_DEG = 7.0;
    /** Flower starts down this long before the near flower, based on current speed. */
    private static final double FLOWER_LEAD_SECONDS = 1.5;
    /** Stopped intake at the flower. The intake is already running on the way in. */
    private static final double FLOWER_HOLD_SECONDS = 0.5;
    /** Straight garden intake. The curved visualizer lines are not used. Intake stays on for one second after the robot stops. */
    private static final double GARDEN_HOLD_SECONDS = 0.5;
    /** Last 40% of the drive from the near shot to the first flower. */
    private static final double FLOWER1_SLOW_START = 0.6;
    private static final double FLOWER1_SLOW_POWER = 0.2;
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

    // Same poses as LM0 Auto. After flower 1 the robot parks and skips the far shot and flower 2.
    private final Pose start = new Pose(56, 9, Math.toRadians(180));
    private final Pose gardenIntake = new Pose(27, 9, Math.toRadians(180));
    private final Pose gardenIntakeEnd = new Pose(12.5, 9, Math.toRadians(180));
    private final Pose nearShoot = new Pose(56, 9, Math.toRadians(150));
    private final Pose flower1 = new Pose(13, 52, Math.toRadians(180));
    private final Pose parkSpot = new Pose(-4.2065, 98.9036, Math.toRadians(0));

    private Path toGardenIntake, toGardenIntakeEnd, toNearShoot, toFlower1, toPark;

    public void buildPaths() {
        toGardenIntake = line(start, gardenIntake).tangent();
        toGardenIntakeEnd = line(gardenIntake, gardenIntakeEnd).tangent();
        toNearShoot = line(gardenIntakeEnd, nearShoot).linear(gardenIntakeEnd.heading(), nearShoot.heading());
        toFlower1 = line(nearShoot, flower1).linear(nearShoot.heading(), flower1.heading());
        toPark = line(flower1, parkSpot).linear(flower1.heading(), parkSpot.heading());
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

    /** Starts the flywheel. The hatch still opens from useDepo once the shooter is up to speed. */
    private void shoot() {
        depo.turn_on_deposit();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case SHOOT_PRELOAD:
                if (pathTimer.seconds() > 1 && !depo.isDepositOn() && turretAimed()) {
                    shoot();
                    setPathState(PathState.DRIVE_GARDEN);
                }
                break;
            case DRIVE_GARDEN:
                if (!depo.isDepositOn()) {
                    intake.turn_on_intake();
                    follower.follow(toGardenIntake);
                    setPathState(PathState.DRIVE_GARDEN_SLOW);
                }
                break;
            case DRIVE_GARDEN_SLOW:
                if (!follower.isBusy()) {
                    intake.turn_on_intake();
                    Constants.setPathPower(0.4);
                    follower.follow(toGardenIntakeEnd);
                    setPathState(PathState.HOLD_GARDEN);
                }
                break;
            case HOLD_GARDEN:
                if (follower.isBusy()) {
                    break;
                }
                if (!gardenHoldStarted) {
                    pathTimer.reset();
                    gardenHoldStarted = true;
                    break;
                }
                if (pathTimer.seconds() > GARDEN_HOLD_SECONDS) {
                    Constants.setPathPower(1.0);
                    aimTurret(150);
                    follower.follow(toNearShoot);
                    setPathState(PathState.SHOOT_NEAR);
                }
                break;
            case SHOOT_NEAR:
                if (!follower.isBusy() && pathTimer.seconds() > 1 && turretAimed()) {
                    shoot();
                    setPathState(PathState.DRIVE_FLOWER1);
                }
                break;
            case DRIVE_FLOWER1:
                if (!flower1DriveStarted) {
                    if (!follower.isBusy() && !depo.isDepositOn()) {
                        Constants.setPathPower(1.0);
                        intake.turn_off_intake();
                        intake.flower_up();
                        follower.follow(toFlower1);
                        flower1DriveStarted = true;
                    }
                    break;
                }
                boolean flower1Slow = follower.parametricCompletion() >= FLOWER1_SLOW_START;
                if (flower1Slow && !flowerPush) {
                    Constants.setPathPower(FLOWER1_SLOW_POWER);
                }
                if (!flower1Down && (flower1Slow || leadTimeReached(flower1, FLOWER_LEAD_SECONDS))) {
                    intake.flower_down();
                    intake.turn_on_intake();
                    flower1Down = true;
                }
                if (reachedFlower(flower1Slow)) {
                    intake.flower_down();
                    intake.turn_on_intake();
                    setPathState(PathState.HOLD_FLOWER1);
                }
                break;
            case HOLD_FLOWER1:
                if (pathTimer.seconds() > FLOWER_HOLD_SECONDS) {
                    intake.turn_off_intake();
                    intake.flower_up();
                    setTurretDeg(0);
                    Constants.setPathPower(1.0);
                    follower.follow(toPark);
                    setPathState(PathState.PARK);
                }
                break;
            case PARK:
                setTurretDeg(0);
                if (pathDone(parkSpot)) {
                    setPathState(PathState.DONE);
                }
                break;
            case DONE:
                setTurretDeg(0);
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
        if (!slowApproach || follower.velocity().linSpeed() > STALL_SPEED) {
            stallTimer.reset();
            return false;
        }
        if (!flowerPush) {
            if (stallTimer.seconds() > STALL_PUSH_SECONDS) {
                flowerPush = true;
                Constants.setPathPower(FLOWER_PUSH_POWER);
                stallTimer.reset();
            }
            return false;
        }
        if (stallTimer.seconds() > STALL_AT_FLOWER_SECONDS) {
            follower.stop();
            return true;
        }
        return false;
    }

    private boolean pathDone(Pose target) {
        if (!follower.isBusy()) {
            return true;
        }
        double speed = follower.velocity().linSpeed();
        if (follower.parametricCompletion() >= 0.8 && speed < 2.0 && distanceTo(target) <= 8) {
            follower.stop();
            return true;
        }
        return false;
    }

    private double distanceTo(Pose target) {
        Pose pose = follower.pose();
        double dx = target.x() - pose.x();
        double dy = target.y() - pose.y();
        return Math.hypot(dx, dy);
    }

    /** True once the robot is about {@code seconds} away from the pose at its current speed. */
    private boolean leadTimeReached(Pose target, double seconds) {
        double speed = follower.velocity().linSpeed();
        if (speed < 8) {
            return distanceTo(target) < 12;
        }
        return distanceTo(target) <= speed * seconds;
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.reset();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        stallTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        intake = new intakeLM0(hardwareMap);
        intake.flower_up();
        depo = new depoLM0(hardwareMap);
        turret = new turret(hardwareMap);
        timers = new org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer();

        buildPaths();
        follower.setPose(start);
        pathState = PathState.SHOOT_PRELOAD;
    }

    @Override
    public void start() {
        turret.ResetTurretEncoders();
        opModeTimer.reset();
        setPathState(pathState);
        intake.start();
        depo.start();
        turret.start();
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
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Turret target", turretTargetDeg);
        telemetry.addData("Turret deg", turret.getPositionDeg());
        telemetry.update();
    }
}
