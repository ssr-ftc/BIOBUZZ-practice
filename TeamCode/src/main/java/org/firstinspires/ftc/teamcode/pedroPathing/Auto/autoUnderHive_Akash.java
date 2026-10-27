package org.firstinspires.ftc.teamcode.pedroPathing.Auto;
//Version 1.3.3
import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.DriveLimits;
import org.firstinspires.ftc.teamcode.pedroPathing.PathTimer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.depoLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.intakeLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.turret;

@Autonomous(name = "LM0 Auto", group = "Autonomous")
public class autoUnderHive_Akash extends OpMode {
    private Follower follower;
    private PathTimer pathTimer, opModeTimer, stallTimer;
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
        BACKUP_FLOWER1,
        DRIVE_FAR_SIDE,
        DRIVE_FAR_SHOOT,
        SHOOT_FAR,
        DROP_FOR_FLOWER2,
        DRIVE_FLOWER2,
        PARK,
        DONE
    }

    private PathState pathState;
    private boolean gardenHoldStarted = false;
    private boolean flower1DriveStarted = false;
    private boolean flower1Down = false;
    private boolean flower2Slowing = false;
    private boolean flower2HoldStarted = false;
    private boolean flower2DropStarted = false;
    private boolean flowerPush = false;
    private double turretTargetDeg = 90;
    /** Heading 180 with the turret at 90 points at the first hive. Turret = robot heading - 90. */
    private static final double TURRET_FIELD_AIM_DEG = 90;
    /** Far shot at the second hive. */
    private static final double SECOND_HIVE_TURRET_DEG = -40;
    /** A shot counts as aimed once the turret is within this many degrees of the target. */
    private static final double TURRET_TOLERANCE_DEG = 7.0;
    /** Flower starts down this long before the near flower, based on current speed. */
    private static final double FLOWER_LEAD_SECONDS = 1.5;
    /** Stopped intake at each flower. The intake is already running on the way in. */
    private static final double FLOWER_HOLD_SECONDS = 0.5;
    /** Time for the flower piece to finish dropping at the turnaround before driving in. */
    private static final double FLOWER2_DROP_SECONDS = 0.6;
    /**
     * Last part of the drive into the far flower. 36 inches at this power is a little
     * over 1.5 seconds and slow enough that the flower piece does not slam in.
     */
    private static final double FAR_FLOWER_APPROACH_INCHES = 36;
    private static final double FAR_FLOWER_APPROACH_POWER = 0.4;
    /** Straight garden intake. The curved visualizer lines are not used. Intake stays on for one second after the robot stops. */
    private static final double GARDEN_INTAKE_POWER = 0.4;
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

    // Visualizer poses are degrees. Pedro Pathing 2 stores heading in radians.
    private final Pose start = Constants.pose(56, 9, Math.toRadians(180));
    private final Pose gardenIntake = Constants.pose(27, 9, Math.toRadians(180));
    private final Pose gardenIntakeEnd = Constants.pose(12.5, 9, Math.toRadians(180));
    private final Pose nearShoot = Constants.pose(56, 9, Math.toRadians(150));
    private final Pose flower1 = Constants.pose(13, 54, Math.toRadians(180));
    private final Pose flower1Backup = Constants.pose(21.4, 53, Math.toRadians(180));
    private final Pose farSide = Constants.pose(22.0147, 88.2872, Math.toRadians(180));
    private final Pose farShoot = Constants.pose(31.9549, 134.2904, Math.toRadians(-102.1929));
    private final Pose flower2Turnaround = Constants.pose(46.2013, 108.9895, Math.toRadians(88.0699));
    private final Pose flower2 = Constants.pose(47.0463, 133.6, Math.toRadians(90));
    private final Pose parkSpot = Constants.pose(-1.2065, 106.9036, Math.toRadians(0));

    private Path toGardenIntake, toGardenIntakeEnd, toNearShoot, toFlower1, backupFlower1, toFarSide, toFarShoot, toFlower2Turnaround, toFlower2, toPark;

    public void buildPaths() {
        toGardenIntake = line(start, gardenIntake).tangent();
        toGardenIntakeEnd = line(gardenIntake, gardenIntakeEnd).tangent();
        toNearShoot = line(gardenIntakeEnd, nearShoot).linear(gardenIntakeEnd, nearShoot);
        toFlower1 = line(nearShoot, flower1).linear(nearShoot, flower1);
        backupFlower1 = line(flower1, flower1Backup).tangent();
        toFarSide = line(flower1Backup, farSide).constant(farSide);
        toFarShoot = line(farSide, farShoot).tangent();
        toFlower2Turnaround = line(farShoot, flower2Turnaround).tangent();
        toFlower2 = line(flower2Turnaround, flower2).constant(Math.toRadians(90));
        toPark = line(flower2, parkSpot).linear(flower2, parkSpot);
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

    /** Far shots raise the flower and stop the intake before the flywheel starts. */
    private void shootWithFlowerUp() {
        intake.turn_off_intake();
        intake.flower_up();
        shoot();
    }

    /** True once the shot has finished and the robot has waited one second. */
    private boolean shotDone() {
        return !depo.isDepositOn() && pathTimer.getElapsedTimeSeconds() > 1;
    }

    public void statePathUpdate() {
        switch (pathState) {
            case SHOOT_PRELOAD:
                if (pathTimer.getElapsedTimeSeconds() > 1 && !depo.isDepositOn() && turretAimed()) {
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
                    //DriveLimits.setMaxPower(follower, GARDEN_INTAKE_POWER);
                    follower.follow(toGardenIntakeEnd);
                    setPathState(PathState.HOLD_GARDEN);
                }
                break;
            case HOLD_GARDEN:
                if (follower.isBusy()) {
                    break;
                }
                if (!gardenHoldStarted) {
                    pathTimer.resetTimer();
                    gardenHoldStarted = true;
                    break;
                }
                if (pathTimer.getElapsedTimeSeconds() > GARDEN_HOLD_SECONDS) {
                    //intake.turn_off_intake();
                    DriveLimits.setMaxPower(follower, 1.0);
                    aimTurret(150);
                    follower.follow(toNearShoot);
                    setPathState(PathState.SHOOT_NEAR);
                }
                break;
            case SHOOT_NEAR:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1 && turretAimed()) {
                    shoot();
                    setPathState(PathState.DRIVE_FLOWER1);
                }
                break;
            case DRIVE_FLOWER1:
                if (!flower1DriveStarted) {
                    if (!follower.isBusy() && !depo.isDepositOn()) {
                        DriveLimits.setMaxPower(follower, 1.0);
                        intake.turn_off_intake();
                        intake.flower_up();
                        follower.follow(toFlower1);
                        flower1DriveStarted = true;
                    }
                    break;
                }
                boolean flower1Slow = follower.completion() >= FLOWER1_SLOW_START;
                if (flower1Slow && !flowerPush) {
                    DriveLimits.setMaxPower(follower, FLOWER1_SLOW_POWER);
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
                if (pathTimer.getElapsedTimeSeconds() > FLOWER_HOLD_SECONDS) {
                    intake.turn_off_intake();
                    intake.flower_up();
                    DriveLimits.setMaxPower(follower, 1.0);
                    follower.follow(backupFlower1);
                    setPathState(PathState.BACKUP_FLOWER1);
                }
                break;
            case BACKUP_FLOWER1:
                if (!follower.isBusy()) {
                    DriveLimits.setMaxPower(follower, 1.0);
                    flowerPush = false;
                    setTurretDeg(SECOND_HIVE_TURRET_DEG);
                    follower.follow(toFarSide);
                    setPathState(PathState.DRIVE_FAR_SIDE);
                }
                break;
            case DRIVE_FAR_SIDE:
                if (!follower.isBusy()) {
                    DriveLimits.setMaxPower(follower, 1.0);
                    follower.follow(toFarShoot);
                    setPathState(PathState.DRIVE_FAR_SHOOT);
                }
                break;
            case DRIVE_FAR_SHOOT:
                if (!follower.isBusy() && turretAimed()) {
                    shootWithFlowerUp();
                    setPathState(PathState.SHOOT_FAR);
                }
                break;
            case SHOOT_FAR:
                if (depo.isDepositOn()) {
                    break;
                }
                if (shotDone()) {
                    DriveLimits.setMaxPower(follower, 1.0);
                    flowerPush = false;
                    intake.flower_up();
                    follower.follow(toFlower2Turnaround);
                    setPathState(PathState.DROP_FOR_FLOWER2);
                }
                break;
            case DROP_FOR_FLOWER2:
                if (follower.isBusy()) {
                    break;
                }
                if (!flower2DropStarted) {
                    DriveLimits.setMaxPower(follower, 0.6);
                    flowerPush = false;
                    intake.flower_down();
                    intake.turn_on_intake();
                    flower2DropStarted = true;
                    pathTimer.resetTimer();
                    break;
                }
                if (pathTimer.getElapsedTimeSeconds() < FLOWER2_DROP_SECONDS) {
                    intake.flower_down();
                    intake.turn_on_intake();
                    break;
                }
                DriveLimits.setMaxPower(follower, FAR_FLOWER_APPROACH_POWER);
                follower.follow(toFlower2);
                setPathState(PathState.DRIVE_FLOWER2);
                break;
            case DRIVE_FLOWER2:
                if (!flower2Slowing && distanceTo(flower2) <= FAR_FLOWER_APPROACH_INCHES) {
                    flower2Slowing = true;
                    intake.flower_down();
                    DriveLimits.setMaxPower(follower, FAR_FLOWER_APPROACH_POWER);
                }
                if (!reachedFlower(flower2Slowing)) {
                    break;
                }
                if (!flower2HoldStarted) {
                    flower2HoldStarted = true;
                    pathTimer.resetTimer();
                    DriveLimits.setMaxPower(follower, 1.0);
                    intake.flower_down();
                    intake.turn_on_intake();
                    break;
                }
                if (pathTimer.getElapsedTimeSeconds() < FLOWER_HOLD_SECONDS) {
                    intake.flower_down();
                    intake.turn_on_intake();
                    break;
                }
                intake.flower_up();
                intake.turn_off_intake();
                setTurretDeg(0);
                DriveLimits.setMaxPower(follower, 1.0);
                follower.follow(toPark);
                setPathState(PathState.PARK);
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
        if (!slowApproach || Math.hypot(follower.velocity().vx, follower.velocity().vy) > STALL_SPEED) {
            stallTimer.resetTimer();
            return false;
        }
        if (!flowerPush) {
            if (stallTimer.getElapsedTimeSeconds() > STALL_PUSH_SECONDS) {
                flowerPush = true;
                DriveLimits.setMaxPower(follower, FLOWER_PUSH_POWER);
                stallTimer.resetTimer();
            }
            return false;
        }
        if (stallTimer.getElapsedTimeSeconds() > STALL_AT_FLOWER_SECONDS) {
            follower.stop();
            return true;
        }
        return false;
    }

    private boolean pathDone(Pose target) {
        if (!follower.isBusy()) {
            return true;
        }
        double speed = Math.hypot(follower.velocity().vx, follower.velocity().vy);
        if (follower.completion() >= 0.8 && speed < 2.0 && distanceTo(target) <= 8) {
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
        double speed = Math.hypot(follower.velocity().vx, follower.velocity().vy);
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
        pathTimer = new PathTimer();
        opModeTimer = new PathTimer();
        stallTimer = new PathTimer();
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
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Turret target", turretTargetDeg);
        telemetry.addData("Turret deg", turret.getPositionDeg());
        telemetry.update();
    }
}