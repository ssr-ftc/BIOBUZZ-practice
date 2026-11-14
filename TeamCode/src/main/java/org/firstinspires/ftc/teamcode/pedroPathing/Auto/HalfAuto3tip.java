package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Robot1;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.AprilTagStuff.APRILTAG_Biobuzz.BioBuzzVision;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Depo;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Turret;

/**
 * One-side auto on the EntireAutoMisha poses. Preload shot, garden, wait at
 * garden, then the garden shot. On that shot the robot looks for
 * {@link #GARDEN_LOOK_SECONDS} seconds. A near hive tag means it shoots. If
 * that tag never shows up, it waits until {@link #PARK_LEAVE_SECONDS} seconds
 * are left and drives to park, skipping the flower. The preload does not use
 * the camera.
 */
@Autonomous(name = "Half Auto 3 Tip", group = "Autonomous")
public class HalfAuto3tip extends OpMode {
    private static final double FULL_INTAKE_BALLS = 4;
    /** Open the hatch this long after the flywheel is at speed. */
    private static final double SHOT_OPEN_SECONDS = 0.3;
    /** How long every shot runs once the flywheel is at speed. Same as Solo TeleOp. */
    private static final double SHOOT_HOLD_SECONDS = 2.5;
    /** Sit at the flower with the flower down and the rollers on before backing out. */
    private static final double FLOWER_INTAKE_SECONDS = 2;
    /** Front gate stays down this long on the way back from the near flower. */
    private static final double INTAKE_HOLD_SECONDS = 1;
    /** Flower and front stay down this long after the garden intake path ends. */
    private static final double GARDEN_FLOWER_UP_SECONDS = 1;
    /** Garden shot watches this long before deciding. */
    private static final double GARDEN_LOOK_SECONDS = 3;
    /** Autonomous period. The no-tag escape leaves when this much time is left. */
    private static final double AUTO_SECONDS = 30;
    private static final double PARK_LEAVE_SECONDS = 5;
    /**
     * A hive tag this close belongs to the near hive. Farther tags can be the
     * other alliance, so they do not count.
     */
    private static final double NEAR_HIVE_INCHES = 76;
    /** Garden creep only. 0.5 is half of the robot's max path speed. */
    private static final double GARDEN_INTAKE_SPEED = 0.5;
    private static final double SHOT_VELOCITY = 1500;
    private static final double SHOT_VELOCITY2 = 1520;

    private static final double PRELOAD = 10;
    private static final double GARDEN = -130;
    private static final double SHOOT1 = 10;

    private enum PathState {
        PRELOAD_SHOOT,
        TO_GARDEN_APPROACH,
        GARDEN_INTAKE,
        GARDEN_CLEAR,
        TO_SHOOT1_GARDEN,
        SHOOT_GARDEN,
        TO_FLOWER_APPROACH,
        FLOWER_INTAKE,
        TO_FLOWER_BACK,
        TO_SHOOT1_FLOWER,
        SHOOT_FLOWER,
        TO_PARK_PREP,
        PARK,
        DONE
    }

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54, 8, 90);
    private final Pose gardenApproach = poseFactory.of(8, 18.5119, 270);
    private final Pose gardenIntake = poseFactory.of(8, 12, 270);
    private final Pose shoot1Start = poseFactory.of(8, 11, 270);
    private final Pose shoot1 = poseFactory.of(54, 8, 90);
    private final Pose shoot1Control1 = poseFactory.of(22.4327, 17.928, 0);
    private final Pose flowerApproach = poseFactory.of(24, 46, 180);
    private final Pose flowerIntake = poseFactory.of(14, 46, 180);
    private final Pose flowerBack = poseFactory.of(24, 46, 180);
    private final Pose parkPrep = poseFactory.of(18, 96, 0);
    private final Pose park = poseFactory.of(14, 96, 0);

    private Follower follower;
    private Intake intake;
    private Depo depo;
    private Turret turret;
    private Timer timers;
    private final BioBuzzVision vision = new BioBuzzVision();

    private PathState pathState = PathState.PRELOAD_SHOOT;
    private boolean cameraReady;
    private String cameraMessage = "";
    private boolean intakeFull;
    /** The garden shot has read the hive and may open the flywheel. */
    private boolean gardenShotReady;
    /** The first garden-shot look has finished. */
    private boolean gardenLookDone;
    /** A near hive tag was seen during the garden shot. */
    private boolean sawGardenTag;

    @Override
    public void init() {
        follower = Robot1.createFollower(hardwareMap);
        follower.setPose(start);
        intake = new Intake(hardwareMap);
        depo = new Depo(hardwareMap);
        turret = new Turret(hardwareMap);
        timers = new Timer();
        timers.createNew("hold");
        timers.createNew("shoot");
        timers.createNew("look");
        depo.start();
        turret.resetTurretEncoders();
        turret.setPositionDeg(10);
        intake.flower_up();
        follower.update();

        try {
            vision.init(hardwareMap, telemetry);
            cameraReady = true;
        } catch (RuntimeException e) {
            cameraReady = false;
            cameraMessage = e.getMessage();
        }
    }

    @Override
    public void init_loop() {
        follower.update();
//        turret.setPositionDeg(PRELOAD);
        turret.run_using_pid();
        updateVision();
        telemetry.addData("Turret deg", turret.getPositionDeg());
        telemetry.addData("Turret target", turret.getTargetDeg());
        telemetry.addData("Camera", cameraReady ? "ON" : cameraMessage);
        telemetry.addData("Hive", hiveLabel());
        telemetry.update();
    }

    @Override
    public void start() {
        intake.start();
        depo.start();
        turret.start();
//        turret.setPositionDeg(PRELOAD);
        intake.resetBalls();
    }

    @Override
    public void loop() {
        follower.update();
        turret.run_using_pid();
        depo.run_using_pid();
        updateVision();
        updateIntakeCount();
        autonomousPathUpdate();

        telemetry.addData("Path", pathState);
        telemetry.addData("Hive", hiveLabel());
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Balls", intake.getBalls());
        telemetry.addData("Depo velo", depo.getVelocity());
        telemetry.addData("Turret deg", turret.getPositionDeg());
        telemetry.addData("Turret target", turret.getTargetDeg());
        telemetry.update();
    }

    @Override
    public void stop() {
        stopCollecting();
        intake.flower_up();
        depo.close_hatch();
        depo.turn_off_deposit();
        turret.turretOff();
        if (cameraReady) {
            vision.stop();
        }
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case PRELOAD_SHOOT:
                if (finishShot(PRELOAD)) {
                    follower.follow(line(start, gardenApproach).linear(start, gardenApproach));
                    pathState = PathState.TO_GARDEN_APPROACH;
                }
                break;
            case TO_GARDEN_APPROACH:
                if (!follower.isBusy()) {
//                    turret.setPositionDeg(GARDEN);
                    intake.front_up();
                    trapGarden();
                    follower.follow(line(gardenApproach, gardenIntake)
                            .constant(gardenIntake)
                            .with(Robot1.foresightConfig.maxPathSpeed.at(GARDEN_INTAKE_SPEED)));
                    pathState = PathState.GARDEN_INTAKE;
                }
                break;
            case GARDEN_INTAKE:
//                turret.setPositionDeg(GARDEN);
                trapGarden();
                if (!follower.isBusy()) {
                    intake.front_down();
                    pathState = PathState.GARDEN_CLEAR;
                }
                break;
            case GARDEN_CLEAR:
                if (timers.checkSecondsLast("hold", GARDEN_FLOWER_UP_SECONDS)) {
                    follower.follow(curve(shoot1Start, shoot1Control1, shoot1).linear(shoot1Start, shoot1));
                    pathState = PathState.TO_SHOOT1_GARDEN;
                } else {
                    holdGardenBalls();
                    if (!timers.isRunning("hold")) {
                        timers.start("hold");
                    }
                }
                break;
            case TO_SHOOT1_GARDEN:
                holdGardenBalls();
//                turret.setPositionDeg(SHOOT1);
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_GARDEN;
                }
                break;
            case SHOOT_GARDEN:
//                turret.setPositionDeg(SHOOT1);
                if (!gardenShotReady) {
                    holdGardenBalls();
                    if (cameraReady && hiveIsUp()) {
                        sawGardenTag = true;
                    }
                    if (!sawGardenTag && getRuntime() >= AUTO_SECONDS - PARK_LEAVE_SECONDS) {
                        intake.flower_up();
                        stopCollecting();
                        follower.follow(line(shoot1, parkPrep).linear(shoot1, parkPrep));
                        pathState = PathState.TO_PARK_PREP;
                        break;
                    }
                    if (!gardenLookDone) {
                        if (timers.checkSecondsLast("look", GARDEN_LOOK_SECONDS)) {
                            gardenLookDone = true;
                        } else if (!timers.isRunning("look")) {
                            timers.start("look");
                        }
                    }
                    if (gardenLookDone && sawGardenTag) {
                        gardenShotReady = true;
                    }
                    break;
                }
                if (finishShot2(SHOOT1)) {
                    intake.flower_up();
                    intake.front_up();
                    follower.follow(line(shoot1, flowerApproach).constant(flowerApproach));
                    pathState = PathState.TO_FLOWER_APPROACH;
                } else {
                    holdGardenBalls();
                }
                break;
            case TO_FLOWER_APPROACH:
                if (!follower.isBusy()) {
                    intake.front_up();
                    trapGarden();
                    follower.follow(line(flowerApproach, flowerIntake).constant(flowerIntake));
                    pathState = PathState.FLOWER_INTAKE;
                }
                break;
            case FLOWER_INTAKE:
                trapGarden();
                if (timers.checkSecondsLast("hold", FLOWER_INTAKE_SECONDS)) {
                    intake.front_down();
                    follower.follow(line(flowerIntake, flowerBack).constant(flowerBack));
                    pathState = PathState.TO_FLOWER_BACK;
                } else if (!follower.isBusy() && !timers.isRunning("hold")) {
                    timers.start("hold");
                }
                break;
            case TO_FLOWER_BACK:
                holdGardenBalls();
                if (!timers.isRunning("hold")) {
                    timers.start("hold");
                } else if (!follower.isBusy() && timers.checkSecondsLast("hold", INTAKE_HOLD_SECONDS)) {
                    follower.follow(line(flowerBack, shoot1).linear(flowerBack, shoot1));
                    pathState = PathState.TO_SHOOT1_FLOWER;
                }
                break;
            case TO_SHOOT1_FLOWER:
                holdGardenBalls();
//                turret.setPositionDeg(SHOOT1);
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_FLOWER;
                }
                break;
            case SHOOT_FLOWER:
                if (finishShot2(SHOOT1)) {
                    intake.flower_up();
                    stopCollecting();
                    follower.follow(line(shoot1, parkPrep).linear(shoot1, parkPrep));
                    pathState = PathState.TO_PARK_PREP;
                } else {
                    holdGardenBalls();
                }
                break;
            case TO_PARK_PREP:
                if (!follower.isBusy()) {
                    follower.follow(line(parkPrep, park).constant(park));
                    pathState = PathState.PARK;
                }
                break;
            case PARK:
                if (!follower.isBusy()) {
                    pathState = PathState.DONE;
                }
                break;
            case DONE:
                break;
            default:
                break;
        }
    }

    /** One shot at {@code turretDeg}. True on the loop it finishes. */
    private boolean finishShot(double turretDeg) {
//        turret.setPositionDeg(turretDeg);
        if (!depo.isDepositOn() && !timers.isRunning("shoot")) {
            depo.set_target_velocity(SHOT_VELOCITY);
        }
        if (depo.reachedvelocity() && !timers.isRunning("shoot")) {
            timers.start("shoot");
        }
        if (timers.checkSeconds("shoot", SHOT_OPEN_SECONDS)) {
            intake.intake_on();
            depo.open_hatch();
        }
        if (timers.checkSecondsLast("shoot", SHOOT_HOLD_SECONDS)) {
            depo.close_hatch();
            depo.set_target_velocity(0);
            intake.intake_off();
            intake.resetBalls();
            intakeFull = false;
            return true;
        }
        return false;
    }

    private boolean finishShot2(double turretDeg) {
//        turret.setPositionDeg(turretDeg);
        if (!depo.isDepositOn() && !timers.isRunning("shoot")) {
            depo.set_target_velocity(SHOT_VELOCITY2);
        }
        if (depo.reachedvelocity() && !timers.isRunning("shoot")) {
            timers.start("shoot");
        }
        if (timers.checkSeconds("shoot", SHOT_OPEN_SECONDS)) {
            intake.intake_on();
            depo.open_hatch();
        }
        if (timers.checkSecondsLast("shoot", SHOOT_HOLD_SECONDS)) {
            depo.close_hatch();
            depo.set_target_velocity(0);
            intake.intake_off();
            intake.resetBalls();
            intakeFull = false;
            return true;
        }
        return false;
    }

    private void updateVision() {
        if (cameraReady) {
            vision.update(getRuntime());
        }
    }

    /** True when the closest hive tag is the near hive. */
    private boolean hiveIsUp() {
        double range = vision.getClosestHiveRange();
        return Double.isFinite(range) && range <= NEAR_HIVE_INCHES;
    }

    private String hiveLabel() {
        if (!cameraReady) {
            return "NO CAMERA";
        }
        double range = vision.getClosestHiveRange();
        if (Double.isFinite(range) && range <= NEAR_HIVE_INCHES) {
            return String.format("%.1f in - SEEN", range);
        }
        if (Double.isFinite(range)) {
            return String.format("%.1f in - ignored", range);
        }
        return "no near tag";
    }

    private void updateIntakeCount() {
        if (shooting() || clearingFlower() || !intake.intaking() || intakeFull) {
            return;
        }
        intake.countBalls();
        if (intake.getBalls() >= FULL_INTAKE_BALLS) {
            intakeFull = true;
            stopCollecting();
        }
    }

    private boolean shooting() {
        return pathState == PathState.PRELOAD_SHOOT
                || pathState == PathState.SHOOT_GARDEN
                || pathState == PathState.SHOOT_FLOWER;
    }

    /** Flower is down and the rollers should keep running. The ball counter must not shut them off. */
    private boolean clearingFlower() {
        return pathState == PathState.GARDEN_CLEAR
                || pathState == PathState.TO_SHOOT1_GARDEN
                || (pathState == PathState.SHOOT_GARDEN && !gardenShotReady)
                || (pathState == PathState.FLOWER_INTAKE && timers.isRunning("hold"))
                || pathState == PathState.TO_FLOWER_BACK
                || pathState == PathState.TO_SHOOT1_FLOWER;
    }

    /** Flower down and rollers on. The front gate stays up until the intake path ends. */
    private void trapGarden() {
        intake.flower_down();
        if (!intakeFull) {
            intake.intake_on();
        }
    }

    /** Flower down, front gate down, rollers on, so balls stay trapped. */
    private void holdGardenBalls() {
        intake.flower_down();
        intake.front_down();
        intake.back_up();
        if (!intakeFull) {
            intake.intake_on();
        }
    }

    private void stopCollecting() {
        intake.intake_off();
        intake.front_up();
        intake.back_up();
    }
}
