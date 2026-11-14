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
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.savePosition;

/**
 * Full auto. The poses are fixed waypoints. After the garden intake, a hive
 * tag within {@link #NEAR_HIVE_INCHES} means the hive has not tipped, so the
 * robot takes shoot1. No tag that close means it has tipped, and farther tags
 * are ignored. Alliance selection is not used.
 */
@Autonomous(name = "EntireAuto Akash (vision)", group = "Season")
public class EntireAuto extends OpMode {
    private static final double FULL_INTAKE_BALLS = 4;
    /** Open the hatch this long after the flywheel is at speed. */
    private static final double SHOT_OPEN_SECONDS = 0.3;
    /** How long every shot runs once the flywheel is at speed. */
    private static final double SHOOT_HOLD_SECONDS = 2;
    /** How long the front gate stays down on the way back from the near flower. */
    private static final double INTAKE_HOLD_SECONDS = 1;
    /** How long to watch the hive before choosing the next path. */
    private static final double HIVE_LOOK_SECONDS = 0.5;
    /**
     * A hive tag this close belongs to the near hive. Tags farther away can be
     * the other alliance, so they do not count. 76 includes a tag at 75 or 76.
     */
    private static final double NEAR_HIVE_INCHES = 76;
    /** Garden creep only. 0.5 is half of the robot's max path speed. */
    private static final double GARDEN_INTAKE_SPEED = 0.5;
    /** Front gate and flower stay down this long after the garden intake path ends. */
    private static final double GARDEN_FLOWER_UP_SECONDS = 2;

    /** Field headings. Converted to robot-relative when commanded. */
    private static final double PRELOAD_FIELD_DEG = 70;
    private static final double GARDEN_FIELD_DEG = 41;
    private static final double SHOOT1_FIELD_DEG = 70;
    private static final double OPP_FIELD_DEG = 296;

    private static final double PRELOAD = 4;
    private static final double GARDEN = -130;
    private static final double SHOOT1 = 0;
    private static final double FLOWER = -195;

    private enum PathState {
        PRELOAD_SHOOT,
        TO_GARDEN_APPROACH,
        GARDEN_INTAKE,
        GARDEN_CLEAR,
        READ_HIVE,
        TO_SHOOT1,
        SHOOT1,
        TO_FLOWER_APPROACH,
        FLOWER_INTAKE,
        TO_FLOWER_BACK,
        TO_OPP_FLOWER,
        SHOOT_OPP,
        TO_OPP_INTAKE,
        TO_OPP_BACK,
        SHOOT_OPP_RETURN,
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
    private final Pose flowerApproach = poseFactory.of(24, 48, 180);
    private final Pose flowerIntake = poseFactory.of(14, 48, 180);
    private final Pose flowerBack = poseFactory.of(24, 48, 180);
    private final Pose oppFlowerApproach = poseFactory.of(47, 124, 90);
    private final Pose oppFlowerIntake = poseFactory.of(47, 129, 90);
    private final Pose parkPrep = poseFactory.of(18, 96, 0);
    private final Pose park = poseFactory.of(10, 96, 0);

    private Follower follower;
    private Intake intake;
    private Depo depo;
    private Turret turret;
    private Timer timers;
    private final BioBuzzVision vision = new BioBuzzVision();

    private PathState pathState = PathState.PRELOAD_SHOOT;
    private boolean intakeFull;
    private boolean cameraReady;
    private String cameraMessage = "";
    private boolean hiveChecked;
    /** A hive tag within {@link #NEAR_HIVE_INCHES} was seen. The hive has not tipped. */
    private boolean hiveNear;
    /** Garden balls stay trapped on the skip drive until the opposite flower. */
    private boolean trapUntilOpp;
    private double commandedFieldDeg = Double.NaN;

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

        intake.start();
        depo.start();
        turret.resetTurretEncoders();
        turret.start();
        intake.flower_up();
        follower.update();
        turret.setPositionDeg(PRELOAD);

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
        turret.run_using_pid();
        updateVision();

        telemetry.addData("Turret deg", turret.getPositionDeg());
        telemetry.addData("Nearest hive", hiveRangeLabel());
        telemetry.addData("Camera", cameraReady ? "ON" : cameraMessage);
        int tagCount = cameraReady ? vision.getRawDetectionCount() : 0;
        telemetry.addData("AprilTag", tagCount > 0 ? "DETECTED" : "NOT DETECTED");
        telemetry.addData("Tag count", tagCount);
        telemetry.update();
    }

    @Override
    public void start() {
        intake.start();
        depo.start();
        turret.start();
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
        telemetry.addData("Nearest hive", hiveRangeLabel());
        telemetry.addData("Closest tag", vision.getClosestHiveTagId() < 0 ? "NONE" : vision.getClosestHiveTagId());
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Field aim", Double.isNaN(commandedFieldDeg) ? "hold" : commandedFieldDeg);
        telemetry.addData("Turret deg", turret.getPositionDeg());
        telemetry.addData("Turret target", turret.getTargetDeg());
        telemetry.addData("Depo velo", depo.getVelocity());
        telemetry.addData("Balls", intake.getBalls());
        int tagCount = cameraReady ? vision.getRawDetectionCount() : 0;
        telemetry.addData("AprilTag", tagCount > 0 ? "DETECTED" : "NOT DETECTED");
        telemetry.addData("Tag count", tagCount);
        telemetry.update();
    }

    @Override
    public void stop() {
        savePosition.endPose=follower.pose();
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
//                aimFieldDegrees(PRELOAD_FIELD_DEG);
                turret.setPositionDeg(PRELOAD);
                if (finishShot()) {
                    follower.follow(line(start, gardenApproach).linear(start, gardenApproach));
                    pathState = PathState.TO_GARDEN_APPROACH;
                }
                break;
            case TO_GARDEN_APPROACH:
                if (!follower.isBusy()) {
//                    aimFieldDegrees(GARDEN_FIELD_DEG);
                    turret.setPositionDeg(GARDEN);
                    intake.front_up();
                    trapGarden();
                    follower.follow(line(gardenApproach, gardenIntake)
                            .constant(gardenIntake)
                            .with(Robot1.foresightConfig.maxPathSpeed.at(GARDEN_INTAKE_SPEED)));
                    pathState = PathState.GARDEN_INTAKE;
                }
                break;
            case GARDEN_INTAKE:
//                aimFieldDegrees(GARDEN_FIELD_DEG);
                turret.setPositionDeg(GARDEN);
                trapGarden();
                if (!follower.isBusy()) {
                    intake.front_down();
                    pathState = PathState.GARDEN_CLEAR;
                }
                break;
            case GARDEN_CLEAR:
                if (timers.checkSecondsLast("hold", GARDEN_FLOWER_UP_SECONDS)) {
                    hiveNear = false;
                    pathState = PathState.READ_HIVE;
                } else {
                    holdGardenBalls();
                    if (!timers.isRunning("hold")) {
                        timers.start("hold");
                    }
                }
                break;
            case READ_HIVE:
                holdGardenBalls();
                rememberNearHive();
                if (timers.checkSecondsLast("look", HIVE_LOOK_SECONDS)) {
                    hiveChecked = true;
                    if (hiveNear) {
                        follower.follow(curve(shoot1Start, shoot1Control1, shoot1).linear(shoot1Start, shoot1));
                        pathState = PathState.TO_SHOOT1;
                    } else {
                        trapUntilOpp = true;
                        follower.follow(line(gardenIntake, oppFlowerApproach).constant(oppFlowerApproach));
                        pathState = PathState.TO_OPP_FLOWER;
                    }
                } else if (!timers.isRunning("look")) {
                    timers.start("look");
                }
                break;
            case TO_SHOOT1:
                holdGardenBalls();
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT1;
                }
                break;
            case SHOOT1:
//                aimFieldDegrees(SHOOT1_FIELD_DEG);
                turret.setPositionDeg(SHOOT1);
                if (finishShot()) {
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
                if (!follower.isBusy()) {
                    intake.front_down();
                    follower.follow(line(flowerIntake, flowerBack).constant(flowerBack));
                    pathState = PathState.TO_FLOWER_BACK;
                }
                break;
            case TO_FLOWER_BACK:
                holdGardenBalls();
                if (!timers.isRunning("hold")) {
                    timers.start("hold");
                } else if (!follower.isBusy() && timers.checkSecondsLast("hold", INTAKE_HOLD_SECONDS)) {
                    intake.flower_up();
                    stopCollecting();
                    follower.follow(line(flowerBack, oppFlowerApproach).constant(oppFlowerApproach));
                    pathState = PathState.TO_OPP_FLOWER;
                }
                break;
            case TO_OPP_FLOWER:
                if (trapUntilOpp && follower.isBusy()) {
                    holdGardenBalls();
                }
                if (!follower.isBusy()) {
                    trapUntilOpp = false;
                    pathState = PathState.SHOOT_OPP;
                }
                break;
            case SHOOT_OPP:
//                aimFieldDegrees(OPP_FIELD_DEG);
                turret.setPositionDeg(FLOWER);
                if (finishShot(true)) {
                    intake.front_up();
                    trapGarden();
                    follower.follow(line(oppFlowerApproach, oppFlowerIntake).constant(oppFlowerIntake));
                    pathState = PathState.TO_OPP_INTAKE;
                } else {
                    holdGardenBalls();
                }
                break;
            case TO_OPP_INTAKE:
                runFlowerIntake(oppFlowerIntake, oppFlowerApproach, PathState.TO_OPP_BACK);
                break;
            case TO_OPP_BACK:
                holdGardenBalls();
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_OPP_RETURN;
                }
                break;
            case SHOOT_OPP_RETURN:
//                aimFieldDegrees(OPP_FIELD_DEG);
                turret.setPositionDeg(FLOWER);
                if (finishShot(true)) {
                    intake.flower_up();
                    stopCollecting();
                    follower.follow(line(oppFlowerApproach, parkPrep).linear(oppFlowerApproach, parkPrep));
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
                    turret.setPositionDeg(0);
                }
                break;
            case DONE:
                break;
            default:
                break;
        }
    }

    /**
     * Field heading to the robot-relative angle {@link Turret#setPositionDeg(double)} expects.
     * Same conversion as {@link Turret#AutoAllignTo}: -(field - robot heading).
     */
    private void aimFieldDegrees(double fieldDeg) {
        commandedFieldDeg = fieldDeg;
        double headingDeg = Math.toDegrees(follower.pose().heading());
        turret.setPositionDeg(-(fieldDeg - headingDeg));
    }

    /** One shot at the normal flywheel speed. True on the loop it finishes. */
    private boolean finishShot() {
        return finishShot(false);
    }

    /**
     * One shot. {@code flowerSpeed} uses the flower flywheel speed.
     * True on the loop it finishes and the flywheel is turned off.
     */
    private boolean finishShot(boolean flowerSpeed) {
        if (!depo.isDepositOn() && !timers.isRunning("shoot")) {
            if (flowerSpeed) {
                depo.turn_on_flower();
            } else {
                depo.turn_on_deposit();
            }
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
            depo.turn_off_deposit();
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

    /**
     * A hive tag at {@link #NEAR_HIVE_INCHES} or closer means the near hive has
     * not tipped. Farther tags are left out, and one close frame is enough.
     */
    private void rememberNearHive() {
        double range = vision.getClosestHiveRange();
        if (Double.isFinite(range) && range <= NEAR_HIVE_INCHES) {
            hiveNear = true;
        }
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
                || pathState == PathState.SHOOT1
                || pathState == PathState.SHOOT_OPP
                || pathState == PathState.SHOOT_OPP_RETURN;
    }

    /** Flower is down and the rollers should keep running. The ball counter must not shut them off. */
    private boolean clearingFlower() {
        return pathState == PathState.GARDEN_CLEAR
                || pathState == PathState.READ_HIVE
                || pathState == PathState.TO_SHOOT1
                || (pathState == PathState.TO_OPP_FLOWER && trapUntilOpp)
                || pathState == PathState.TO_FLOWER_BACK
                || (pathState == PathState.TO_OPP_INTAKE && timers.isRunning("hold"))
                || pathState == PathState.TO_OPP_BACK;
    }

    /**
     * Opposite-flower intake. The front gate stays up while the robot drives in
     * with the flower down, then both stay down for the hold before backing out.
     */
    private void runFlowerIntake(Pose intakePose, Pose backPose, PathState backState) {
        if (timers.checkSecondsLast("hold", INTAKE_HOLD_SECONDS)) {
            follower.follow(line(intakePose, backPose).constant(backPose));
            pathState = backState;
        } else if (!follower.isBusy() && !timers.isRunning("hold")) {
            intake.front_down();
            intake.resetBalls();
            intakeFull = false;
            timers.start("hold");
            holdGardenBalls();
        } else if (timers.isRunning("hold")) {
            holdGardenBalls();
        } else {
            trapGarden();
        }
    }

    /** Garden rollers on and flower down. The front gate stays up until the intake path ends. */
    private void trapGarden() {
        intake.flower_down();
//        intake.back_down();
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

    private String hiveRangeLabel() {
        double range = vision.getClosestHiveRange();
        if (!Double.isFinite(range)) {
            return "NONE";
        }
        if (range <= NEAR_HIVE_INCHES) {
            return String.format("%.1f in - counts", range);
        }
        return String.format("%.1f in - ignored", range);
    }

    private String hiveLabel() {
        if (!hiveChecked) {
            return hiveNear ? "NEAR" : "looking";
        }
        return hiveNear ? "NOT TIPPED - shoot1" : "TIPPED - opp flower";
    }
}
