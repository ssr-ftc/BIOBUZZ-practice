package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
@Autonomous(name = "EntireAuto Misha", group = "Season")
public class EntireAutoMisha extends OpMode {
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
    private static final double GARDEN_FLOWER_UP_SECONDS = 1;

    /** Field headings. Converted to robot-relative when commanded. */
    private static final double PRELOAD_FIELD_DEG = 70;
    private static final double GARDEN_FIELD_DEG = 41;
    private static final double SHOOT1_FIELD_DEG = 70;
    private static final double OPP_FIELD_DEG = 296;

    private static final double PRELOAD = 4;
    private static final double GARDEN = -130;
    private static final double SHOOT1 = 0;
    private static final double FLOWER = -195;

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




    private final Pose path1 = poseFactory.of(8, 12, 270);
    private final Pose point2 = poseFactory.of(58.9663, 130, -90);
    private final Pose point2Control1 = poseFactory.of(24.2123, 113.7783, 0);
    private final Pose point2Control2 = poseFactory.of(44.6367, 84.9421, 0);
    private final Pose point2Segment1Start = poseFactory.of(58.9663, 130, 270);
    private final Pose point2Segment1End = poseFactory.of(58.9663, 130, -102);
    private final Pose point2Segment3Start = poseFactory.of(58.9663, 130, -111.7293);
    private final Pose point2Segment3End = poseFactory.of(58.9663, 130, 270);
    private final Pose point3 = poseFactory.of(49, 129, 90);
    private final Pose point3Control1 = poseFactory.of(53.6906, 86.1718, 0);
    private final Pose point4 = poseFactory.of(59.0409, 129, 90);
    private final Pose point4Control1 = poseFactory.of(46.1515, 98.6592, 0);
    private final Pose point5 = poseFactory.of(14, 94.9534, 0);
    private final Pose point5Control1 = poseFactory.of(45.8176, 87.9668, 0);
    private final Pose point5Segment1Start = poseFactory.of(10.9862, 94.9534, 90);
    private final Pose point5Segment1End = poseFactory.of(10.9862, 94.9534, 67);
    private final Pose point5Segment3Start = poseFactory.of(10.9862, 94.9534, -3.4417);
    private final Pose point5Segment3End = poseFactory.of(10.9862, 94.9534, 0);


    public Path toOppShot() {
        return Paths.curve(path1, point2Control1, point2Control2, point2).heading(Interpolator.piecewise().until(0.0865, Interpolator.linear(point2Segment1Start, point2Segment1End)).until(0.945, Interpolator.tangent.reverse()).until(1, Interpolator.linear(point2Segment3Start, point2Segment3End)));
    }

    public Path toBackFlower() {
        return Paths.curve(point2, point3Control1, point3).linear(point2, point3);
    }

    public Path toThirdShot() {
        return Paths.curve(point3, point4Control1, point4).constant(point4);
    }

    public Path toPark() {
        return Paths.curve(point4, point5Control1, point5).heading(Interpolator.piecewise().until(0.0936, Interpolator.linear(point5Segment1Start, point5Segment1End)).until(0.9042, Interpolator.tangent.reverse()).until(1, Interpolator.linear(point5Segment3Start, point5Segment3End)));
    }

    // Paths
    private Path toGardenApproachPath;
    private Path gardenIntakePath;
    private Path gardenToShoot1Path;
    private Path shoot1ToFlowerPath;
    private Path flowerIntakePath;
    private Path flowerBackPath;
    private Path flowerToOppFlowerPath;
    private Path gardenToOppFlowerPath;
    private Path oppFlowerIntakePath;
    private Path oppFlowerBackPath;
    private Path oppFlowerToParkPrepPath;
    private Path parkPath;


    private Follower follower;
    private Intake intake;
    private Depo depo;
    private Turret turret;
    private Timer timers;
    private final BioBuzzVision vision = new BioBuzzVision();

    private int pathState = 1;
    private boolean intakeFull;
    private boolean cameraReady;
    private String cameraMessage = "";
    private boolean hiveChecked;
    /** A hive tag within {@link #NEAR_HIVE_INCHES} was seen. The hive has not tipped. */
    private boolean hiveNear;
    /** Garden balls stay trapped on the skip drive until the opposite flower. */
    private boolean trapUntilOpp;
    private double commandedFieldDeg = Double.NaN;
    ElapsedTime didytimer;
    ElapsedTime toleranceTimer;

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

        didytimer = new ElapsedTime();
        toleranceTimer = new ElapsedTime();

        toGardenApproachPath = line(start, gardenApproach).linear(start, gardenApproach);
        gardenIntakePath = line(gardenApproach, gardenIntake)
                .constant(gardenIntake)
                .with(Robot1.foresightConfig.maxPathSpeed.at(GARDEN_INTAKE_SPEED));
        gardenToShoot1Path = curve(shoot1Start, shoot1Control1, shoot1)
                .linear(shoot1Start, shoot1);
        shoot1ToFlowerPath = line(shoot1, flowerApproach).constant(flowerApproach);
        flowerIntakePath = line(flowerApproach, flowerIntake).constant(flowerIntake);
        flowerBackPath = line(flowerIntake, flowerBack).constant(flowerBack);
        flowerToOppFlowerPath = line(flowerBack, oppFlowerApproach).constant(oppFlowerApproach);
        gardenToOppFlowerPath = line(gardenIntake, oppFlowerApproach).constant(oppFlowerApproach);
        oppFlowerIntakePath = line(oppFlowerApproach, oppFlowerIntake).constant(oppFlowerIntake);
        oppFlowerBackPath = line(oppFlowerIntake, oppFlowerApproach).constant(oppFlowerApproach);
        oppFlowerToParkPrepPath = line(oppFlowerApproach, parkPrep).linear(oppFlowerApproach, parkPrep);
        parkPath = line(parkPrep, park).constant(park);

        intake.start();
        depo.start();
        turret.resetTurretEncoders();
        turret.start();
        intake.flower_up();
        follower.update();
        turret.setPositionDeg(10);

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
            case 1:
                depo.set_target_velocity(1500);
                turret.setPositionDeg(PRELOAD);
                pathState=11;
                break;
            case 11:
                if(depo.reachedvelocity()){
                    didytimer.reset();

                    pathState=12;
                }
                break;
            case 12:
                if(didytimer.seconds()>1.5){
                    depo.open_hatch();
                    intake.intake_on();
                    didytimer.reset();
                    pathState=13;
                }
                break;
            case 13:
                if(didytimer.seconds()>1.5){
                    depo.close_hatch();
                    follower.follow(toGardenApproachPath);
                    toleranceTimer.reset();
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()||toleranceTimer.seconds()>3) {
//                    aimFieldDegrees(GARDEN_FIELD_DEG);
                    turret.setPositionDeg(GARDEN);
                    intake.front_up();
                    trapGarden();
                    follower.follow(gardenIntakePath);
                    toleranceTimer.reset();
                    pathState = 3;
                }
                break;
            case 3:
//                aimFieldDegrees(GARDEN_FIELD_DEG);
                turret.setPositionDeg(GARDEN);
                trapGarden();
                if (!follower.isBusy()||toleranceTimer.seconds()>2) {
                    intake.front_down();
                    pathState = 4;
                }
                break;
            case 4:
                if (timers.checkSecondsLast("hold", GARDEN_FLOWER_UP_SECONDS)) {
                    follower.follow(toOppShot());
                    toleranceTimer.reset();
                    didytimer.reset();
                    pathState = 50;
                } else {
                    holdGardenBalls();
                    if (!timers.isRunning("hold")) {
                        timers.start("hold");
                    }
                }
                break;
            case 50:
                if(didytimer.seconds()>1){
                    depo.set_target_velocity(1500);
                    turret.setPositionDeg(0);
//                    intake.flower_up();
//                    intake.front_up();
                    intake.back_up();
                    pathState=5;
                }
                break;

            case 5://this is opp flower
                if (!follower.isBusy()||toleranceTimer.seconds()>4) {
                    didytimer.reset();
                    pathState = 60;
                }
                break;
            case 60:
                if(didytimer.seconds()>1){
                    depo.open_hatch();
                    intake.intake_on();
                    didytimer.reset();
                    pathState=61;
                }
                break;
            case 61:
                if(didytimer.seconds()>2){
                    depo.set_target_velocity(0);
                    depo.close_hatch();
                    intake.flower_down();
                    intake.front_up();
                    follower.follow(toBackFlower());
                    toleranceTimer.reset();
                    pathState=6;

                }
                break;
            case 6:
                if(!follower.isBusy()||toleranceTimer.seconds()>3){//reached flower
                    didytimer.reset();
                    pathState = 70;
                }
                break;
            case 70:
                if(didytimer.seconds()>1.5){
                    follower.follow(toThirdShot());
                    toleranceTimer.reset();
                    intake.front_down();
                    intake.intake_on();
                    depo.set_target_velocity(1480);
                    turret.setPositionDeg(-180);
                    pathState=7;
                }
                break;
            case 7:
                if(!follower.isBusy()||toleranceTimer.seconds()>3){
                    intake.intake_on();
                    depo.set_target_velocity(1480);
                    didytimer.reset();
                    pathState=8;
                }
                break;
            case 8:
                if(didytimer.seconds()>1){//settled
                    depo.open_hatch();
                    intake.intake_on();
                    didytimer.reset();
                    pathState=81;
                }
                break;
            case 81:
                if(didytimer.seconds()>2.5){
                    intake.intake_off();
                    intake.front_up();
                    intake.flower_up();
                    depo.set_target_velocity(0);
                    depo.close_hatch();
                    follower.follow(toPark());
                    pathState=9;
                }
                break;
            case 9:
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
                depo.set_target_velocity(1480);
            } else {
                depo.set_target_velocity(1480);
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
        return pathState == 1
                || pathState == 6
                || pathState == 9;
    }

    /** Flower is down and the rollers should keep running. The ball counter must not shut them off. */
    private boolean clearingFlower() {
        return pathState == 4
                || (pathState == 5 && trapUntilOpp)
                || (pathState == 7 && timers.isRunning("hold"))
                || pathState == 8;
    }

    /**
     * Opposite-flower intake. The front gate stays up while the robot drives in
     * with the flower down, then both stay down for the hold before backing out.
     */
    private void runFlowerIntake(Path backPath, int backState) {
        if (timers.checkSecondsLast("hold", INTAKE_HOLD_SECONDS)) {
            follower.follow(backPath);
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
