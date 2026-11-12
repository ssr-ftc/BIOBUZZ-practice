package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Robot1;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Depo;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "alone auto 2 tips", group = "Autonomous")
public class FullAuto2tip extends OpMode {
    private static final double HIVE_SPLIT_Y = 71;
    private static final double FULL_INTAKE_BALLS = 4;

    private final PoseFactory poses = PoseFactory.degrees();
    private final Pose start = poses.of(56, 9, 180);
    private final Pose gardenIntakeMid = poses.of(24, 9, 180);
    private final Pose gardenIntakeEnd = poses.of(13.5, 9, 180);
    private final Pose checkTip = poses.of(56, 10, 180);
    private final Pose bottomShot = poses.of(60, 131, 90);
    private final Pose flowerIntake = poses.of(24, 48, 180);
    private final Pose flowerIntakeSlow = poses.of(14, 48, 180);
    private final Pose shoot2 = poses.of(38, 11, 90);
    private final Pose parkPrep = poses.of(18, 95, 0);
    private final Pose end = poses.of(10, 95, 0);

    double startShotVelo=1500;

    private Follower follower;
    private Intake intake;
    private Turret turret;
    private Depo depo;
    private Timer timers;
    private int pathState = 11;
    private boolean intakeFull;
    private AprilTagProcessor tagProcessor;
    private VisionPortal portal;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 9, 6, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -45, 180, 0);

    @Override
    public void init() {
        follower = Robot1.createFollower(hardwareMap);
        follower.setPose(start);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        depo = new Depo(hardwareMap);
        timers = new Timer();
        timers.createNew("hold");
        timers.createNew("depo");
        timers.createNew("check");

        turret.start();
        turret.resetTurretEncoders();
        turret.setPositionDeg(90);
        intake.start();
        depo.start();
        follower.update();
        tagProcessor = new AprilTagProcessor.Builder().setCameraPose(cameraPosition, cameraOrientation).build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor)
                .build();

    }

    @Override
    public void init_loop() {
        follower.update();
        checkForTag();
        turret.run_using_pid();
        telemetry.update();
    }
    private void checkForTag() {
        double tags = 0;
        for (AprilTagDetection d : tagProcessor.getDetections()) {
            tags+=1;
            telemetry.addData("id",d.id);
            telemetry.addData("y pos", d.center.y);
            telemetry.addData("x pos", d.center.x);

        }
        telemetry.addData("tags", tags);
    }

    @Override
    public void start() {
        intake.resetBalls();
        depo.set_target_velocity(startShotVelo);
    }

    @Override
    public void loop() {
        follower.update();
//        aimTurret();
        turret.run_using_pid();
        depo.run_using_pid();
//        updateIntakeCount();
        autonomousPathUpdate();

        telemetry.addData("Path", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Intake", intake.getMotorPower());
        telemetry.addData("Balls", intake.getBalls());
        telemetry.addData("Turret deg", turret.getPositionDeg());
        telemetry.addData("Turret target", turret.getTargetDeg());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 11:
                if(depo.reachedvelocity() && !timers.isRunning("depo")){
                    timers.start("depo");
                }
                shootingFunc();
                if(depo.get_target_velocity()==0){
                    pathState = 12;
                }
                break;
            case 12:
                collectGarden();//prep intake
                follower.follow(line(start, gardenIntakeMid).linear(start, gardenIntakeMid));//map path to garden mid pose
                pathState = 2;
                break;
            case 2:
                if (!follower.isBusy()) {//if reached garden midpose
                    follower.follow(line(gardenIntakeMid, gardenIntakeEnd).constant(gardenIntakeEnd));//map path to garden
                    pathState = 3;
                }
                break;
            case 3:
                if (timers.checkSecondsLast("hold", 1)) {//after intake finished build next path
                    stopCollecting();
                    follower.follow(line(gardenIntakeEnd, checkTip).constant(checkTip));
                    pathState = 4;
                } else if (!follower.isBusy() && !timers.isRunning("hold")) {//when reached garden pose sit for 1 sec
                    collectGarden();
                    timers.start("hold");
                }
                break;
            case 4:
                if (!follower.isBusy()) {//reached the checktip
                    timers.start("check");
                }
                if (timers.checkSecondsLast("check",0.5)) {//waited
                    follower.follow(line(checkTip, bottomShot).linear(checkTip,bottomShot));//build path to bottom shot
                    depo.set_target_velocity(startShotVelo);
                    turret.setPositionDeg(180);
                    pathState = 5;
                }
                break;
            case 5://driving to bottom
                shootingFunc();
                if (!follower.isBusy() && !timers.isRunning("depo")) {//reached shooting pose
                    timers.start("depo");
                }
                if(depo.get_target_velocity()==0){
//                    follower.follow(line(bottomShot, flowerIntakeSlow).constant(flowerIntakeSlow));
                    pathState = 6;
                }
                break;
            case 6:

                break;
            case 7:
                if (!follower.isBusy()) {
                    intake.flower_up();
                    stopCollecting();
                    follower.follow(line(flowerIntake, shoot2).linear(flowerIntake, shoot2));
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.follow(line(shoot2, parkPrep).linear(shoot2, parkPrep));
                    pathState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.follow(line(parkPrep, end).constant(end));
                    pathState = 10;
                }
                break;
            default:
                break;
        }
    }

    /** Garden hive below the field split, flower hive above it. Same rule as Solo TeleOp. */
    private void aimTurret() {
        Pose robot = follower.pose();
        if (robot.y() > HIVE_SPLIT_Y) {
            turret.faceFlowerHive(robot);
        } else {
            turret.faceGardenHive(robot);
        }
    }

    /** Ball counting moved onto Intake with the distance sensors. */
    private void updateIntakeCount() {
        if (!intake.intaking() || intakeFull) {
            return;
        }
        intake.countBalls();
        if (intake.getBalls() >= FULL_INTAKE_BALLS) {
            intakeFull = true;
            stopCollecting();
        }
    }

    /** Garden pickup: rollers on and both gates down while the flower stays up. */
    private void collectGarden() {
        if (intakeFull) {
            return;
        }
        intake.flower_up();
        intake.front_down();
//        intake.back_down();
        intake.intake_on();
    }

    /** Flower pickup: gates up so the flower can reach. */
    private void prepareFlower() {
        intake.front_up();
        intake.back_up();
        intake.flower_down();
        intake.intake_off();
    }

    private void collectFlower() {
        if (intakeFull) {
            return;
        }
        intake.front_up();
        intake.back_up();
        intake.flower_down();
        intake.intake_on();
    }

    private void stopCollecting() {
        intake.intake_off();
        intake.front_up();
        intake.back_up();
    }
    void shootingFunc(){

        if(timers.checkSeconds("depo",0.3)){
            intake.intake_on();
            depo.open_hatch();
        }
        if(timers.checkSecondsLast("depo", 1.5)){
            depo.close_hatch();
            depo.set_target_velocity(0);
        }
    }


    @Override
    public void stop() {
        stopCollecting();
        intake.flower_up();
        turret.turretOff();
    }

}
