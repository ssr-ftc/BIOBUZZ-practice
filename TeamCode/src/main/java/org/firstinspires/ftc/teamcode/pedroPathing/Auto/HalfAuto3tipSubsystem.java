package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Robot1;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Turret;

@Autonomous(name = "Half Auto 3 Tip Subsystems", group = "Autonomous")
public class HalfAuto3tipSubsystem extends OpMode {
    private static final double HIVE_SPLIT_Y = 71;
    private static final double FULL_INTAKE_BALLS = 4;

    private final PoseFactory poses = PoseFactory.degrees();
    private final Pose start = poses.of(56, 9, 180);
    private final Pose gardenIntake = poses.of(24, 9, 180);
    private final Pose gardenIntakeSlow = poses.of(13.5, 9, 180);
    private final Pose shoot1 = poses.of(38, 9, 180);
    private final Pose flowerIntake = poses.of(24, 48, 180);
    private final Pose flowerIntakeSlow = poses.of(14, 48, 180);
    private final Pose shoot2 = poses.of(38, 11, 90);
    private final Pose parkPrep = poses.of(18, 95, 0);
    private final Pose end = poses.of(10, 95, 0);

    private Follower follower;
    private Intake intake;
    private Turret turret;
    private Timer timers;
    private int pathState = 1;
    private boolean intakeFull;

    @Override
    public void init() {
        follower = Robot1.createFollower(hardwareMap);
        follower.setPose(start);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        timers = new Timer();
        timers.createNew("hold");

        turret.resetTurretEncoders();
        turret.setPositionDeg(0);
        intake.flower_up();
        follower.update();
    }

    @Override
    public void init_loop() {
        follower.update();
        aimTurret();
        turret.run_using_pid();
        telemetry.addData("Turret deg", turret.getPositionDeg());
        telemetry.addData("Turret target", turret.getTargetDeg());
        telemetry.update();
    }

    @Override
    public void start() {
        intake.start();
        turret.start();
        turret.setPositionDeg(0);
        intake.resetBalls();
    }

    @Override
    public void loop() {
        follower.update();
        aimTurret();
        turret.run_using_pid();
        updateIntakeCount();
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
            case 1:
                collectGarden();
                follower.follow(line(start, gardenIntake).linear(start, gardenIntake));
                pathState = 2;
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.follow(line(gardenIntake, gardenIntakeSlow).constant(gardenIntakeSlow));
                    pathState = 3;
                }
                break;
            case 3:
                if (timers.checkSecondsLast("hold", 1)) {
                    stopCollecting();
                    follower.follow(line(gardenIntakeSlow, shoot1).constant(shoot1));
                    pathState = 4;
                } else if (!follower.isBusy() && !timers.isRunning("hold")) {
                    collectGarden();
                    timers.start("hold");
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.follow(line(shoot1, flowerIntake).constant(flowerIntake));
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    prepareFlower();
                    follower.follow(line(flowerIntake, flowerIntakeSlow).constant(flowerIntakeSlow));
                    pathState = 6;
                }
                break;
            case 6:
                if (timers.checkSecondsLast("hold", 1)) {
                    intake.intake_off();
                    follower.follow(line(flowerIntakeSlow, flowerIntake).linear(flowerIntakeSlow, flowerIntake));
                    pathState = 7;
                } else if (!follower.isBusy() && !timers.isRunning("hold")) {
                    intake.resetBalls();
                    intakeFull = false;
                    collectFlower();
                    timers.start("hold");
                }
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
        intake.back_down();
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

    @Override
    public void stop() {
        stopCollecting();
        intake.flower_up();
        turret.turretOff();
    }
}
