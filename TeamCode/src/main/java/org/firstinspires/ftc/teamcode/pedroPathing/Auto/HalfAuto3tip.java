package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Robot1;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Depo;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Intake;

@Autonomous(name = "Half Auto 3 Tip", group = "Autonomous")
public class HalfAuto3tip extends OpMode {
    private final PoseFactory poses = PoseFactory.degrees();
    private final Pose start = poses.of(56, 9, 180);
    private final Pose gardenIntake = poses.of(24, 9, 180);
    private final Pose gardenIntakeSlow = poses.of(13.5, 9, 180);
    private final Pose shoot1 = poses.of(38, 9, 180);
    private final Pose flowerIntake = poses.of(24, 47, 180);
    private final Pose flowerIntakeSlow = poses.of(14, 47, 180);
    private final Pose shoot2 = poses.of(38, 9, 90);
    private final Pose parkPrep = poses.of(18, 95, 0);
    private final Pose end = poses.of(10, 95, 0);

    private Follower follower;
    private Intake intake;
    private Depo depo;
    private Turret turret;
    private Timer timers;
    private int pathState = 1;

    @Override
    public void init() {
        follower = Robot1.createFollower(hardwareMap);
        follower.setPose(start);
        intake = new Intake(hardwareMap);
        depo = new Depo(hardwareMap);
        turret = new Turret(hardwareMap);
        timers = new Timer();
        timers.createNew("shoot");
        timers.createNew("hold");

        turret.resetTurretEncoders();
        turret.setPositionDeg(90);
        intake.flower_up();
        follower.update();
    }

    @Override
    public void init_loop() {
        // Brake the wheels before the turret PID runs. The drivetrain is created in coast,
        // and a full-power turret move yaws the robot while the wheels are free.
        follower.update();
        turret.run_using_pid();
        telemetry.addData("Turret deg", turret.getPositionDeg());
        telemetry.addLine("Turret moving to 90");
        telemetry.update();
    }

    @Override
    public void start() {
        intake.start();
        depo.start();
        turret.start();
        turret.setPositionDeg(90);
    }

    @Override
    public void loop() {
        follower.update();
        turret.run_using_pid();
        depo.run_using_pid();
        autonomousPathUpdate();

        telemetry.addData("Path", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Turret deg", turret.getPositionDeg());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1:
                if (timers.checkSecondsLast("shoot", 1.5)) {
                    stopShooting();
                    turret.setPositionDeg(0);
                    intake.intake_on();
                    follower.follow(line(start, gardenIntake).linear(start, gardenIntake));
                    pathState = 2;
                } else if (!timers.isRunning("shoot")) {
                    shoot();
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.follow(line(gardenIntake, gardenIntakeSlow).constant(gardenIntakeSlow));
                    pathState = 3;
                }
                break;
            case 3:
                if (timers.checkSecondsLast("hold", 1)) {
                    follower.follow(line(gardenIntakeSlow, shoot1).constant(shoot1));
                    pathState = 4;
                } else if (!follower.isBusy() && !timers.isRunning("hold")) {
                    timers.start("hold");
                }
                break;
            case 4:
                if (timers.checkSecondsLast("shoot", 1.5)) {
                    stopShooting();
                    turret.setPositionDeg(0);
                    follower.follow(line(shoot1, flowerIntake).constant(flowerIntake));
                    pathState = 5;
                } else if (!follower.isBusy() && !timers.isRunning("shoot")) {
                    intake.intake_off();
                    turret.setPositionDeg(113);
                    shoot();
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    intake.flower_down();
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
                    intake.intake_on();
                    timers.start("hold");
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    intake.flower_up();
                    follower.follow(line(flowerIntake, shoot2).linear(flowerIntake, shoot2));
                    pathState = 8;
                }
                break;
            case 8:
                if (timers.checkSecondsLast("shoot", 1.5)) {
                    stopShooting();
                    turret.setPositionDeg(0);
                    follower.follow(line(shoot2, parkPrep).linear(shoot2, parkPrep));
                    pathState = 9;
                } else if (!follower.isBusy() && !timers.isRunning("shoot")) {
                    turret.setPositionDeg(27);
                    shoot();
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.follow(line(parkPrep, end).constant(end));
                    pathState = 10;
                }
                break;
            case 10:
                break;
            default:
                break;
        }
    }

    private void shoot() {
        depo.turn_on_deposit();
        depo.open_hatch();
        timers.start("shoot");
    }

    private void stopShooting() {
        depo.turn_off_deposit();
        depo.close_hatch();
    }

    @Override
    public void stop() {
        stopShooting();
        intake.intake_off();
        intake.flower_up();
    }
}
