package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Robot1;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Turret;


@TeleOp(name = "Turret only", group = "Templates")
public class TurretOnly extends OpMode {
    Pose localGarden = new Pose(64.5, 8.5, Math.toRadians(90));
    Pose localFlower = new Pose(64.5,133.5, Math.toRadians(270));

    private Follower follower;//this is pedropathing follower
    private double speedScale = 1;
    Turret turret;
    boolean useTurret = true;
    boolean autoTracking = false;


    @Override
    public void init() {//this is called once when the driver presses init
        follower = Robot1.createFollower(hardwareMap);
        turret = new Turret(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

        telemetry.addLine("Initialized");
        telemetry.addData("Status", "Press start to run TeleOp");
        telemetry.addData("Starting Pose", "(%.1f, %.1f, %.1f)", 0.0, 0.0, 0.0);
        telemetry.update();
    }


    @Override
    public void start() {//this is called once when the driver presses start
        turret.setPositionDeg(0);
    }

    @Override
    public void loop() {//this is called repeatedly during the op mode
        if(gamepad1.shareWasPressed()){
            useTurret = !useTurret;
        }
        if(gamepad1.crossWasPressed()){
            autoTracking = !autoTracking;
        }
        if(gamepad1.circleWasPressed()){
            follower.setPose(localGarden);
        }
        if(autoTracking){
            turret.faceGardenHive(follower.pose());
        }
        if(useTurret){
            turret.run_using_pid();
        }
        else turret.turretOff();
        if(gamepad1.psWasPressed()){
            turret.resetTurretEncoders();
        }
        if(gamepad1.rightBumperWasPressed()){
            turret.setPositionDeg(turret.getTargetDeg()+30);
        }if(gamepad1.leftBumperWasPressed()){
            turret.setPositionDeg(turret.getTargetDeg()-30);
        }
        if(gamepad1.dpadDownWasPressed()) turret.setPositionDeg(180);
        if(gamepad1.dpadUpWasPressed()) turret.setPositionDeg(0);
        if(gamepad1.dpadLeftWasPressed()) turret.setPositionDeg(270);
        if(gamepad1.dpadRightWasPressed()) turret.setPositionDeg(90);
        driveWithPedro();
        sendTelemetry();
        follower.update();
    }

    private void driveWithPedro() {

        double forward = -gamepad1.left_stick_y * speedScale;
        double strafe = -gamepad1.left_stick_x  * speedScale;
        double turn = -gamepad1.right_stick_x * speedScale;

        follower.manual(forward, strafe, turn);
    }



    private void sendTelemetry() {
        telemetry.addData("turret running",useTurret);
        telemetry.addData("auto tracking",autoTracking);
        telemetry.addData("turret position",turret.getPositionDeg());
        telemetry.addData("target position",turret.getTargetDeg());
        telemetry.addData("robot x",follower.pose().x());
        telemetry.addData("robot y",follower.pose().y());
        telemetry.addData("robot heading",Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }
}
