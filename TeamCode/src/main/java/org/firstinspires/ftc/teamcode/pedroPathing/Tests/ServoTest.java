package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Robot1;

@TeleOp(name = "Servos tester depo (hood,hatch,cam)")
public class ServoTest extends OpMode {

    private Follower follower;//this is pedropathing follower
    private double speedScale = 0.8;
    Servo hatch;
    Servo cam;
    Servo hood;


    @Override
    public void init() {//this is called once when the driver presses init
        follower = Robot1.createFollower(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));
        hatch = hardwareMap.get(Servo.class,"hatch");
        hood = hardwareMap.get(Servo.class,"hood");
        cam = hardwareMap.get(Servo.class,"camera");
        telemetry.addLine("Initialized");
        telemetry.addData("Status", "Press start to run TeleOp");
        telemetry.addData("Starting Pose", "(%.1f, %.1f, %.1f)", 0.0, 0.0, 0.0);
        telemetry.update();
    }

    @Override
    public void start() {//this is called once when the driver presses start
        hood.setPosition(0.5);
        hatch.setPosition(0.5);
        cam.setPosition(0.5);
    }

    @Override
    public void loop() {//this is called repeatedly during the op mode
        follower.update();
//        driveWithPedro();
        if(gamepad1.dpadRightWasPressed()) hatch.setPosition(hatch.getPosition()+0.01);//dpad left/right for hatch
        if(gamepad1.dpadLeftWasPressed()) hatch.setPosition(hatch.getPosition()-0.01);

        if(gamepad1.dpadDownWasPressed()) hood.setPosition(hood.getPosition()-0.01);//dpad up/down for hood
        if(gamepad1.dpadUpWasPressed()) hood.setPosition(hood.getPosition()+0.01);

        if(gamepad1.rightBumperWasPressed()) cam.setPosition(cam.getPosition()+0.01);//left and right bumpers for cam
        if(gamepad1.leftBumperWasPressed()) cam.setPosition(cam.getPosition()-0.01);
        //0.4 is hatch open
        //0.54 is close
        //0.66 is open2
        sendTelemetry();
    }
    private void driveWithPedro() {

        double forward = gamepad1.left_stick_y * speedScale;
        double strafe = gamepad1.left_stick_x  * speedScale;
        double turn = gamepad1.right_stick_x * speedScale;

        follower.manual(forward, strafe, turn);
    }
    private void sendTelemetry() {
        telemetry.addLine("dpad left/right for hatch");
        telemetry.addLine("dpad up/down for hood");
        telemetry.addLine("left and right bumpers for cam");
        telemetry.addData("hatch",hatch.getPosition());
        telemetry.addData("hood", hood.getPosition());
        telemetry.addData("camera",cam.getPosition());
        telemetry.update();
    }
}
//hi