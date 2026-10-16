package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name = "Template Pedro TeleOp", group = "Templates")
public class ServoTest extends OpMode {

    private Follower follower;//this is pedropathing follower
    private double speedScale = 0.8;
    Servo hatch;

    Servo flower;


    @Override
    public void init() {//this is called once when the driver presses init
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        hatch = hardwareMap.get(Servo.class,"hatch");
        flower = hardwareMap.get(Servo.class,"flower_intake");
        telemetry.addLine("Initialized");
        telemetry.addData("Status", "Press start to run TeleOp");
        telemetry.addData("Starting Pose", "(%.1f, %.1f, %.1f)", 0.0, 0.0, 0.0);
        telemetry.update();
    }


    @Override
    public void start() {//this is called once when the driver presses start
        flower.setPosition(0.5);
        hatch.setPosition(0.5);
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {//this is called repeatedly during the op mode
        follower.update();
//        driveWithPedro();
        if(gamepad1.dpadUpWasPressed()) hatch.setPosition(hatch.getPosition()+0.01);
        if(gamepad1.dpadDownWasPressed()) hatch.setPosition(hatch.getPosition()-0.01);
        if(gamepad1.dpadLeftWasPressed()) flower.setPosition(flower.getPosition()-0.01);
        if(gamepad1.dpadRightWasPressed()) flower.setPosition(flower.getPosition()+0.01);
        //0.4 is hatch open
        //0.54 is close
        //0.66 is open2
        sendTelemetry();
    }

    private void driveWithPedro() {

        double forward = gamepad1.left_stick_y * speedScale;
        double strafe = gamepad1.left_stick_x  * speedScale;
        double turn = gamepad1.right_stick_x * speedScale;

        follower.setTeleOpDrive(forward, strafe, turn, true);
    }



    private void sendTelemetry() {
        telemetry.addLine("this is telemetry");
        telemetry.addData("hatch",hatch.getPosition());
        telemetry.addData("flower",flower.getPosition());
        telemetry.update();
    }
}
