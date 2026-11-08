package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Robot1;


@TeleOp(name = "Servos tester intake (intakes,flower)")
public class ServoTest2 extends OpMode {

    private Follower follower;//this is pedropathing follower
    private double speedScale = 0.8;
    Servo flower;
    Servo front;
    Servo back;

    @Override
    public void init() {//this is called once when the driver presses init
        follower = Robot1.createFollower(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));
        flower = hardwareMap.get(Servo.class,"flower");
        front = hardwareMap.get(Servo.class,"front");
        back = hardwareMap.get(Servo.class,"back");
        telemetry.addLine("Initialized");
        telemetry.addData("Status", "Press start to run TeleOp");
        telemetry.addData("Starting Pose", "(%.1f, %.1f, %.1f)", 0.0, 0.0, 0.0);
        telemetry.update();
    }

    @Override
    public void start() {//this is called once when the driver presses start
//        flap.setPosition(0.5);
        flower.setPosition(0.5);
        front.setPosition(0.5);
        back.setPosition(0.5);
    }

    @Override
    public void loop() {//this is called repeatedly during the op mode
        follower.update();
//        driveWithPedro();
        if(gamepad1.dpadUpWasPressed()) flower.setPosition(flower.getPosition()+0.01);//dpad up/down for flower intake
        if(gamepad1.dpadDownWasPressed()) flower.setPosition(flower.getPosition()-0.01);

//        if(gamepad1.dpadLeftWasPressed()) flap.setPosition(flap.getPosition()-0.01);//dpad left/right for flap
//        if(gamepad1.dpadRightWasPressed()) flap.setPosition(flap.getPosition()+0.01);

        if(gamepad1.rightBumperWasPressed()) front.setPosition(front.getPosition()+0.01);//left and right bumpers for front intake
        if(gamepad1.leftBumperWasPressed()) front.setPosition(front.getPosition()-0.01);

        if(gamepad1.rightTriggerWasPressed()) back.setPosition(back.getPosition()+0.01);//left and right triggers for back intake
        if(gamepad1.leftTriggerWasPressed()) back.setPosition(back.getPosition()-0.01);
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
        telemetry.addLine("dpad up/down for flower intake");
//        telemetry.addLine("dpad left/right for flap");
        telemetry.addLine("left and right bumpers for front intake");
        telemetry.addLine("left and right triggers for back intake");
        telemetry.addData("flower", flower.getPosition());
//        telemetry.addData("flap", flap.getPosition());
        telemetry.addData("front", front.getPosition());
        telemetry.addData("back", back.getPosition());
        telemetry.update();
    }
}
//hi