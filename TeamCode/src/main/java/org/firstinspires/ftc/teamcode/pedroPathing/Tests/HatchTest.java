package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Hatch Test")
public class HatchTest extends OpMode {
    private Follower follower;
    private double curr = 0;
    Servo hatch;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        hatch = hardwareMap.get(Servo.class, "hatch");
        hatch.setPosition(0);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) hatch.setPosition(0);
        if (gamepad1.dpadUpWasPressed()) {
            curr = curr + 0.01;
            hatch.setPosition(curr);
        }
        if (gamepad1.dpadDownWasPressed()) {
            curr = curr - 0.01;
            hatch.setPosition(curr);
        }
        if (gamepad1.bWasPressed()) {
            hatch.setPosition(0.14);
        }
        curr = hatch.getPosition();
        telemetry.addData("Hatch:", curr);
    }
}