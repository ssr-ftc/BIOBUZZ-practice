package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.DepoLM1;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp(name = "Hatch Test 1")
public class HatchTest1 extends OpMode {
    public DepoLM1 hatch;


    @Override
    public void init() {
        hatch = new DepoLM1(hardwareMap);
        hatch.close_hatch();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) hatch.open_hatch();
        if (gamepad1.bWasPressed()) hatch.close_hatch();
    }
}