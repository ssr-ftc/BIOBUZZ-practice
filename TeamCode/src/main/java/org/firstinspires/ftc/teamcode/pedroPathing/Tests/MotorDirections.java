package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Motor Direction Test")
public class MotorDirections extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "rf");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "rb");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            fl.setPower(gamepad1.left_stick_y);
            fr.setPower(gamepad1.left_stick_y);
            bl.setPower(gamepad1.left_stick_y);
            br.setPower(gamepad1.left_stick_y);

            telemetry.addData("FL", fl.getPower());
            telemetry.addData("FR", fr.getPower());
            telemetry.addData("BL", bl.getPower());
            telemetry.addData("BR", br.getPower());
            telemetry.update();
        }
    }
}