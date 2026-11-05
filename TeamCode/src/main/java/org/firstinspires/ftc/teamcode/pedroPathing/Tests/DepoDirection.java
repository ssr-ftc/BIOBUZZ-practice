package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "depo Direction Test")
public class DepoDirection extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor depo = hardwareMap.get(DcMotor.class, "depo");
        DcMotor depo1 = hardwareMap.get(DcMotor.class, "depo1");

        depo.setDirection(DcMotorSimple.Direction.FORWARD);
        depo1.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            depo.setPower(gamepad1.left_stick_y);
            depo1.setPower(gamepad1.right_stick_y);

            telemetry.addData("depo", depo.getPower());
            telemetry.addData("depo1", depo1.getPower());
            telemetry.addData("right stick y", gamepad1.right_stick_y);
            telemetry.addData("left stick y", gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}