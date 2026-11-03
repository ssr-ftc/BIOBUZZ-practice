package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "turret test with motor", group = "tuning")
public class TurretTestMotor extends LinearOpMode {

    public static double p = 0.0078;
    public static double i = 0.055;
    public static double d = 0.00039;
    public static double kF = 0.0002;
    public static double targetPos = 0;

    private DcMotorEx turret;
    private PIDFController pid;

    @Override
    public void runOpMode() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid = new PIDFController(p, i, d, kF);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Hold left bumper for manual stick. Otherwise PID runs to targetPos.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            pid.setPIDF(p, i, d, kF);

            if (gamepad1.dpad_up) {
                targetPos += 2;
            }
            if (gamepad1.dpad_down) {
                targetPos -= 2;
            }

            double currentPos = turret.getCurrentPosition();
            double power;

            if (gamepad1.left_bumper) {
                power = gamepad1.left_stick_y;
            } else {
                power = pid.calculate(currentPos, targetPos);
                power = Math.max(-1.0, Math.min(1.0, power));
            }

            turret.setPower(power);

            telemetry.addData("targetPos", targetPos);
            telemetry.addData("currentPos", currentPos);
            telemetry.addData("error", targetPos - currentPos);
            telemetry.addData("power", power);
            telemetry.addData("p", p);
            telemetry.addData("i", i);
            telemetry.addData("d", d);
            telemetry.addData("kF", kF);
            telemetry.update();
        }

        turret.setPower(0);
    }
}
