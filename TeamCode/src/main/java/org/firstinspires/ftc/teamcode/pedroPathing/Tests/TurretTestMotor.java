package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "turret test with motor", group = "tuning")
@Config
public class TurretTestMotor extends LinearOpMode {

    PIDFController pid;
    private DcMotorEx turret;   // Has encoder

    // PID coefficients (tune these live in FTC Dashboard)
    public double power;
    public static volatile double p = 0.0078;
    public static volatile double i = 0.055;
    public static volatile double d = 0.00039;
    public static volatile double kF = 0.0002;

    // Target velocity/position in ticks
    public static volatile double targetPos = 0;

    private enum Mode {DRIVER, AUTO}
    private Mode mode = Mode.DRIVER;

    @Override
    public void runOpMode() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new PIDFController(p, i, d, kF);

        // Send telemetry to both the Driver Station and FTC Dashboard.
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, com.acmerobotics.dashboard.FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            // Read the latest Dashboard PID values each loop.
            pid.setPIDF(p, i, d, kF);

            double curPos = turret.getCurrentPosition();

            if (gamepad1.triangle) mode = Mode.AUTO;
            if (gamepad1.circle) mode = Mode.DRIVER;

            double calculatedPower;

            if (mode == Mode.DRIVER) {
                calculatedPower = 0;
                double manualPower = gamepad1.left_stick_y;
                turret.setPower(manualPower);

                if (gamepad1.dpad_up) targetPos += 100;
                if (gamepad1.dpad_down) targetPos -= 100;
            } else {
                calculatedPower = pid.calculate(curPos, targetPos);
                calculatedPower = Math.max(-1, Math.min(1, calculatedPower));
                turret.setPower(calculatedPower);
            }

            telemetry.addLine("Press triangle to apply PID\nPress Circle for manual power");
            telemetry.addData("Mode", mode);
            telemetry.addData("Target pos (ticks/s)", targetPos);
            telemetry.addData("Current pos (ticks/s)", curPos);
            telemetry.addData("Error", targetPos - curPos); // Fixed missing operator here
            telemetry.addData("Power Output", calculatedPower);
            telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);
            telemetry.addData("kF", kF);
            telemetry.update();
        }
    }
}
