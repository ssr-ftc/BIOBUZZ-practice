package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.turret;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "turret test with motor", group = "tuning")
@Config
public class TurretTestMotor extends LinearOpMode {

    PIDFController pid;
    private DcMotorEx turret;   // Has encoder

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    // PID coefficients (tune these in dashboard)
    public double power;
    public static double p = 0.0078; //0.002
    public static double i = 0.055; //0
    public static double d = 0.00039; //0
    public static double kF = 0.0002; // adjust for your motor's max ticks/sec

    // Target velocity in ticks per second
    public static double targetPos = 0;
    private enum Mode {DRIVER, AUTO}
    private Mode mode = Mode.DRIVER;

    @Override
    public void runOpMode() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");


        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new PIDFController(p, i, d, kF);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            pid.setPIDF(p, i, d, kF);

            double curPos = turret.getCurrentPosition();

            if (gamepad1.triangle) mode = Mode.AUTO;
            if (gamepad1.circle) mode = Mode.DRIVER;

            double power;

            if (mode == Mode.DRIVER) {
                power = 0;
                double manualPower = gamepad1.left_stick_y;
                turret.setPower(manualPower);

                if (gamepad1.dpad_up) targetPos += 100;
                if (gamepad1.dpad_down) targetPos -= 100;
            } else {
                power = pid.calculate(curPos, targetPos);
                power = Math.max(-1, Math.min(1, power));
                turret.setPower(power);
            }

            telemetry.addLine("Press triangle to apply PID\nPress Circle for manual power");
            telemetry.addData("Mode", mode);
            telemetry.addData("Target pos (ticks/s)", targetPos);
            telemetry.addData("Current pos (ticks/s)", turret.getCurrentPosition());
            telemetry.addData("Error", targetPos - turret.getCurrentPosition());
            telemetry.addData("Power Output", power);
            telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);
            telemetry.addData("kF", kF);
            telemetry.update();
        }
    }
}

//hii
