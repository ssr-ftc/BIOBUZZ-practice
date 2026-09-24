package org.firstinspires.ftc.teamcode.OFSWB.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "tiny depo 2 test", group = "tuning")
@Config
public class tinydepo2 extends LinearOpMode {

    PIDFController pid;
    private DcMotorEx depo;   // Has encoder
    private DcMotorEx depo1;    // Follower 1, no encoder

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    // PID coefficients (tune these in dashboard)
    public static double p = 0.001;
    public static double i = 0.0;
    public static double d = 0.00;
    // Feedforward coefficient (applied by PIDFController to target velocity)
    public static double kF = 0.000385; // adjust for your motor's max ticks/sec

    // Target velocity in ticks per second
    public static double targetVelocity = 1;
    private enum Mode {DRIVER, AUTO}
    private Mode mode = Mode.DRIVER;

    @Override
    public void runOpMode() {
        depo = hardwareMap.get(DcMotorEx.class, "");
        depo1 = hardwareMap.get(DcMotorEx.class, "");

        depo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        depo.setDirection(DcMotorSimple.Direction.FORWARD);

//        depo1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        depo1.setDirection(DcMotorSimple.Direction.REVERSE);
        depo1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        depo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new PIDFController(p, i, d, kF);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            pid.setPIDF(p, i, d, kF);

            double currentVelocity = depo1.getVelocity();

            if (gamepad1.triangle) mode = Mode.AUTO;
            if (gamepad1.circle) mode = Mode.DRIVER;

            double power;

            if (mode == Mode.DRIVER) {
                power = 0;
                double manualPower = gamepad1.left_stick_y;
                depo.setPower(manualPower);
                depo1.setPower(manualPower);

                if (gamepad1.dpad_up) targetVelocity += 100;
                if (gamepad1.dpad_down) targetVelocity -= 100;
            } else {
                power = pid.calculate(currentVelocity, targetVelocity);
                power = Math.max(-1, Math.min(1, power));
                depo.setPower(power);
                depo1.setPower(power);
            }

            telemetry.addLine("Press triangle to apply PID\nPress Circle for manual power");
            telemetry.addData("Mode", mode);
            telemetry.addData("Target Velocity (ticks/s)", targetVelocity);
            telemetry.addData("Current Velocity (ticks/s)", currentVelocity);
            telemetry.addData("Depo",depo.getVelocity());
            telemetry.addData("Depo1",depo1.getVelocity());
            telemetry.addData("Error", targetVelocity - currentVelocity);
            telemetry.addData("Power Output", power);
            telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);
            telemetry.addData("kF", kF);
            telemetry.update();
        }
    }
}
