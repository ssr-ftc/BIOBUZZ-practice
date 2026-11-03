package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Tune the turret from the Driver Station only.
 *
 * Triangle / Y = AUTO (PID to target)
 * Circle / B   = DRIVER (manual left stick)
 *
 * Dpad up/down     = targetPos +/- 50 ticks
 * Dpad left/right  = select which gain to edit (P, I, D, kF)
 * Bumpers          = gain step size
 * Triggers         = nudge selected gain
 * Options/start    = reset encoder and target to 0
 */
@TeleOp(name = "turret test with motor", group = "tuning")
public class TurretTestMotor extends LinearOpMode {

    private static final double[] GAIN_STEPS = {0.0001, 0.001, 0.01, 0.1};
    private static final String[] GAIN_NAMES = {"P", "I", "D", "kF"};

    private DcMotorEx turret;
    private PIDFController pid;

    private double p = 0.00;
    private double i = 0.00;
    private double d = 0.000;
    private double kF = 0;
    private double targetPos = 0;

    private enum Mode {DRIVER, AUTO}
    private Mode mode = Mode.DRIVER;

    private int gainIndex = 0;
    private int stepIndex = 1;

    @Override
    public void runOpMode() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid = new PIDFController(p, i, d, kF);

        // DS controls the values; Dashboard only graphs telemetry.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Turret DS tuner ready");
        telemetry.addLine("Triangle=AUTO  Circle=DRIVER");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.triangleWasPressed() || gamepad1.yWasPressed()) {
                mode = Mode.AUTO;
            }
            if (gamepad1.circleWasPressed() || gamepad1.bWasPressed()) {
                mode = Mode.DRIVER;
            }

            if (gamepad1.dpadUpWasPressed()) {
                targetPos += 50;
            }
            if (gamepad1.dpadDownWasPressed()) {
                targetPos -= 50;
            }

            if (gamepad1.dpadLeftWasPressed()) {
                gainIndex = (gainIndex + GAIN_NAMES.length - 1) % GAIN_NAMES.length;
            }
            if (gamepad1.dpadRightWasPressed()) {
                gainIndex = (gainIndex + 1) % GAIN_NAMES.length;
            }

            if (gamepad1.leftBumperWasPressed()) {
                stepIndex = (stepIndex + GAIN_STEPS.length - 1) % GAIN_STEPS.length;
            }
            if (gamepad1.rightBumperWasPressed()) {
                stepIndex = (stepIndex + 1) % GAIN_STEPS.length;
            }

            double step = GAIN_STEPS[stepIndex];
            if (gamepad1.leftTriggerWasPressed()) {
                adjustGain(-step);
            } else if (gamepad1.rightTriggerWasPressed()) {
                adjustGain(step);
            }

            if (gamepad1.optionsWasPressed() || gamepad1.startWasPressed()) {
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                targetPos = 0;
                pid.reset();
            }

            pid.setPIDF(p, i, d, kF);

            double currentPos = turret.getCurrentPosition();
            double power;

            if (mode == Mode.DRIVER) {
                power = gamepad1.left_stick_y;
                turret.setPower(power);
            } else {
                power = pid.calculate(currentPos, targetPos);
                power = Math.max(-1.0, Math.min(1.0, power));
                turret.setPower(power);
            }

            telemetry.addData("Mode", mode);
            telemetry.addData("targetPos", targetPos);
            telemetry.addData("currentPos", currentPos);
            telemetry.addData("error", targetPos - currentPos);
            telemetry.addData("power", power);
            telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);
            telemetry.addData("kF", kF);
            telemetry.addData("Editing", GAIN_NAMES[gainIndex]);
            telemetry.addData("Gain step", step);
            telemetry.addLine("Triggers = nudge gain, bumpers = step size");
            telemetry.update();
        }

        turret.setPower(0);
    }

    private void adjustGain(double delta) {
        switch (gainIndex) {
            case 0:
                p = Math.max(0, p + delta);
                break;
            case 1:
                i = Math.max(0, i + delta);
                break;
            case 2:
                d = Math.max(0, d + delta);
                break;
            case 3:
                kF = Math.max(0, kF + delta);
                break;
            default:
                break;
        }
    }
}
