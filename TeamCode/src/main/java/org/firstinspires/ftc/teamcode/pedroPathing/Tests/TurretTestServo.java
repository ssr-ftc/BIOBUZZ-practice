package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Tune the turret from the Driver Station only.
 *
 * Triangle / Y = AUTO (PID to target)
 * Circle / B   = DRIVER (manual left stick)
 *
 * Dpad up/down     = targetPos +/- 500 ticks
 * Dpad left/right  = select which gain to edit (P, I, D, kF)
 * Bumpers          = gain step size
 * Triggers         = nudge selected gain
 * Options/start    = reset encoder and target to 0
 *
 * p, i, d, kF are public static, so they can also be edited live in FTC Dashboard (Config tab).
 */
@TeleOp(name = "turret test servo cr mode", group = "tuning")
@Config
public class TurretTestServo extends LinearOpMode {

    private static final double[] GAIN_STEPS = {0.0001, 0.001, 0.01, 0.1};
    private static final String[] GAIN_NAMES = {"P", "I", "D", "kF"};

    private DcMotorEx encoder;
    CRServo turret;
    private PIDFController pid;

    public static double p = 0.0008;
    public static double i = 0;
    public static double d = 0.00004;
    public static double kF = 0;
    private double targetPos = 0;

    private enum Mode {DRIVER, AUTO}
    private Mode mode = Mode.DRIVER;

    private int gainIndex = 0;
    private int stepIndex = 1;

    @Override
    public void runOpMode() {
        encoder = hardwareMap.get(DcMotorEx.class, "intake2");
        turret = hardwareMap.get(CRServo.class, "turret");

        pid = new PIDFController(p, i, d, kF);

        // Telemetry goes to both the DS and the Dashboard graph.
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
                targetPos += 500;
            }
            if (gamepad1.dpadDownWasPressed()) {
                targetPos -= 500;
            }

            if (gamepad1.dpadLeftWasPressed()) {
                gainIndex = (gainIndex + GAIN_NAMES.length - 1) % GAIN_NAMES.length;
            }
            if (gamepad1.dpadRightWasPressed()) {
                gainIndex = (gainIndex + 1) % GAIN_NAMES.length;
            }

            if (gamepad1.rightBumperWasPressed()) {
                stepIndex = (stepIndex + GAIN_STEPS.length - 1) % GAIN_STEPS.length;
            }
            if (gamepad1.leftBumperWasPressed()) {
                stepIndex = (stepIndex + 1) % GAIN_STEPS.length;
            }

            double step = GAIN_STEPS[stepIndex]/100;
            if (gamepad1.leftTriggerWasPressed()) {
                adjustGain(-step);
            } else if (gamepad1.rightTriggerWasPressed()) {
                adjustGain(step);
            }

            if (gamepad1.optionsWasPressed() || gamepad1.startWasPressed()) {
                encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                targetPos = 0;
                pid.reset();
            }

            pid.setPIDF(p, i, d, kF);

            double currentPos = encoder.getCurrentPosition();
            double power;

            if (mode == Mode.DRIVER) {
                power = gamepad1.left_stick_y;
                turret.setPower(power);
            } else {
                power = pid.calculate(currentPos, targetPos);
                power = Math.max(-1.0, Math.min(1.0, power));
                turret.setPower(-power);
            }

            telemetry.addData("Mode", mode);
            telemetry.addData("targetPos", targetPos);
            telemetry.addData("currentPos", currentPos);
            telemetry.addData("error", targetPos - currentPos);
            telemetry.addData("power", power);

            telemetry.addData("P","%.7f", p);
            telemetry.addData("I","%.7f", i);
            telemetry.addData("D","%.7f", d);
            telemetry.addData("kF","%.7f", kF);
            telemetry.addData("Editing", GAIN_NAMES[gainIndex]);
            telemetry.addData("Gain step","%.7f", step);
            telemetry.addLine("Triggers = nudge gain, bumpers = step size");
            telemetry.update();
        }

        encoder.setPower(0);
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