package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Tune depo velocity PID values from the Driver Station.
 *
 * Triangle / Y = AUTO (PID to target velocity)
 * Circle / B   = DRIVER (manual left stick)
 *
 * Dpad up/down     = target velocity +/- 100 ticks/second
 * Dpad left/right  = select which gain to edit (P, I, D, kF)
 * Bumpers          = gain step size
 * Triggers         = nudge selected gain
 */
@TeleOp(name = "depo pid test", group = "tuning")
@Config
public class DepoTest extends LinearOpMode {

    private static final double[] GAIN_STEPS = {0.0001, 0.001, 0.01, 0.1};
    private static final String[] GAIN_NAMES = {"P", "I", "D", "kF"};

    private DcMotorEx depo;
    private DcMotorEx depo1;
    private PIDFController pid;

    public static double p = 0.004;
    public static double i = 0;
    public static double d = 0;
    public static double kF = 0.0005;
    public static double targetVelocity = 0;

    private enum Mode {DRIVER, AUTO}
    private Mode mode = Mode.DRIVER;

    private int gainIndex = 0;
    private int stepIndex = 1;

    @Override
    public void runOpMode() {
        depo = hardwareMap.get(DcMotorEx.class, "depo");
        depo1 = hardwareMap.get(DcMotorEx.class, "depo1");

        depo.setDirection(DcMotorSimple.Direction.FORWARD);
        depo1.setDirection(DcMotorSimple.Direction.FORWARD);
        depo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        depo1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        depo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        depo1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pid = new PIDFController(p, i, d, kF);

        // Driver Station controls values; Dashboard only graphs telemetry.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Depo Driver Station tuner ready");
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
                targetVelocity += 100;
            }
            if (gamepad1.dpadDownWasPressed()) {
                targetVelocity -= 100;
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

            pid.setPIDF(p, i, d, kF);

            double currentVelocity = depo.getVelocity();
            double power;
            if (mode == Mode.DRIVER) {
                power = gamepad1.left_stick_y;
            } else {
                power = pid.calculate(currentVelocity, targetVelocity);
                power = Math.max(-1.0, Math.min(1.0, power));
            }
            depo.setPower(-power);
            depo1.setPower(-power);

            telemetry.addData("Mode", mode);
            telemetry.addData("targetVelocity", targetVelocity);
            telemetry.addData("currentVelocity", currentVelocity);
            telemetry.addData("depo", depo.getVelocity());
            telemetry.addData("depo1", depo1.getVelocity());
            telemetry.addData("error", targetVelocity - currentVelocity);
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

        depo.setPower(0);
        depo1.setPower(0);
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
