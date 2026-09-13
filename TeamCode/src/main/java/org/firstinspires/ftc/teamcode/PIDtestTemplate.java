package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "PID Test Template", group = "tuning")
@Config
public class PIDtestTemplate extends LinearOpMode {

    private DcMotorEx motor;
    private PIDFController velocityPid;
    private PIDController positionPid;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // Velocity PIDF coefficients; tune in FTC Dashboard.
    public static double velocityP = 0.001;
    public static double velocityI = 0.0;
    public static double velocityD = 0.0;
    public static double velocityF = 0.00048;

    // Position PID coefficients; tune independently in FTC Dashboard.
    public static double positionP = 0.005;
    public static double positionI = 0.0;
    public static double positionD = 0.0;

    public static double targetVelocity = 0.0; // ticks per second
    public static double targetPosition = 0.0; // encoder ticks

    private enum Mode { DRIVER, VELOCITY, POSITION }
    private Mode mode = Mode.DRIVER;

    @Override
    public void runOpMode() {
        // Rename "motor" here to match the motor's name in the Robot Configuration.
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        velocityPid = new PIDFController(velocityP, velocityI, velocityD, velocityF);
        positionPid = new PIDController(positionP, positionI, positionD);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            velocityPid.setPIDF(velocityP, velocityI, velocityD, velocityF);
            positionPid.setPID(positionP, positionI, positionD);

            if (gamepad1.circle) mode = Mode.DRIVER;
            if (gamepad1.triangle) mode = Mode.VELOCITY;
            if (gamepad1.square) mode = Mode.POSITION;

            double currentVelocity = motor.getVelocity();
            int currentPosition = motor.getCurrentPosition();
            double power;

            if (mode == Mode.DRIVER) {
                power = gamepad1.left_stick_y;
            } else if (mode == Mode.VELOCITY) {
                if (gamepad1.dpad_up) targetVelocity += 100;
                if (gamepad1.dpad_down) targetVelocity -= 100;
                power = velocityPid.calculate(currentVelocity, targetVelocity);
            } else {
                if (gamepad1.dpad_up) targetPosition += 100;
                if (gamepad1.dpad_down) targetPosition -= 100;
                power = positionPid.calculate(currentPosition, targetPosition);
            }

            power = Math.max(-1.0, Math.min(1.0, power));
            motor.setPower(power);

            telemetry.addLine("Circle: manual | Triangle: velocity PIDF | Square: position PID");
            telemetry.addData("Mode", mode);
            telemetry.addData("Current Velocity (ticks/s)", currentVelocity);
            telemetry.addData("Target Velocity (ticks/s)", targetVelocity);
            telemetry.addData("Current Position (ticks)", currentPosition);
            telemetry.addData("Target Position (ticks)", targetPosition);
            telemetry.addData("Power Output", power);
            telemetry.addData("Velocity P/I/D/F", "%f / %f / %f / %f", velocityP, velocityI, velocityD, velocityF);
            telemetry.addData("Position P/I/D", "%f / %f / %f", positionP, positionI, positionD);
            telemetry.update();
        }
    }
}
