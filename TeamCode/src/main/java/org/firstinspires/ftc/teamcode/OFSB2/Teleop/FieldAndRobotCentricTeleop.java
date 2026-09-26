package org.firstinspires.ftc.teamcode.OFSB2.Teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.OFSB2.Auto.Constants;

@TeleOp(name="FieldAndRobotCentricTeleop", group="Iterative Opmode")
public class FieldAndRobotCentricTeleop extends OpMode {

    DcMotorEx fl, fr, rl, rr;
    DcMotorEx intake;
    private GoBildaPinpointDriver driver;
    private Follower follower;

    final double MAX_DRIVE_VELOCITY = 2787.625;

    @Override
    public void init() {
        hardwareMap();
        if (driver != null) {
            driver.resetPosAndIMU();
        }
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void loop() {
        drive();
        handleIntake();
    }

    public void hardwareMap() {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        rl = hardwareMap.get(DcMotorEx.class, "rl");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        try {
            driver = hardwareMap.get(GoBildaPinpointDriver.class, "imu");
        } catch (Exception ignored) {}

        fl.setDirection(DcMotor.Direction.REVERSE);
        rl.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        try {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
        } catch (Exception ignored) {}
    }

    public void drive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if (follower != null) {
            follower.setTeleOpDrive(forward, strafe, turn, true);
            follower.update();
        }
    }

    public void handleIntake() {
        if (intake != null) {
            if (gamepad1.right_bumper) {
                intake.setVelocity(ticksPerSecondCalculator(6000, 28));
            } else {
                intake.setVelocity(0);
            }
        }
    }

    public double ticksPerSecondCalculator(double targetMax, double encoderResolution) {
        return targetMax / 60 * encoderResolution;
    }
}
