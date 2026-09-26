package org.firstinspires.ftc.teamcode.OFSB2.Teleop;

import org.firstinspires.ftc.teamcode.OFSB2.Subsystems.CustomFollower;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="TeleopIntakeTest", group="Iterative Opmode")
public class FieldAndRobotCentricTeleop extends OpMode {

    DcMotorEx fl, fr, rl, rr;
    DcMotorEx intake;
    private GoBildaPinpointDriver driver;
    private CustomFollower follower;

    final double MAX_DRIVE_VELOCITY = 2787.625;

    @Override
    public void init() {
        hardwareMap();
        driver.resetPosAndIMU();
        follower = new CustomFollower(hardwareMap, telemetry);
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

        driver = hardwareMap.get(GoBildaPinpointDriver.class, "imu");

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

        intake = hardwareMap.get(DcMotorEx.class, "intake");
    }

    public void drive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double heading = driver.getHeading(AngleUnit.RADIANS);

        if ((heading > Math.toRadians(-45) && heading < Math.toRadians(45)) ||
                (heading < Math.toRadians(-135) || heading > Math.toRadians(135))) {
            follower.pedro.manual(ManualDrive.fieldCentric(forward, strafe, turn, heading));
        } else {
            follower.pedro.manual(forward, strafe, turn);
        }

        follower.pedro.update();

        fl.setVelocity(fl.getPower() * MAX_DRIVE_VELOCITY);
        fr.setVelocity(fr.getPower() * MAX_DRIVE_VELOCITY);
        rl.setVelocity(rl.getPower() * MAX_DRIVE_VELOCITY);
        rr.setVelocity(rr.getPower() * MAX_DRIVE_VELOCITY);
    }

    public void handleIntake() {
        if (gamepad1.right_bumper) {
            intake.setVelocity(ticksPerSecondCalculator(6000, 28));
        } else {
            intake.setVelocity(0);
        }
    }

    public double ticksPerSecondCalculator(double targetMax, double encoderResolution) {
        return targetMax / 60 * encoderResolution;
    }
}
