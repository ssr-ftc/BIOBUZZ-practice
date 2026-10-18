package org.firstinspires.ftc.teamcode.pedroPathing.templates.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.depoLM0;

@TeleOp(name = "drivePractice", group = "TeleOp")
public class drivePractice extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private DcMotorEx intake;
    private depoLM0 depo;
    private carDashboard dashboard;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        follower.setStartingPose(startPose);
        depo = new depoLM0(hardwareMap);
        dashboard = new carDashboard(hardwareMap, telemetry, follower);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        depo.start();
        telemetry.addLine("--------CONTROLS--------");
        telemetry.addLine("                          ");
        telemetry.addLine("Base Driver:");
        telemetry.addLine("Left Joystick = forward and backward (up-down)");
        telemetry.addLine("Left Joystick = strafs (left-right");
        telemetry.addLine("Right Trigger = Lowers speed by 70%");
        telemetry.addLine("Three Lines = resets theCompass");
        telemetry.addLine("                          ");
        telemetry.addLine("Attachments:");
        telemetry.addLine("Right Bumper = intake");
        telemetry.addLine("Left Bumper = outtake");
        telemetry.addLine("Right Trigger = shooter");
        telemetry.addLine("DPad Up/Down = changes veclotiy");
        dashboard.start();
    }

    @Override
    public void loop() {
        drive();
        useIntake();
        useDepo();
        dashboard.update();
    }

    public void drive() {
        double speedShift = 0.9;
        if (gamepad1.right_trigger > 0.1) {
            speedShift = 0.25;
            gamepad1.rumbleBlips(1);
        }

        if (gamepad1.options) {
            follower.setPose(new Pose(0, 0, Math.toRadians(0)));
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speedShift,
                -gamepad1.left_stick_x * speedShift,
                -gamepad1.right_stick_x * speedShift * 0.8,
                true
        );

        follower.update();
    }

    public void useIntake() {
        if (gamepad2.right_bumper) {
            intake.setPower(1.00);
        } else if (gamepad2.left_bumper) {
            intake.setPower(-1.00);
        } else {
            intake.setPower(0);
        }
    }

    public void useDepo() {
        depo.run_using_pid();
        if (gamepad2.right_trigger > 0.3) {
            if (depo.isDepositOn()) {
                depo.turn_off_deposit();
                //depo.hatch_close();
            } else {
                depo.turn_on_deposit();
            }
        }
        if (gamepad2.dpad_up) {
            depoLM0.ON_VELOCITY += 10;
        } else if (gamepad2.dpad_down) {
            depoLM0.ON_VELOCITY -= 10;
        }
    }

    @Override
    public void stop() {
        dashboard.stop();
    }
}
