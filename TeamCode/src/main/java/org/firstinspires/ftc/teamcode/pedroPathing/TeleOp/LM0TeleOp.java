package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.depoLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.intakeLM0;

@TeleOp(name = "LM0 TeleOp", group = "LM0")
public class LM0TeleOp extends OpMode {

    private Follower follower;
    private intakeLM0 intake;
    private depoLM0 depo;
    private double speedScale = 0.8;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        intake = new intakeLM0(hardwareMap);
        depo = new depoLM0(hardwareMap);

        telemetry.addLine("LM0 TeleOp initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        intake.start();
        depo.turn_off_deposit();
    }

    @Override
    public void loop() {
        follower.update();
        driveWithPedro();
        runIntake();
        runDepo();
        sendTelemetry();
    }

    @Override
    public void stop() {
        intake.turn_off_intake();
        depo.turn_off_deposit();
        depo.run_using_pid();
    }

    private void driveWithPedro() {
        double forward = -gamepad1.left_stick_y * speedScale;
        double strafe = -gamepad1.left_stick_x * speedScale;
        double turn = -gamepad1.right_stick_x * speedScale;
        follower.setTeleOpDrive(forward, strafe, turn, true);
    }

    private void runIntake() {
        if (gamepad1.left_bumper) {
            intake.outtake();
        } else if (gamepad1.right_bumper) {
            intake.turn_on_intake();
        } else {
            intake.turn_off_intake();
        }

        if (gamepad1.dpad_up) {
            intake.flower_up();
        } else if (gamepad1.dpad_down) {
            intake.flower_down();
        }
    }

    private void runDepo() {
        if (gamepad1.aWasPressed()) {
            if (depo.isDepositOn()) {
                depo.turn_off_deposit();
            } else {
                depo.turn_on_deposit();
            }
        }
        depo.run_using_pid();
    }

    private void sendTelemetry() {
        telemetry.addData("Intake", intake.isIntakeOn() ? "ON" : "OFF");
        telemetry.addData("Intake power", intake.getMotorPower());
        telemetry.addData("Deposit", depo.isDepositOn() ? "ON" : "OFF");
        telemetry.addData("Deposit power", depo.power);
        telemetry.update();
    }
}
