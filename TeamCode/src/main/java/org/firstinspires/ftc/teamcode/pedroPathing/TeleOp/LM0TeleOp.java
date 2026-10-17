package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.depoLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.intakeLM0;

@TeleOp(name = "LM0 TeleOp", group = "LM0")
@Config
public class LM0TeleOp extends OpMode {

    private Follower follower;

    private intakeLM0 intake;
    private depoLM0 depo;
    private double speedScale = 1;
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
        depo.start();
    }

    @Override
    public void loop() {
        follower.update();
        driveWithPedro();
        runIntake();
        runDepo();
        //  openhatch();
        sendTelemetry();

        if(gamepad1.squareWasPressed()) depo.updatePID();
    }

    @Override
    public void stop() {
        intake.turn_off_intake();
        depo.turn_off_deposit();
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

//        if (gamepad1.dpad_up) {
//            intake.flower_up();
//        } else if (gamepad1.dpad_down) {
//            intake.flower_down();
//        }
    }

    private void openhatch(){
        if (gamepad1.circleWasPressed()) {
            if (depo.ishatchopen()){
                depo.hatch_close();
            }
            else {
                depo.hatch_open();
            }
        }
    }
    private void runDepo() {
        depo.run_using_pid();
        if (gamepad1.aWasPressed()) {
            if (depo.isDepositOn()) {
                depo.turn_off_deposit();
                //depo.hatch_close();
            } else {
                depo.turn_on_deposit();
            }
        }

    }

    private void sendTelemetry() {
        telemetry.addData("Intake", intake.isIntakeOn() ? "ON" : "OFF");
        telemetry.addData("Intake power", intake.getMotorPower());
        telemetry.addData("Deposit", depo.isDepositOn() ? "ON" : "OFF");
        telemetry.addData("Target velocity", depoLM0.targetVelocity);
        telemetry.addData("Bottom velocity", depo.getVelocity());
        telemetry.addData("Top velocity", depo.getTopVelocity());
        telemetry.addData("Error", depoLM0.targetVelocity - depo.getVelocity());
        telemetry.addData("Deposit power", depo.power);
        telemetry.update();
    }
}
//hi