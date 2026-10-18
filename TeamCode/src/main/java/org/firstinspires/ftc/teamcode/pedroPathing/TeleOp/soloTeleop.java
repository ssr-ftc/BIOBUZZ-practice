package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.depoLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.intakeLM0;

@TeleOp(name = "soloControllerTeleop", group = "LM0")
//@Config
public class soloTeleop extends OpMode {

    private Follower follower;

    private intakeLM0 intake;
    private depoLM0 depo;
    private Timer timers;
    private double speedScale = 1;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        timers = new Timer();
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
        timers.createNew("depo");
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
    }

    private void driveWithPedro() {
        double forward = -gamepad1.left_stick_y * speedScale;
        double strafe = -gamepad1.left_stick_x * speedScale;
        double turn = -gamepad1.right_stick_x * 0.8;
        follower.setTeleOpDrive(forward, strafe, turn, true);
    }

    private void runIntake() {
        if(gamepad1.left_bumper){
            intake.outtake();
        }
        else if(intake.isOuttaking()){
            intake.turn_off_intake();
        }
        if (gamepad1.rightBumperWasPressed()){
            if (intake.isIntakeOn()){
                intake.turn_off_intake();
            } else {
                intake.turn_on_intake();
            }
        }
        if(gamepad1.squareWasPressed()){
            if(intake.isFlowerDown()){
                intake.flower_up();
            }
            else intake.flower_down();
        }


    }

    private void runDepo() {
        depo.run_using_pid();
        if (gamepad1.crossWasPressed()){
            depo.turn_on_deposit();
        }
        if (depo.reachedvelocity() && !timers.isRunning("depo")){
            timers.start("depo");
            depo.open_hatch();
            intake.turn_on_intake();
        }
        if (timers.checkSecondsLast("depo", 3)){
            depo.turn_off_deposit();
            depo.close_hatch();

        }




        if (gamepad1.dpad_up) {
            depoLM0.ON_VELOCITY +=1;
        } else if (gamepad1.dpad_down) {
            depoLM0.ON_VELOCITY -=1;
        }

    }

    private void sendTelemetry() {
        telemetry.addData("Intake", intake.isIntakeOn() ? "ON" : "OFF");
        telemetry.addData("Intake power", intake.getMotorPower());
        telemetry.addData("Deposit", depo.isDepositOn() ? "ON" : "OFF");
        telemetry.addData("Target velocity", depoLM0.ON_VELOCITY);
        telemetry.addData("Current velocity", depo.getVelocity());
        telemetry.addData("Deposit power", depo.power);
        telemetry.update();
    }
}
//hi