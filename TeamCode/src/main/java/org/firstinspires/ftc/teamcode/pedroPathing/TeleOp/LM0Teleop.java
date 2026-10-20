package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.depoLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.intakeLM0;

@TeleOp(name = "realLM0Teleop", group = "TeleOp")
public class LM0Teleop extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private intakeLM0 intake;
    private depoLM0 depo;
    private carDashboard dashboard;
    private Timer timers;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        intake = new intakeLM0(hardwareMap);
        follower.setStartingPose(startPose);
        depo = new depoLM0(hardwareMap);
        dashboard = new carDashboard(hardwareMap, telemetry, follower);
        timers = new Timer();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        intake.start();
        depo.start();
        dashboard.start();
        timers.createNew("depo");
    }

    @Override
    public void loop() {
        drive();
        useIntake();
        useDepo();
        telemetry.addData("Status", depo.isDepositOn() ? "ON" : "OFF");
        telemetry.addData("Target Velocity", depoLM0.ON_VELOCITY);
        telemetry.addData("Current Velocity", "%.1f", depo.getVelocity());
        telemetry.addLine("                             ");
        dashboard.update();
        telemtry();
        telemetry.update();
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
        if (gamepad2.circleWasPressed()) {
            intake.outtake();
        } else if (intake.isOuttaking()) {
            intake.turn_off_intake();
        }
        if (gamepad2.rightBumperWasPressed()) {
            if (intake.isIntakeOn()) {
                intake.turn_off_intake();
            } else {
                intake.turn_on_intake();
            }
        }
        if(gamepad2.leftBumperWasPressed()){
            if(intake.isFlowerDown()){
                intake.flower_up();
            }
            else intake.flower_down();
        }
    }

    public void useDepo() {
        depo.run_using_pid();
        if (gamepad2.crossWasPressed()) {
            depo.turn_on_deposit();
        }
        if (depo.reachedvelocity() && !timers.isRunning("depo")) {
            timers.start("depo");
            depo.open_hatch();
            intake.turn_on_intake();
        }
        if (timers.checkSecondsLast("depo", 2)) {
            depo.turn_off_deposit();
            depo.close_hatch();
        }


        if (gamepad2.dpad_up) {
            depoLM0.ON_VELOCITY += 10;
        } else if (gamepad2.dpad_down) {
            depoLM0.ON_VELOCITY -= 10;
        }
    }

    public void telemtry() {
        telemetry.addLine( "                 ");
        telemetry.addLine("-------------CONTROLS---------------");
        telemetry.addLine("                          ");
        telemetry.addLine("Base driver:");
        telemetry.addLine("Left Joystick = forward and backward (up-down)");
        telemetry.addLine("Left Joystick = strafs (left-right)");
        telemetry.addLine("Right Trigger = Lowers speed by 70%");
        telemetry.addLine("Three Lines = resets theCompass");
        telemetry.addLine("                          ");
        telemetry.addLine("Shooter driver:");
        telemetry.addLine("Right Bumper = intake");
        telemetry.addLine("Left Bumper = outtake");
        telemetry.addLine("A = shooter");
        telemetry.addLine("DPad Up/Down = changes veclotiy");

    }

    @Override
    public void stop() {
        intake.turn_off_intake();
        depo.turn_off_deposit();
        dashboard.stop();
    }
}
