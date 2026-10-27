package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.DepoLM1;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0.intakeLM0;

@TeleOp(name = "LM1teleop", group = "TeleOp")
public class LM1TeleOp extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private DepoLM1 depo;
    private intakeLM0 intake;
    private final ElapsedTime shotTimer = new ElapsedTime();
    private boolean shotTimerRunning = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        depo = new DepoLM1(hardwareMap);
        intake = new intakeLM0(hardwareMap);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        depo.start();
        intake.start();
    }

    @Override
    public void loop() {
        drive();
        useIntake();
        useDepo();
        telemetry.addData("Status", depo.isDepositOn() ? "ON" : "OFF");
        telemetry.addData("Target Velocity", DepoLM1.ON_VELOCITY);
        telemetry.addData("Current Velocity", "%.1f", depo.getVelocity());
        telemetry.addLine("                             ");
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
        if (gamepad2.leftBumperWasPressed()) {
            if (intake.isOuttaking()) {
                intake.turn_off_intake();
            } else {
                intake.outtake();
            }
        }
        if (gamepad2.rightBumperWasPressed()) {
            if (intake.getMotorPower() < 0) {
                intake.turn_off_intake();
            } else {
                intake.turn_on_intake();
            }
        }
        if (gamepad1.triangleWasPressed()) {
            if (intake.isFlowerDown()) {
                intake.flower_up();
            } else {
                intake.flower_down();
            }
        }
    }

    public void useDepo() {
        depo.run_using_pid();
        if (gamepad2.crossWasPressed()) {
            depo.turn_on_deposit();
        }
        if (depo.reachedvelocity() && !shotTimerRunning) {
            shotTimer.reset();
            shotTimerRunning = true;
            depo.open_hatch();
            intake.turn_on_intake();
        }
        if (shotTimerRunning && shotTimer.seconds() >= 2) {
            shotTimerRunning = false;
            depo.turn_off_deposit();
            depo.close_hatch();
            intake.turn_off_intake();
        }
    }

    public void telemtry() {
        telemetry.addLine("                 ");
        telemetry.addLine("-------------CONTROLS---------------");
        telemetry.addLine("                          ");
        telemetry.addLine("Base driver:");
        telemetry.addLine("Left Joystick = forward and backward (up-down)");
        telemetry.addLine("Left Joystick = strafes (left-right)");
        telemetry.addLine("Right Trigger = Lowers speed by 70%");
        telemetry.addLine("Three Lines = resets the Compass");
        telemetry.addLine("                          ");
        telemetry.addLine("Shooter driver:");
        telemetry.addLine("Cross = shooter");
        telemetry.addLine("Right Bumper = intake");
        telemetry.addLine("Left Bumper = outtake");
        telemetry.addLine("Triangle = flower");
    }

    @Override
    public void stop() {
        depo.turn_off_deposit();
        depo.close_hatch();
        intake.turn_off_intake();
        intake.flower_up();
    }
}
