package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.DepoLM1;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0.depoLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0.intakeLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.turret;
//import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0.

@TeleOp(name = "deandreAytonsucks", group = "TeleOp")
public class soloTeleopWithManualTurret extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private DepoLM1 depo;
    private intakeLM0 intake;
    private turret turret;
    private final ElapsedTime shotTimer = new ElapsedTime();
    private boolean shotTimerRunning = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        depo = new DepoLM1(hardwareMap);
        intake = new intakeLM0(hardwareMap);
        turret = new turret(hardwareMap);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        depo.start();
        intake.start();
        turret.setPositionDeg(0);
    }

    @Override
    public void loop() {
        turret.run_using_pid();
        drive();
        useIntake();
        otherControls();
        useDepo();
        useturret();
        manualTurretPosition();
        telemetry.addData("Status", depo.isDepositOn() ? "ON" : "OFF");
        telemetry.addData("Target Velocity", depoLM0.ON_VELOCITY);
        telemetry.addData("Current Velocity", "%.1f", depo.getVelocity());
        telemetry.addLine("                             ");
        telemtry();
        telemetry.update();
    }

    public void drive() {
        double speedShift = 0.9;
        if (gamepad1.rightTriggerWasPressed()) {
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
        if (gamepad1.right_bumper) {
            intake.turn_on_intake();
        } else if (gamepad1.left_bumper) {
            intake.outtake();
        } else {
            intake.turn_off_intake();
        }
    }

    public void otherControls() {
        if (gamepad1.circleWasPressed()) {
            if (intake.isFlowerDown()) {
                intake.flower_up();
            } else {
                intake.flower_down();
            }
        }
        if (gamepad1.optionsWasPressed()) {
            turret.setPositionDeg(0);
        }
    }


    public void useturret() {
        if (gamepad1.optionsWasPressed() || gamepad1.startWasPressed()) {
            turret.setPositionDeg(0);
        }
    }

    public void manualTurretPosition(){
        /*
        if (gamepad1.rightTriggerWasPressed()) {
            turret.setPositionDeg(turret.getPositionDeg() - 10);
        } else if (gamepad1.leftTriggerWasPressed()) {
            turret.setPositionDeg(turret.getPositionDeg() + 10);

         */
         if (gamepad1.yWasPressed()) {
            turret.setPositionDeg(turret.getPositionDeg() - 5);
        } else if (gamepad1.xWasPressed()) {
            turret.setPositionDeg(turret.getPositionDeg() + 5);
        }
    }

    public void useDepo() {
        depo.run_using_pid();
        if (gamepad1.crossWasPressed()) {
            depo.turn_on_deposit();
        }
        if (depo.reachedvelocity() && !shotTimerRunning) {
            shotTimer.reset();
            shotTimerRunning = true;
            depo.open_hatch();
        }
        if (shotTimerRunning) {
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
        telemetry.addData("CURRENT SERVO POSITION", turret.getPositionDeg());
        telemetry.addLine("                 ");
        telemetry.addLine("-------------CONTROLS---------------");
        telemetry.addLine("                          ");
        telemetry.addLine("Left Joystick = forward and backward (up-down)");
        telemetry.addLine("Right Joystick = strafes (left-right)");
        telemetry.addLine("Right Trigger = Lowers speed by 70%");
        telemetry.addLine("Three Lines = resets the turret to 0");
        telemetry.addLine("A = shooter");
        telemetry.addLine("Right Bumper = intake");
        telemetry.addLine("Left Bumper = outtake");
        telemetry.addLine("B = flower");
        telemetry.addLine("Y = turret right");
        telemetry.addLine("X = turret left");

    }

    @Override
    public void stop() {
        depo.turn_off_deposit();
        depo.close_hatch();
        intake.turn_off_intake();
        intake.flower_up();
    }
}
