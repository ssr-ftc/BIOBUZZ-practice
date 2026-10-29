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

@TeleOp(name = "LM1teleop", group = "TeleOp")
public class LM1TeleOp extends OpMode {
    private Follower follower;
    Pose hiveTop = new Pose(84.3,81.4,0);
    Pose hiveBottom = new Pose(84.3,56,0);
    private final Pose startPose = new Pose(0, 0, 0);
    private DepoLM1 depo;
    private intakeLM0 intake;
    private turret turret;
    private final ElapsedTime shotTimer = new ElapsedTime();
    private boolean shotTimerRunning = false;
    private boolean isTracking = false;

    ///pretty basic stuff just defines things

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        depo = new DepoLM1(hardwareMap);
        intake = new intakeLM0(hardwareMap);
        turret = new turret(hardwareMap);
    }
    ///what happens on init

    @Override
    public void start() {
        follower.startTeleopDrive();
        depo.start();
        intake.start();
        turret.setPositionDeg(0);
    }
    ///what happens on start and sets turret to facing forward

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
        if (gamepad1.right_trigger > 0.1) {
            speedShift = 0.25;
            gamepad1.rumbleBlips(1);
        }
        if (gamepad1.options) {
            follower.setPose(new Pose(86.5, 133.3, Math.toRadians(270)));
        }
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speedShift,
                -gamepad1.left_stick_x * speedShift,
                -gamepad1.right_stick_x * speedShift * 0.8,
                true
        );
        follower.update();
    }
    ///uses pedro follower to make driving stuff easier, also slows down after right trigger and rumbles
    public void useIntake() {
        if (gamepad2.right_bumper) {
            intake.turn_on_intake();
        } else if (gamepad2.left_bumper) {
            intake.outtake();
        } else {
            intake.turn_off_intake();
        }
    }
    ///hold right trigger to intake because I like that better

    public void otherControls() {
        if (gamepad1.triangleWasPressed()) {
            if (intake.isFlowerDown()) {
                intake.flower_up();
            } else {
                intake.flower_down();
            }
        }
        if (gamepad2.optionsWasPressed()) {
            turret.setPositionDeg(0);
        }
    }
    ///flower and options reset button


    public void useturret() {
        if (gamepad2.optionsWasPressed() || gamepad2.startWasPressed()) {
            turret.setPositionDeg(0);
        }
        if(gamepad1.leftBumperWasPressed()){
            isTracking = !isTracking;
        }
        if(isTracking){
            double opp,adj;//relative to pose
            opp=hiveTop.getX()-follower.getPose().getX();
            adj=hiveTop.getY()-follower.getPose().getY();
            double angle= Math.atan(opp/adj);
            double turAngle;
            turAngle = 180+angle-Math.toDegrees(follower.getPose().getHeading());
            turret.setPositionDeg(turAngle);
        }
    }
    ///no cliue why this is here

    public void manualTurretPosition(){
        /*
        if (gamepad2.rightTriggerWasPressed()) {
            turret.setPositionDeg(turret.getPositionDeg() - 10);
        } else if (gamepad2.leftTriggerWasPressed()) {
            turret.setPositionDeg(turret.getPositionDeg() + 10);

         */
        if (gamepad2.yWasPressed()) {
            turret.setPositionDeg(turret.getPositionDeg() - 5);
        } else if (gamepad2.xWasPressed()) {
            turret.setPositionDeg(turret.getPositionDeg() + 5);
        }
    }
    ///manual turret position change

    public void useDepo() {
        depo.run_using_pid();
        if (gamepad2.crossWasPressed()) {
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
    ///launches depo. Basically depo turns on and when it hits target velocity, it runs intake and opens hatch for
    /// a certain period of time and then after timer hits 2 seconds it turns everything off.

    public void telemtry() {
        telemetry.addData("CURRENT SERVO POSITION", turret.getPositionDeg());
        telemetry.addLine("                 ");
        telemetry.addLine("-------------CONTROLS---------------");
        telemetry.addLine("                          ");
        telemetry.addLine("Base driver:");
        telemetry.addLine("Left Joystick = forward and backward (up-down)");
        telemetry.addLine("Right Joystick = strafes (left-right)");
        telemetry.addLine("Right Trigger = Lowers speed by 70%");
        telemetry.addLine("Y = Flower ");
        telemetry.addLine("                          ");
        telemetry.addLine("Shooter driver:");
        telemetry.addLine("A = shooter");
        telemetry.addLine("Right Bumper = intake");
        telemetry.addLine("Left Bumper = outtake");
        telemetry.addLine("Y = turret right");
        telemetry.addLine("X = turret left");
        telemetry.addLine("Start/options = reset turret 0");
    }
    ///controls

    @Override
    public void stop() {
        depo.turn_off_deposit();
        depo.close_hatch();
        intake.turn_off_intake();
        intake.flower_up();
    }
}
