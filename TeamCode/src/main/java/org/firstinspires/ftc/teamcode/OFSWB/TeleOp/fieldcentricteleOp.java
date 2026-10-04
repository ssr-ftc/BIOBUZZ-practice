package org.firstinspires.ftc.teamcode.OFSWB.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.intake;
import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.depo;
import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.colorsensors;
import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.lifters;
import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.turret;
import org.firstinspires.ftc.teamcode.Timer;

import org.firstinspires.ftc.teamcode.OFSWB.Constants;

@TeleOp(name = "Field Centric TeleOp", group = "tests")
public class fieldcentricteleOp extends OpMode {

    private Follower follower;

    private intake intake;
    private depo depo;
    private lifters lifters;
    private colorsensors colorsensors;
    private turret turret;
    private double speedScale = 0;
    private double xvelocity = -1800;

    private ElapsedTime shotTimer = new ElapsedTime();
    Timer timer;
    private int shotStep = 0;
    private boolean shooting = false;

    private static final double warmup_seconds = 0.75;
    private static final double up_hold_seconds = 0.4;
    private static final double down_wait_seconds = 0.6;
    int ballCount;

    @Override
    public void init() {
        // Initialize Follower via Constants.createFollower()
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        intake = new intake(hardwareMap);
        depo = new depo(hardwareMap);
        lifters = new lifters(hardwareMap);
        colorsensors = new colorsensors(hardwareMap);
        turret = new turret(hardwareMap);
        timer = new Timer();
        timer.createNew("intake");
        timer.createNew("depo");
    }

    @Override
    public void start() {
        follower.startTeleopDrive(); // Required for PedroPathing TeleOp
        lifters.allDown();
        turret.setservotodegree(0);
    }

    @Override
    public void loop() {
        driveMecanum();

        depo.run_using_pid();

        shootingfunction();
        depoonoff();
        changeVelo();
        intakingstuff();
        turretstuff();
        telemetrystuff();
    }

    private void telemetrystuff(){
        telemetry.addData("intake on", intake.isIntakeOn());
        telemetry.addData("depo on", depo.isDepositOn());
        telemetry.addData("xvelocity", xvelocity);
        telemetry.addData("ball count", ballCount);
    }

    public void shootingfunction() {
        if(gamepad2.crossWasPressed()){ // start shooting
            depo.set_target_velocity(xvelocity);
            timer.start("depo");
        }
        if (timer.checkSeconds("depo", 0.5)) {
            lifters.rightUp();
        }
        if (timer.checkSeconds("depo", 1.0)) {
            lifters.rightDown();
        }
        if (timer.checkSeconds("depo", 1.5)) {
            lifters.backUp();
        }
        if (timer.checkSeconds("depo", 2.0)) {
            lifters.backDown();
        }
        if (timer.checkSeconds("depo", 2.5)) {
            lifters.leftUp();
        }
        if (timer.checkSeconds("depo", 3.0)) {
            lifters.leftDown();
        }
        if (timer.checkSecondsLast("depo", 3.5)) {
            lifters.allDown();
            depo.turn_off_deposit();
        }
    }

    public void depoonoff(){
        if (gamepad2.triangleWasPressed() ) {
            if (depo.isDepositOn()){
                depo.turn_off_deposit();
            }
            else{
                depo.turn_on_deposit();
            }
        }
    }

    private void changeVelo(){
        if (gamepad2.dpadUpWasPressed()){
            xvelocity -= 100;
        }
        if (gamepad2.dpadDownWasPressed()){
            xvelocity += 100;
        }
    }

    public void intakingstuff(){
        ballCount = colorsensors.countBalls();
        if (intake.isIntakeOn() && ballCount >= 3) {
            intake.turn_off_intake();
        }

        if (gamepad2.rightBumperWasPressed() && !shooting) {
            if (intake.isIntakeOn()) {
                intake.turn_off_intake();
            } else if (ballCount < 3) {
                intake.turn_on_intake();
            }
        }
    }

    public void turretstuff() {
        if (gamepad2.dpad_right) {
            turret.move1degright();
        }
        if (gamepad2.dpad_left){
            turret.move1degleft();
        }
    }

    private void driveMecanum() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x; // <-- Added negative sign here to fix the turning direction

        // Deadzones to ignore controller stick drift (prevents going diagonal)
        if (Math.abs(forward) < 0.1) forward = 0;
        if (Math.abs(strafe) < 0.1) strafe = 0;
        if (Math.abs(turn) < 0.1) turn = 0;

        if (follower != null){
            // Field Centric Heading Reset Button
            if (gamepad1.options) {
                follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
            }

            // 'false' makes it field-centric drive
            follower.setTeleOpDrive(forward, strafe, turn, false);
            follower.update();
        }
    }
}