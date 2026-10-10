package org.firstinspires.ftc.teamcode.OFSWB.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.intake;
import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.depo;
import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.colorsensors;

import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.lifters;
import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.turret;
import org.firstinspires.ftc.teamcode.Timer;

@TeleOp(name = "Outreach TeleOp", group = "tests")
public class wbtelecolorsensors extends OpMode {

    private DcMotor lfmotor;
    private DcMotor lbmotor;
    private DcMotor rfmotor;
    private DcMotor rbmotor;

    private intake intake;
    private depo depo;
    private lifters lifters;
    private colorsensors colorsensors;
    private turret turret;
    private double speedScale = 0.8;
    private double xvelocity = -1800;

    private ElapsedTime shotTimer = new ElapsedTime();
    Timer timer;
    private int shotStep = 0;
    private boolean shooting = false;
    private boolean shootingActive = false; // true only while the lifter sequence is allowed to run

    private static final double warmup_seconds = 0.75;
    private static final double up_hold_seconds = 0.4;
    private static final double down_wait_seconds = 0.6;
    int ballCount;


    @Override
    public void init() {
        lfmotor = hardwareMap.get(DcMotor.class, "lfmotor");
        lbmotor = hardwareMap.get(DcMotor.class, "lbmotor");
        rfmotor = hardwareMap.get(DcMotor.class, "rfmotor");
        rbmotor = hardwareMap.get(DcMotor.class, "rbmotor");

        lfmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lbmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rfmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rbmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intake = new intake(hardwareMap);
        depo = new depo(hardwareMap);
        lifters = new lifters(hardwareMap);
        colorsensors= new colorsensors(hardwareMap);
        turret= new turret((hardwareMap));
        timer = new Timer();
        timer.createNew("intake");
        timer.createNew("depo");
    }

    @Override
    public void start() {
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
        if(gamepad2.crossWasPressed()){//start shooting
            depo.set_target_velocity(xvelocity);
            timer.start("depo");
            shootingActive = true;
        }

        // If the shot was stopped (or has finished), don't let the timeline touch the lifters
        if (!shootingActive) return;

        if (timer.checkSeconds("depo", 0.5)) {
            lifters.rightUp();
        }
        if (timer.checkSeconds("depo", 0.8)) {
            lifters.rightDown();
        }
        if (timer.checkSeconds("depo", 1.1)) {
            lifters.backUp();
        }
        if (timer.checkSeconds("depo", 1.4)) {
            lifters.backDown();
        }
        if (timer.checkSeconds("depo", 1.7)) {
            lifters.leftUp();
        }
        if (timer.checkSeconds("depo", 2.0)) {
            lifters.leftDown();
        }
        if (timer.checkSecondsLast("depo", 2.3)) {
            lifters.allDown();
            depo.turn_off_deposit();
            shootingActive = false;
        }
    }
    public void depoonoff(){
        if (gamepad2.triangleWasPressed() ) {
            if (depo.isDepositOn() || shootingActive){
                shootingActive = false;
                depo.turn_off_deposit();
                lifters.allDown();
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
        double forward = -gamepad1.left_stick_y * speedScale;
        double strafe = gamepad1.left_stick_x * speedScale;
        double turn = gamepad1.right_stick_x * speedScale;

        lfmotor.setPower(forward + strafe + turn);
        lbmotor.setPower(forward - strafe + turn);
        rfmotor.setPower(forward - strafe - turn);
        rbmotor.setPower(forward + strafe - turn);
    }
}