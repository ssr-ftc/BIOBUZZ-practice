package org.firstinspires.ftc.teamcode.OFSWB.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.intake;
import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.depo;
import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.lifters;
import org.firstinspires.ftc.teamcode.Timer;

@TeleOp(name = "wb tele w color sensors", group = "tests")
public class wbtelecolorsensors extends OpMode {

    private DcMotor lfmotor;
    private DcMotor lbmotor;
    private DcMotor rfmotor;
    private DcMotor rbmotor;

    private intake intake;
    private depo depo;
    private lifters lifters;
    private double speedScale = 0.8;

    private double xvelocity = -1000;

    private ElapsedTime shotTimer = new ElapsedTime();
    Timer timer;
    private int shotStep = 0;
    private boolean shooting = false;

    private static final double warmup_seconds = 0.75;
    private static final double up_hold_seconds = 0.4;
    private static final double down_wait_seconds = 0.6;

    private ColorSensor left;
    private ColorSensor left2;
    private ColorSensor back;
    private ColorSensor back2;
    private ColorSensor right;
    private ColorSensor right2;

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
        timer = new Timer();
        timer.createNew("intake");
        timer.createNew("depo");

        left = hardwareMap.get(ColorSensor.class, "color_left");
        left2 = hardwareMap.get(ColorSensor.class, "color_left2");
        back = hardwareMap.get(ColorSensor.class, "color_back");
        back2 = hardwareMap.get(ColorSensor.class, "color_back2");
        right = hardwareMap.get(ColorSensor.class, "color_right");
        right2 = hardwareMap.get(ColorSensor.class, "color_right2");
    }

    @Override
    public void start() {
        lifters.allDown();
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

    boolean isBall(ColorSensor sensor, ColorSensor sensor2){ // returns true if ball is in slot
        double red,blue,green;
        if(sensor2.red() > sensor.red()) {
            red = sensor2.red();
        } else{
            red = sensor.red();
        }
        if(sensor2.blue() > sensor.blue()) {
            blue = sensor2.blue();
        } else{
            blue = sensor.blue();
        }if(sensor2.green() > sensor.green()) {
            green = sensor2.green();
        } else{
            green = sensor.green();
        }
        if (red > 100|| blue > 100|| green > 100) {
            return true;
        }
        else return false;
    }

    int countBalls(){
        int count = 0;
        if(isBall(left,left2)){
            count++;
        }
        if(isBall(right,right2)){
            count++;
        }
        if(isBall(back,back2)){
            count++;
        }
        return count;
    }

    @Override
    public void loop() {
        driveMecanum();

        depo.run_using_pid();

        int ballCount = countBalls();

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

        if (gamepad2.triangleWasPressed() ) {
            if (depo.isDepositOn()){
                depo.turn_off_deposit();
            }
            else{
                depo.turn_on_deposit();
            }
        }
        if (gamepad2.dpadUpWasPressed()){
            xvelocity -= 100;
        }
        if (gamepad2.dpadDownWasPressed()){
            xvelocity += 100;
        }
        if(gamepad2.crossWasPressed()){ //start shooting
            depo.set_target_velocity(xvelocity);
            timer.start("depo");

        }
        if(timer.checkSeconds("depo",0.5)){
            lifters.rightUp();
        }
        if(timer.checkSeconds("depo",0.8)){
            lifters.rightDown();
            lifters.backUp();
        }
        if(timer.checkSeconds("depo",1.1)){
            lifters.backDown();
            lifters.leftUp();
        }
        if(timer.checkSecondsLast("depo",1.4)){
            lifters.allDown();
            depo.turn_off_deposit();
        }


        telemetry.addData("intake on", intake.isIntakeOn());
        telemetry.addData("depo on", depo.isDepositOn());
        telemetry.addData("xvelocity", xvelocity);
        telemetry.addData("ball count", ballCount);
    }
}