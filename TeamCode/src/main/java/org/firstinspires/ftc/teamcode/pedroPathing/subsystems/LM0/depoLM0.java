package org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0;

//import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
//@Config
public class depoLM0 {
    private DcMotorEx top;
    private DcMotorEx bottom;
    public Servo hatch;
    public Servo cam;

    public depoLM0(HardwareMap hardwareMap) {
        top = hardwareMap.get(DcMotorEx.class, "depo");
        bottom = hardwareMap.get(DcMotorEx.class, "depo1");

        top.setDirection(DcMotorSimple.Direction.FORWARD);
        bottom.setDirection(DcMotorSimple.Direction.REVERSE);

        top.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // needed so power works and velocity still reads

        top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hatch = hardwareMap.get(Servo.class,"hatch");
        cam = hardwareMap.get(Servo.class,"camera");

        pid = new PIDFController(p, i, d, kF);
    }
    public void start(){
        pid.setPIDF(p,i,d,kF);
        close_hatch();
        turn_off_deposit();
    }
    public void updatePID(){
        pid.setPIDF(p,i,d,kF);
    }


    PIDFController pid;
    public double power;
    public static double p = 0.00346;
    public static double i = 0.00047;
    public static double d = 0.000035;
    public static double kF = 0.0004501;
    private static double targetVelocity = 0;

    //important constatnts below
    public static double ON_VELOCITY = 1485; // change the deposit speed here (prev 550)
    public static double hatchopen = 0.65;
    public static double hatchopen_2 = 0.43;
    public static double hatchclose = 0.54;
    public static double cam_up = 0.2;
    public static double cam_down = 0.8;
    public static double camMiddle = 0.5;


    public void set_target_velocity(double x) {
        targetVelocity = x;
    }

    public void run_using_pid() {
        if (targetVelocity == 0) {
            power = 0;
            top.setPower(0);
            bottom.setPower(0);
            return;
        }
        power = pid.calculate(top.getVelocity(), targetVelocity);
        power = Math.max(-1, Math.min(1, power));
        top.setPower(power);
        bottom.setPower(power);
    }

    public void turn_on_deposit() {
        turn_on_deposit(ON_VELOCITY);
    }

    public void turn_on_deposit(double velocity) {
        pid.reset();
        targetVelocity = velocity;
    }

    public void turn_off_deposit() {
        targetVelocity = 0;
        pid.reset();
        power = 0;
        top.setPower(0);
        bottom.setPower(0);
    }

    public boolean ishatchopen(){
        double position = hatch.getPosition();
        boolean negativeSide = Math.abs(position - hatchopen) <= 0.04;
        boolean positiveSide = Math.abs(position - hatchopen_2) <= 0.04;
        return negativeSide || positiveSide;
    }

    public boolean reachedvelocity(){
        boolean x;
        if (targetVelocity != 0){
            x = getVelocity() >= targetVelocity - 20;
        } else {
            x = false;
        }
        return x;
    }



    public void open_hatch(){
        hatch.setPosition(hatchopen);
    }

    public void open_hatch_2(){
        hatch.setPosition(hatchopen_2);
    }

    /** Negative turret degrees use hatchopen. Zero and positive degrees use hatchopen_2. */
    public void open_hatch(double turretDeg) {
        if (turretDeg < 0) {
            open_hatch();
        } else {
            open_hatch_2();
        }
    }

    public void close_hatch(){
        hatch.setPosition(hatchclose);
    }
    public void setCam_up(){
        cam.setPosition(cam_up);
    }
    public void setCam_down(){
        hatch.setPosition(cam_down);
    }
    public void setCam_middle(){
        cam.setPosition(camMiddle);
    }


    public double getVelocity() {
        return top.getVelocity();
    }
    public double getTopVelocity() {
        return top.getVelocity();
    }


    public boolean isDepositOn() {
        return targetVelocity != 0;
    }
}
//hi
