package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
//@Config
public class Depo {
    private DcMotorEx top;
    private DcMotorEx bottom;
    public Servo hatch;
    public Servo hood;
    public Servo cam;

    public Depo(HardwareMap hardwareMap) {
        top = hardwareMap.get(DcMotorEx.class, "depo");
        bottom = hardwareMap.get(DcMotorEx.class, "depo1");

        top.setDirection(DcMotorSimple.Direction.FORWARD);
        bottom.setDirection(DcMotorSimple.Direction.FORWARD
        );

        top.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // needed so power works and velocity still reads

        top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hatch = hardwareMap.get(Servo.class,"hatch");
        hood = hardwareMap.get(Servo.class,"hood");
        cam = hardwareMap.get(Servo.class,"camera");

        pid = new PIDFController(p, i, d, kF);
    }
    public void start(){
        pid.setPIDF(p,i,d,kF);
        close_hatch();
        turn_off_deposit();
        setHood(hoodMin);
    }
    public void updatePID(){
        pid.setPIDF(p,i,d,kF);
    }
    PIDFController pid;
    public double power;
    public static double p = 0.004; //0.0188
    public static double i = 0.000;
    public static double d = 0.0000;
    public static double kF = 0.0005;
    private static double targetVelocity = 0;

    //important constatnts below
    double ON_VELOCITY = 1520; // change the deposit speed here (prev 550)
    double hatchopen = 0.45;
    double hatchclose = 0.29;
    double cam_up = 0.2;
    double cam_down = 0.8;
    double camMiddle = 0.5;
    public double hoodMax = 0.5;
    public double hoodMin = 0.44;



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
        top.setPower(-power);
        bottom.setPower(-power);
    }

    public void turn_on_deposit() {
        pid.reset();
        targetVelocity = ON_VELOCITY;
    }

    public void turn_off_deposit() {
        targetVelocity = 0;
        pid.reset();
        power = 0;
        top.setPower(0);
        bottom.setPower(0);
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

    /** Negative turret degrees use hatchopen. Zero and positive degrees use hatchopen_2. */
    public void open_hatch() {hatch.setPosition(hatchopen);}

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
    public void setHood(double pos){
        if(pos<=hoodMax && pos>=hoodMin) {
            hood.setPosition(pos);
        }
    }
    public double getHood(){
        return hood.getPosition();
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
    public double distToVelo(double dist){
        return 5.75489*dist+1279.47344;
    }
    public double distToAngle(double dist){
        if(dist>35) return 0.44; //far/normal positons
        else return 0.47; //close positions
    }
}
//hi

