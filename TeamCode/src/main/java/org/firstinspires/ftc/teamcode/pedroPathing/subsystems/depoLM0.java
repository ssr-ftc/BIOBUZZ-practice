package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

//import com.acmerobotics.dashboard.config.Config;
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

    public static class PIDFController {
        public double p, i, d, kF;
        private double lastError = 0;
        private double integralSum = 0;
        private static final double MAX_INTEGRAL = 1000; // tune; limits windup

        public PIDFController(double p, double i, double d, double kF) {
            this.p = p;
            this.i = i;
            this.d = d;
            this.kF = kF;
        }

        public void reset() {
            integralSum = 0;
            lastError = 0;
        }

        public double calculate(double current, double target) {
            double error = target - current;
            integralSum += error;
            integralSum = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integralSum));
            double derivative = error - lastError;
            lastError = error;
            return (p * error) + (i * integralSum) + (d * derivative) + (kF * target);
        }
    }

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

        pid = new PIDFController(p, i, d, kF);
    }
    public void start(){
        pid.p=p;
        pid.i=i;
        pid.d=d;
        pid.kF=kF;
      //  hatch_close();
        turn_off_deposit();
    }
    public void updatePID(){
        pid.p=p;
        pid.i=i;
        pid.d=d;
        pid.kF=kF;
    }


    PIDFController pid;
    public double power;
    public static double p = 0.00346;
    public static double i = 0.00047;
    public static double d = 0.000035;
    public static double kF = 0.0004001;
    public static double ON_VELOCITY = 1470; // change the deposit speed here (prev 550)
    public static double targetVelocity = 0;
    public static double hatchopen = 0.66;
    public static double hatchclose = 0.54;

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

    public boolean ishatchopen(){
        boolean x = hatch.getPosition() > (hatchclose + 0.04);
        return x;
    }



    public void hatch_open(){
            hatch.setPosition(hatchopen);
    }
public void hatch_close(){
        hatch.setPosition(hatchclose);
    }


    public double getVelocity() {
        return bottom.getVelocity();
    }
    public double getTopVelocity() {
        return top.getVelocity();
    }


    public boolean isDepositOn() {
        return targetVelocity != 0;
    }
}
//hi