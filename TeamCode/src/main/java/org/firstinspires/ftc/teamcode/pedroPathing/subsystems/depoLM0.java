package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class depoLM0 {
    private DcMotor top;
    private DcMotorEx bottom;
    private boolean running = false;

    public static class PIDFController {
        public double p, i, d, kF;
        private double lastError = 0;
        private double integralSum = 0;

        public PIDFController(double p, double i, double d, double kF) {
            this.p = p;
            this.i = i;
            this.d = d;
            this.kF = kF;
        }

        public double calculate(double current, double target) {
            double error = target - current;
            integralSum += error;
            double derivative = error - lastError;
            lastError = error;
            return (p * error) + (i * integralSum) + (d * derivative) + (kF * target);
        }
    }

    public depoLM0(HardwareMap hardwareMap) {
        top = hardwareMap.get(DcMotor.class, "depo");
        bottom = hardwareMap.get(DcMotorEx.class, "depo1");

        top.setDirection(DcMotorSimple.Direction.FORWARD);
        bottom.setDirection(DcMotorSimple.Direction.REVERSE);

        top.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new PIDFController(p, i, d, kF);
    }

    PIDFController pid;
    public double power;
    public static double p = 0.00346;
    public static double i = 0.00047;
    public static double d = 0.000035;
    public static double kF = 0.0004001;

    public static double targetVelocity = 0;

    public void set_target_velocity(double x){
        targetVelocity = x;
    }

    public void run_using_pid(){
        power = pid.calculate(bottom.getVelocity(), targetVelocity);
        power = Math.max(-1, Math.min(1, power));
        bottom.setPower(power);
        top.setPower(power);
    }

    public void turn_on_deposit() {
        targetVelocity = 800;
    }

    public void turn_off_deposit() {
        targetVelocity = 0;
    }

    private void setPower(double power) {
        top.setPower(power);
        bottom.setPower(power);
    }

    public boolean isDepositOn() {
        return targetVelocity != 0;
    }

}
