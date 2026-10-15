package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class depoLM0 {
    private DcMotor top;
    private DcMotorEx bottom;
    private boolean running = false;

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
    // Feedforward coefficient (applied by PIDFController to target velocity)
    public static double kF = 0.0004001; // adjust for your motor's max ticks/sec

    // Target velocity in ticks per second
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