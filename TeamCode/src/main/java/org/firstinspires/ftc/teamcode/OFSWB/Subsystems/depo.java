package org.firstinspires.ftc.teamcode.OFSWB.Subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Timer;

public class depo {
    private DcMotor left;
    private DcMotorEx right;
    private DcMotor middle;
    private boolean running = false;
    public static double POWER = 0.7;

    public depo(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotor.class, "depo");
        right = hardwareMap.get(DcMotorEx.class, "depo1");
        middle = hardwareMap.get(DcMotor.class, "depo2");

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        middle.setDirection(DcMotorSimple.Direction.FORWARD);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        middle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pid = new PIDFController(p, i, d, kF);
    }
    PIDFController pid;

    public double power;

    public static double p = 0.001;
    public static double i = 0.0;
    public static double d = 0.00;
    // Feedforward coefficient (applied by PIDFController to target velocity)
    public static double kF = 0.000385; // adjust for your motor's max ticks/sec

    // Target velocity in ticks per second
    public static double targetVelocity = 0;

    public void set_target_velocity(double x){
       targetVelocity = x;
    }
    public void run_using_pid(){
        power = pid.calculate(right.getVelocity(), targetVelocity);
        power = Math.max(-1, Math.min(1, power));
        middle.setPower(power);
        right.setPower(power);
        left.setPower(power);
    }
    public void turn_on_deposit() {
        targetVelocity = -1000;
    }
    public void turn_off_deposit() {
        targetVelocity = 0;
    }
    private void setPower(double power) {
        left.setPower(power);
        right.setPower(power);
        middle.setPower(power);
    }
    public boolean isDepositOn() {
        return targetVelocity != 0;
    }

}