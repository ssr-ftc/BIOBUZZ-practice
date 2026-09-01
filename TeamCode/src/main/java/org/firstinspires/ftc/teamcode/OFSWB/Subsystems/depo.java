package org.firstinspires.ftc.teamcode.OFSWB.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class depo {
    private DcMotor left;
    private DcMotor right;
    private DcMotor middle;
    private boolean running = false;
    public static double POWER = 0.7;

    public depo(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotor.class, "depo");
        right = hardwareMap.get(DcMotor.class, "depo1");
        middle = hardwareMap.get(DcMotor.class, "depo2");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);
        middle.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        middle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void turn_on_deposit() {
        running = true;
        setPower(POWER);
    }
    public void turn_off_deposit() {
        running = false;
        setPower(0);
    }
    private void setPower(double power) {
        left.setPower(power);
        right.setPower(power);
        middle.setPower(power);
    }
    public boolean isDepositOn() {
        return running;
    }
}