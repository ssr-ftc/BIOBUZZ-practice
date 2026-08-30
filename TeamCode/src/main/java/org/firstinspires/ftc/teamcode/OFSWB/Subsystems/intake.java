package org.firstinspires.ftc.teamcode.OFSWB.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake {

    private DcMotor intakeMotor;
    private static final String MOTOR_NAME = "intake";
    private static final double intake_power = 0.967;
    private static final double outtake_power = -0.967;

    public intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void turn_on_intake() {
        intakeMotor.setPower(intake_power);
    }
    public void turn_off_intake() {
        intakeMotor.setPower(0);
    }
    public void outtake() {
        intakeMotor.setPower(outtake_power);
    }
    public boolean isIntakeOn() {
        boolean x = (intakeMotor.getPower() != 0);
        return x;
    }
    public double getMotorPower() {
        return intakeMotor.getPower();
    }
}

// hi