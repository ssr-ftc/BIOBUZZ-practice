package org.firstinspires.ftc.teamcode.OFSB2.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem for Off Season Bot 1 (OFSWB).
 * Define your hardware here and add helper methods.
 */
public class OFSB2Subsystem {

    // Add your motors and servos here
    // public DcMotorEx armMotor;
    // public Servo intakeServo;

    public OFSB2Subsystem(HardwareMap hardwareMap) {
        // Initialize your hardware
        // armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        // intakeServo = hardwareMap.get(Servo.class, "intake");

        // Set default behaviors
        // armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void stopAll() {
        // Stop all movement
    }
}
