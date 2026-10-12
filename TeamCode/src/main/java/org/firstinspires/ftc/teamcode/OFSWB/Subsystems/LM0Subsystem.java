package org.firstinspires.ftc.teamcode.OFSWB.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem for the LM0 robot.
 */
public class LM0Subsystem {

    public final lm0intake intake;

    public LM0Subsystem(HardwareMap hardwareMap) {
        intake = new lm0intake(hardwareMap);
    }

    public void stopAll() {
        intake.turn_off_intake();
    }
}
