package org.firstinspires.ftc.teamcode.OFSWB.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class turret {

    public static double jogTicksPerLoop = 5.0;

    private static final double SERVO_FORWARD   = 0.515;
    private static final double SERVO_MAX_RIGHT = 0.08;
    private static final double SERVO_MAX_LEFT  = 0.95;
    private static final double MIN_TICKS = -670;
    private static final double MAX_TICKS =  670;

    private final Servo turret;
    private final Servo turret2;
    private double currentTicks = 0.0;

    public turret   (HardwareMap hardwareMap) {

        turret  = hardwareMap.get(Servo.class, "turret");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        setPosition(0);
    }

    public void moveRight() {
        setPosition(currentTicks + jogTicksPerLoop);
    }

    public void moveLeft() {
        setPosition(currentTicks - jogTicksPerLoop);
    }

    public void stop() { }

    public double getCurrentTicks() {
        return currentTicks;
    }

    private void setPosition(double ticks) {
        ticks = Math.max(MIN_TICKS, Math.min(MAX_TICKS, ticks));
        currentTicks = ticks;

        double slope = ticks >= 0
                ? (SERVO_MAX_RIGHT - SERVO_FORWARD) / MAX_TICKS
                : (SERVO_MAX_LEFT - SERVO_FORWARD) / MIN_TICKS;
        double pos = SERVO_FORWARD + ticks * slope;

        turret.setPosition(pos);
        turret2.setPosition(pos);
    }
}