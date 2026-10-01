package org.firstinspires.ftc.teamcode.OFSB2.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class turret {
    public Servo servoturret;
    public turret(HardwareMap hardwareMap) {
        servoturret = hardwareMap.get(Servo.class, "turret");
    }
    public void setservotodegree(double x) {
        double y = -0.00236111 * x + 0.545;
        servoturret.setPosition(y);
    }
    public double servovalueindegreees() {
        double y = -423.52941 * servoturret.getPosition() + 230.82353;
        return y;
    }
    public void move1degleft() {
        if(servovalueindegreees()>-180){
            setservotodegree(servovalueindegreees() - 1);
        }
    }
    public void move1degright() {
        if (servovalueindegreees() < 180) {
            setservotodegree(servovalueindegreees() + 1);
        }
    }
}