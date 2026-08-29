package org.firstinspires.ftc.teamcode.OFSWB.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
public class limitswitchlm2 {
    private TouchSensor sensor;

    public limitswitchlm2(HardwareMap hardwareMap, String name) {
        sensor = hardwareMap.get(TouchSensor.class, name);
    }

    public boolean isPressed() {
        return sensor.isPressed();
    }
}