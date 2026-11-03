package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Transfer {
    Servo flap;

    Rev2mDistanceSensor front_bottom;
    Rev2mDistanceSensor back_bottom;

    double flap_mid = 0.52;
    double flap_back = 0.75;
    double flap_front = 0.29;



    public Transfer(HardwareMap hardwareMap) {
        flap = hardwareMap.get(Servo.class, "flap");
        front_bottom = hardwareMap.get(Rev2mDistanceSensor.class, "front_bottom");
        back_bottom = hardwareMap.get(Rev2mDistanceSensor.class, "back_bottom");
    }
    public void start(){
        setFlap_mid();
    }
    public void setFlap_mid(){
        flap.setPosition(flap_mid);
    }
    public void setFlap_front(){
        flap.setPosition(flap_front);
    }
    public void setFlap_back(){
        flap.setPosition(flap_back);
    }
    public double getDistanceFront_MM(){
        return back_bottom.getDistance(DistanceUnit.MM);
    }public double getDistanceBack_MM(){
        return front_bottom.getDistance(DistanceUnit.MM);
    }


}
//hi