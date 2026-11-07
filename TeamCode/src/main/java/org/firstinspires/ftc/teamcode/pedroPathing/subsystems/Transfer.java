package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Transfer {

    Rev2mDistanceSensor front_bottom;
    Rev2mDistanceSensor back_bottom;



    public Transfer(HardwareMap hardwareMap) {
        front_bottom = hardwareMap.get(Rev2mDistanceSensor.class, "front_bottom");
        back_bottom = hardwareMap.get(Rev2mDistanceSensor.class, "back_bottom");
    }
    public double getDistanceFront_MM(){
        return back_bottom.getDistance(DistanceUnit.MM);
    }public double getDistanceBack_MM(){
        return front_bottom.getDistance(DistanceUnit.MM);
    }


}
//hi