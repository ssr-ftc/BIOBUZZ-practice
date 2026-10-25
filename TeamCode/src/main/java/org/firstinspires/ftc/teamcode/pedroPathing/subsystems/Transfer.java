package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {
    Intake intake;
    Servo flap;




    public Transfer(HardwareMap hardwareMap) {
        intake = new Intake(hardwareMap);
        flap = hardwareMap.get(Servo.class, "flap");
    }
    public void start(){
        intake.start();
    }


}
//hi