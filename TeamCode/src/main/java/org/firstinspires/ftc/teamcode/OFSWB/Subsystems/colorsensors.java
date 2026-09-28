package org.firstinspires.ftc.teamcode.OFSWB.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class colorsensors {

   public ColorSensor left;
    public ColorSensor left2;
    public ColorSensor back;
    public ColorSensor back2;
    public ColorSensor right;
    public ColorSensor right2;
    public colorsensors(HardwareMap hardwareMap) {
        left = hardwareMap.get(ColorSensor .class, "color_left");
        left2 = hardwareMap.get(ColorSensor.class, "color_left2");
        back = hardwareMap.get(ColorSensor.class, "color_back");
        back2 = hardwareMap.get(ColorSensor.class, "color_back2");
        right = hardwareMap.get(ColorSensor.class, "color_right");
        right2 = hardwareMap.get(ColorSensor.class, "color_right2");
    }

    public boolean isBall(ColorSensor sensor, ColorSensor sensor2){ // returns true if ball is in slot
        double red,blue,green;
        if(sensor2.red() > sensor.red()) {
            red = sensor2.red();
        } else{
            red = sensor.red();
        }
        if(sensor2.blue() > sensor.blue()) {
            blue = sensor2.blue();
        } else{
            blue = sensor.blue();
        }if(sensor2.green() > sensor.green()) {
            green = sensor2.green();
        } else{
            green = sensor.green();
        }
        if (red > 100|| blue > 100|| green > 100) {
            return true;
        }
        else return false;
    }

    public int countBalls(){
        int count = 0;
        if(isBall(left,left2)){
            count++;
        }
        if(isBall(right,right2)){
            count++;
        }
        if(isBall(back,back2)){
            count++;
        }
        return count;
    }
    public boolean isleftfull(){
        return(isBall(left,left2));
    }
    public boolean isrightfull(){
        return(isBall(right,right2));
    }
    public boolean isbackfull(){
        return(isBall(back,back2));
    }












}
