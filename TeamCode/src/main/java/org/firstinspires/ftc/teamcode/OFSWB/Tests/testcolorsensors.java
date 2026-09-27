package org.firstinspires.ftc.teamcode.OFSWB.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * TeleOp for Off Season Bot 1 (OFSWB).
 * Uses PedroPathing for driving.
 */
@TeleOp(name = "Color Sensor Test", group = "OFSWB")
public class testcolorsensors extends OpMode {
//

    private ColorSensor left;
    private ColorSensor left2;
    private ColorSensor back;
    private ColorSensor back2;
    private ColorSensor right;
    private ColorSensor right2;


    @Override
    public void init() {
        left = hardwareMap.get(ColorSensor.class, "color_left");
        left2 = hardwareMap.get(ColorSensor.class, "color_left2");
        back = hardwareMap.get(ColorSensor.class, "color_back");
        back2 = hardwareMap.get(ColorSensor.class, "color_back2");
        right = hardwareMap.get(ColorSensor.class, "color_right");
        right2 = hardwareMap.get(ColorSensor.class, "color_right2");


        telemetry.addData("Status", "OFSWB Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
//        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        left.green();
//        follower.update();

        // Driving control

        // Add robot subsystem controls here
        // if (gamepad1.a) robot.doSomething();
        if(left2.green() > left.green()) {
            telemetry.addData("green", left2.green());
        } else{
            telemetry.addData("green", left.green());
        }

        if(left2.red() > left.red()) {
            telemetry.addData("red", left2.red());
        } else{
            telemetry.addData("red", left.red());
        }

        if(left2.blue() > left.blue()) {
            telemetry.addData("blue", left2.blue());
        } else{
            telemetry.addData("blue", left.blue());
        }
        telemetry.addData("is there a ball in left?",isBall(left,left2));
        telemetry.addData("is there a ball in right?",isBall(right,right2));
        telemetry.addData("is there a ball in back?",isBall(back,back2));
        telemetry.addData("are all 3 balls in?",allBallsIn());
        telemetry.update();
    }
    boolean allBallsIn(){
        return  false;//to be changed
    }
    boolean isBall(ColorSensor sensor, ColorSensor sensor2){//returns true if ball is in left slot
        double red,blue,green;
        if(sensor2.red() > sensor.red()) {
            red = sensor2.red();
        } else{
            red =sensor.red();
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

    @Override
    public void stop() {

    }
}
