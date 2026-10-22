package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import android.content.Context;
import android.content.SharedPreferences;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/// yo AI made some of this but pls no delete it is very dear to me

public class carDashboard {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Follower follower;

    private Pose lastPose = new Pose(0, 0, 0);
    private double tripFeet = 0;
    private static double allTimeFeet = 0;
    private final SharedPreferences prefs;

    public carDashboard(HardwareMap hardwareMap, Telemetry telemetry, Follower follower) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.follower = follower;

        prefs = hardwareMap.appContext.getSharedPreferences("RobotData", Context.MODE_PRIVATE);
        allTimeFeet = prefs.getFloat("allTimeOdo", 0.0f);
    }
    public void start() {
        lastPose = follower.getPose();
    }
    public void update() {
        double deltaFeet = follower.getPose().distanceFrom(lastPose) / 12.0;
        tripFeet += deltaFeet;
        allTimeFeet += deltaFeet;
        lastPose = follower.getPose();

        telemetry.addLine("-----------------FUEL-----------------");
        telemetry.addData("Fuel Level", fuelLevel());
        telemetry.addLine("                 ");
        telemetry.addLine("----------------DISTSNCE--------------");
        telemetry.addData("IPS", "%.2f", follower.getVelocity().getMagnitude());
        telemetry.addData("Feet Traveled In Session", "%.2f ft", tripFeet);
        telemetry.addData("All-Time Feet Traveled", "%.2f ft", allTimeFeet);
        telemetry.addLine("                            ");
        telemetry.addLine("---------------NAVIGATION-------------");
        telemetry.addData("Compass", getCompass());
        telemetry.addLine("                            ");
    }
    public String fuelLevel() {
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        if (voltage > 8 && voltage <= 8.6) return "Empty";
        else if (voltage > 8.6 && voltage <= 9.2) return "Empty";
        else if (voltage > 9.2 && voltage <= 9.8) return "Empty";
        else if (voltage > 9.8 && voltage <= 10.4) return "Low";
        else if (voltage > 10.4 && voltage <= 10.8) return "50%";
        else if (voltage > 11 && voltage <= 11.6) return "Medium";
        else if (voltage > 11.6 && voltage <= 12.2) return "Medium";
        else if (voltage > 12.2 && voltage <= 12.8) return "80%";
        else if (voltage > 12.8 && voltage <= 13.4) return "Full";
        else if (voltage > 13.4 && voltage <= 14) return "Full";
        else if (voltage > 14) return "Pristine like Sritan";
        else if (voltage < 8) return "Fried Battery...Throw Away";
        else return "Plug in New Battery";
    }
    public String getCompass() {
        double degrees = Math.toDegrees(follower.getPose().getHeading());
        while (degrees < 0) degrees += 360;
        while (degrees >= 360) degrees -= 360;
        if (degrees > 337.5 || degrees <= 22.5)      return "N";
        else if (degrees > 22.5  && degrees <= 67.5)  return "NW";
        else if (degrees > 67.5  && degrees <= 112.5) return "W";
        else if (degrees > 112.5 && degrees <= 157.5) return "SW";
        else if (degrees > 157.5 && degrees <= 202.5) return "S";
        else if (degrees > 202.5 && degrees <= 247.5) return "SE";
        else if (degrees > 247.5 && degrees <= 292.5) return "E";
        else return "NE";
    }

    public void stop() {
        prefs.edit().putFloat("allTimeOdo", (float) allTimeFeet).apply();
    }
}
