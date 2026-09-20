package org.firstinspires.ftc.teamcode.OFSB2.Teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.OFSB2.Auto.Constants;

@TeleOp(name = "fieldCentricTeleop", group = "TeleOp")
public class fieldCentricTeleop extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private DcMotorEx intake;

    private Pose lastPose = new Pose(0, 0, 0);
    private double tripFeet = 0;
    private static double allTimeFeet = 0;
    private android.content.SharedPreferences prefs;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        follower.setStartingPose(startPose);

        prefs = hardwareMap.appContext.getSharedPreferences("RobotData", android.content.Context.MODE_PRIVATE);
        allTimeFeet = prefs.getFloat("allTimeOdo", 0.0f);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        drive();
        useIntake();
        telemetry();
    }
    public void drive() {
        double speedShift = 0.7;
        if (gamepad1.right_trigger > 0.1) {
            speedShift = 0.3;
        }
        double currentPower = Math.abs(gamepad1.left_stick_y) * speedShift;

        if (gamepad1.options) {
            follower.setPose(new Pose(0, 0, Math.toRadians(0)));
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speedShift,
                -gamepad1.left_stick_x * speedShift,
                -gamepad1.right_stick_x * speedShift * 0.8,
                true
        );


        follower.update();

        double deltaFeet = follower.getPose().distanceFrom(lastPose) / 12.0;
        tripFeet += deltaFeet;
        allTimeFeet += deltaFeet;
        lastPose = follower.getPose();
    }

    public void telemetry() {
        telemetry.addLine("-----Fuel-----");
        telemetry.addData("Fuel Level", fuelLevel());
        telemetry.addLine("                            ");
        telemetry.addLine("-----Distance-----");
        telemetry.addData("IPS", "%.2f", follower.getVelocity().getMagnitude());
        telemetry.addData("Feet Traveled In Session", "%.2f ft", tripFeet);
        telemetry.addData("All-Time Feet Traveled", "%.2f ft", allTimeFeet);
        telemetry.addLine("                            ");
        telemetry.addLine("-----Navigation-----");
        telemetry.addData("Compass", getCompass());
        telemetry.update();
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
        else if (voltage > 14) return "Pristine";
        else if (voltage < 8) return "Fried Battery...Throw Away";
        else return "Plug in New Battery";

    }



    public String getCompass() {
        double degrees = Math.toDegrees(follower.getPose().getHeading());
        while (degrees < 0) degrees += 360;
        while (degrees >= 360) degrees -= 360;

        if (degrees > 337.5 || degrees <= 22.5)      return "N";
        else if (degrees > 22.5  && degrees <= 67.5)  return "NE";
        else if (degrees > 67.5  && degrees <= 112.5) return "E";
        else if (degrees > 112.5 && degrees <= 157.5) return "SE";
        else if (degrees > 157.5 && degrees <= 202.5) return "S";
        else if (degrees > 202.5 && degrees <= 247.5) return "SW";
        else if (degrees > 247.5 && degrees <= 292.5) return "W";
        else return "NW";
    }
    public void useIntake() {
        if (gamepad1.right_bumper) {
            intake.setPower(1.00);
        }else {
            intake.setPower(0);
        }

    }

    @Override
    public void stop() {
        prefs.edit().putFloat("allTimeOdo", (float)allTimeFeet).apply();
    }
}






