package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/** Thin wrapper around a Pedro 3 follower so existing autos can still call followPath. */
public class CustomFollower {
    public Follower pedro;
    private Telemetry telemetry;

    public CustomFollower(HardwareMap hardwareMap, Telemetry telemetry) {
        pedro = Constants.createFollower(hardwareMap);
        this.telemetry = telemetry;
    }

    public void update() {
        pedro.update();
        if (telemetry != null && pedro.isBusy()) {
            telemetry.addData("path completion", pedro.completion());
        }
    }

    public boolean isBusy() {
        return pedro.isBusy();
    }

    public void followPath(Path path, boolean holdEnd) {
        pedro.follow(path);
    }

    public void followPath(Path path, double maxPower, boolean holdEnd) {
        pedro.follow(path);
    }

    public void followPath(Path path) {
        pedro.follow(path);
    }

    public void setPose(Pose pose) {
        pedro.setPose(pose);
    }

    public Pose getPose() {
        return pedro.pose();
    }
}
