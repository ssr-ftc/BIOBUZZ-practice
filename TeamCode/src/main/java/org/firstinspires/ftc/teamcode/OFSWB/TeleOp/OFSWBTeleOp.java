package org.firstinspires.ftc.teamcode.OFSWB.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OFSWB.Constants;
import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.OFSWBSubsystem;

/**
 * TeleOp for Off Season Bot 1 (OFSWB).
 * Uses PedroPathing for driving.
 */
@TeleOp(name = "OFSWB TeleOp", group = "OFSWB")
public class OFSWBTeleOp extends OpMode {

    private Follower follower;
    private OFSWBSubsystem robot;

    @Override
    public void init() {
        // Initialize PedroPathing Follower using OFSB1-specific constants
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Initialize robot subsystems
        robot = new OFSWBSubsystem(hardwareMap);

        telemetry.addData("Status", "OFSWB Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        // Driving control
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        follower.setTeleOpDrive(forward, strafe, turn, true);

        // Add robot subsystem controls here
        // if (gamepad1.a) robot.doSomething();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stopAll();
    }
}
