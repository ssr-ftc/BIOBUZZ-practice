package org.firstinspires.ftc.teamcode.pedroPathing.templates;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Template iterative TeleOp using PedroPathing-style lifecycle methods.
 *
 * This uses init(), init_loop(), start(), loop(), and stop() instead of LinearOpMode.
 */
@TeleOp(name = "Template Pedro TeleOp", group = "Templates")
public class TemplateTeleOp extends OpMode {

//    private Follower follower;//this is pedropathing follower
    private TemplateSubsystem subsystem;
    private double speedScale = 0.8;


    @Override
    public void init() {//this is called once when the driver presses init
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(0, 0, 0));

        subsystem = new TemplateSubsystem(hardwareMap);

        telemetry.addLine("Initialized");
        telemetry.addData("Status", "Press start to run TeleOp");
        telemetry.addData("Starting Pose", "(%.1f, %.1f, %.1f)", 0.0, 0.0, 0.0);
        telemetry.update();
    }

    @Override
    public void init_loop() {//this is called repeatedly during initiallzation
//        follower.update();

        telemetry.update();
    }

    @Override
    public void start() {//this is called once when the driver presses start
//        follower.startTeleopDrive();
    }

    @Override
    public void loop() {//this is called repeatedly during the op mode
//        follower.update();
//        driveWithPedro();
        sendTelemetry();
    }

    @Override
    public void stop() {//this is called once when the driver presses stop
        subsystem.stopAll();
//        follower.setTeleOpDrive(0, 0, 0, true);
//        follower.update();
    }

    private void driveWithPedro() {

        double forward = -gamepad1.left_stick_y * speedScale;
        double strafe = -gamepad1.left_stick_x * speedScale;
        double turn = -gamepad1.right_stick_x * speedScale;

//        follower.setTeleOpDrive(forward, strafe, turn, true);
    }



    private void sendTelemetry() {
        telemetry.addLine("this is telemetry");
        telemetry.addData("speed scale",speedScale);//example of how to use telemetry
        telemetry.update();
    }
}
