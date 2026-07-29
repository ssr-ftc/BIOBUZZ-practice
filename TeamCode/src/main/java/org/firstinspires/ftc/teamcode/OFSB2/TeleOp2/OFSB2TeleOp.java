package org.firstinspires.ftc.teamcode.OFSB2.TeleOp2;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OFSB1.Constants1;
import org.firstinspires.ftc.teamcode.OFSB1.Subsystems1.OFSB1Subsystem;
import org.firstinspires.ftc.teamcode.OFSB1.Vision1.OFSB1VisionProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * TeleOp for Off Season Bot 1 (OFSB1).
 * Uses PedroPathing for driving.
 */
@TeleOp(name = "OFSB1 TeleOp", group = "OFSB1")
public class OFSB2TeleOp extends OpMode {

    private Follower follower;
    private OFSB1Subsystem robot;
    private OpenCvWebcam webcam;
    private OFSB1VisionProcessor visionProcessor;

    @Override
    public void init() {
        // Initialize PedroPathing Follower using OFSB1-specific constants
        follower = Constants1.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Initialize robot subsystems
        robot = new OFSB1Subsystem(hardwareMap);

        // Initialize Vision
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        visionProcessor = new OFSB1VisionProcessor();
        webcam.setPipeline(visionProcessor);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
            }
        });

        telemetry.addData("Status", "OFSB1 Initialized");
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
        if (webcam != null) {
            webcam.stopStreaming();
        }
    }
}
