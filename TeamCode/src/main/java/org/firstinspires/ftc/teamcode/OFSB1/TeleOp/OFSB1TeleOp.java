package org.firstinspires.ftc.teamcode.OFSB1.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OFSB1.Constants;
import org.firstinspires.ftc.teamcode.OFSB1.Subsystems.OFSB1Subsystem;
import org.firstinspires.ftc.teamcode.OFSB1.Vision.OFSB1VisionProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

/**
 * TeleOp for Off Season Bot 1 (OFSB1).
 * Uses PedroPathing for driving.
 */
@TeleOp(name = "OFSB1 TeleOp", group = "OFSB1")
public class OFSB1TeleOp extends OpMode {

    private Follower follower;
    private OFSB1Subsystem robot;
    private OpenCvWebcam webcam;
    private OFSB1VisionProcessor visionProcessor;
    private volatile boolean cameraInitialized = false;

    @Override
    public void init() {
        // Initialize PedroPathing Follower using OFSB1-specific constants
        follower = Constants.createFollower(hardwareMap);
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
                // Camera is physically mounted upside down, so rotate the
                // stream 180 degrees. This makes image left/right/up/down
                // match the real world, so the X/Y math needs no sign flips.
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
                cameraInitialized = true;
            }

            @Override
            public void onError(int errorCode) {
                cameraInitialized = false;
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

        // ---- Pollen (yellow ball) vision telemetry ----
        List<OFSB1VisionProcessor.Detection> balls = visionProcessor.getDetections();
        telemetry.addData("Camera Initialized", cameraInitialized);
        telemetry.addData("Number of balls detected", balls.size());
        for (int i = 0; i < balls.size(); i++) {
            OFSB1VisionProcessor.Detection ball = balls.get(i);
            telemetry.addData("Ball #" + (i + 1),
                    "X: %.1f in, Y: %.1f in, Z: %.1f in, Area: %.0f px",
                    ball.x, ball.y, ball.z, ball.area);
        }

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
