package org.firstinspires.ftc.teamcode.OFSB1.Vision;

import android.annotation.SuppressLint;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.easyobjd.ClusterInfo;
import org.firstinspires.ftc.easyobjd.EasyOBJD;
import org.firstinspires.ftc.easyobjd.EasyOBJDPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.OFSB1.Constants;
import org.firstinspires.ftc.teamcode.OFSB1.Subsystems.OFSB1Subsystem;
import org.firstinspires.ftc.teamcode.OFSB1.TeleOp.EasyOBJDUserConfig;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@SuppressWarnings("unused")
@TeleOp(name = "EasyOBJD Cluster Drive", group = "OFSB1")
public class EasyOBJDCluster extends OpMode {

    // ---------------- ROBOT / DRIVE ----------------
    private Follower follower;
    private OFSB1Subsystem robot;

    // ---------------- VISION ----------------
    private OpenCvWebcam webcam;
    private EasyOBJDPipeline pipeline;
    private volatile boolean cameraInitialized = false;

    @SuppressLint("DiscouragedApi")
    @Override
    public void init() {

        // =========================================================
        // ROBOT / PEDRO PATHING INITIALIZATION
        // =========================================================
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        robot = new OFSB1Subsystem(hardwareMap);

        // =========================================================
        // EASYOBJD CAMERA INITIALIZATION
        // =========================================================
        int cameraMonitorViewId =
                hardwareMap.appContext.getResources()
                        .getIdentifier(
                                "cameraMonitorViewId",
                                "id",
                                hardwareMap.appContext.getPackageName()
                        );

        webcam =
                OpenCvCameraFactory.getInstance().createWebcam(
                        hardwareMap.get(
                                WebcamName.class,
                                EasyOBJDUserConfig.WEBCAM_NAME
                        ),
                        cameraMonitorViewId
                );

        // Use fixed EasyOBJDUserConfig settings
        pipeline =
                EasyOBJD.createPipeline(
                        EasyOBJDUserConfig.create()
                );

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {

                    @Override
                    public void onOpened() {

                        webcam.startStreaming(
                                EasyOBJDUserConfig.STREAM_WIDTH,
                                EasyOBJDUserConfig.STREAM_HEIGHT,
                                EasyOBJDUserConfig.STREAM_ROTATION,
                                OpenCvWebcam.StreamFormat.MJPEG
                        );

                        cameraInitialized = true;
                    }

                    @Override
                    public void onError(int errorCode) {

                        cameraInitialized = false;

                        telemetry.addData(
                                "Camera Error",
                                errorCode
                        );
                    }
                }
        );

        telemetry.addData(
                "Status",
                "OFSB1 + EasyOBJD Initialized"
        );

        telemetry.update();
    }

    @Override
    public void init_loop() {

        addVisionTelemetry();

        telemetry.update();
    }

    @Override
    public void start() {

        // Start Pedro teleop driving
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        // =========================================================
        // UPDATE PEDRO PATHING
        // =========================================================
        follower.update();

        // =========================================================
        // NORMAL OFSB1 GAMEPAD DRIVE CONTROLS
        // =========================================================

        /*
         * LEFT STICK Y:
         * forward / backward
         */
        double forward =
                -gamepad1.left_stick_y;

        /*
         * LEFT TRIGGER:
         * strafe left
         *
         * RIGHT TRIGGER:
         * strafe right
         */
        double strafe =
                gamepad1.left_trigger
                        - gamepad1.right_trigger;

        /*
         * RIGHT STICK X:
         * turning
         */
        double turn =
                -gamepad1.right_stick_x;

        follower.setTeleOpDrive(
                forward,
                strafe,
                turn,
                true
        );

        // =========================================================
        // ADD FUTURE SUBSYSTEM CONTROLS HERE
        // =========================================================

        /*
         * Example:
         *
         * if (gamepad1.a) {
         *     robot.doSomething();
         * }
         */

        // =========================================================
        // ROBOT POSITION TELEMETRY
        // =========================================================

        telemetry.addData(
                "Robot X",
                "%.1f",
                follower.getPose().getX()
        );

        telemetry.addData(
                "Robot Y",
                "%.1f",
                follower.getPose().getY()
        );

        telemetry.addData(
                "Robot Heading",
                "%.1f",
                follower.getPose().getHeading()
        );

        // =========================================================
        // EASYOBJD TELEMETRY
        // =========================================================

        addVisionTelemetry();

        telemetry.update();
    }

    /**
     * EasyOBJD vision telemetry.
     *
     * HSV values and overlay mode are DISPLAYED ONLY.
     *
     * There are deliberately NO gamepad controls here
     * that change HSV or overlay.
     */
    private void addVisionTelemetry() {

        if (!cameraInitialized) {
            telemetry.addLine(
                    "Camera starting..."
            );
        }

        telemetry.addData(
                "Camera Initialized",
                cameraInitialized
        );

        telemetry.addLine(
                "EasyOBJD fixed configuration"
        );

        telemetry.addLine(
                "HSV / Overlay editing disabled"
        );

        // ---------------- CONFIG ----------------

        telemetry.addData(
                "Overlay",
                pipeline.getConfig().overlayMode
        );

        telemetry.addData(
                "Grid",
                "%d x %d",
                pipeline.getConfig().gridRows,
                pipeline.getConfig().gridCols
        );

        telemetry.addData(
                "HSV lower (H,S,V)",
                "%.0f, %.0f, %.0f",
                pipeline.getConfig().hsvLower.val[0],
                pipeline.getConfig().hsvLower.val[1],
                pipeline.getConfig().hsvLower.val[2]
        );

        telemetry.addData(
                "HSV upper (H,S,V)",
                "%.0f, %.0f, %.0f",
                pipeline.getConfig().hsvUpper.val[0],
                pipeline.getConfig().hsvUpper.val[1],
                pipeline.getConfig().hsvUpper.val[2]
        );

        // ---------------- DETECTIONS ----------------

        telemetry.addData(
                "Occupied cells",
                pipeline.getOccupiedCount()
        );

        telemetry.addData(
                "Clusters",
                pipeline.getClusterCount()
        );

        telemetry.addData(
                "Balls",
                pipeline.getBallCount()
        );

        List<ClusterInfo> clusters =
                pipeline.getClusters();

        for (ClusterInfo cluster : clusters) {

            telemetry.addLine(
                    "Cluster #" + cluster.id
            );

            telemetry.addData(
                    "#" + cluster.id + " cells",
                    cluster.cells.size()
            );

            telemetry.addData(
                    "#" + cluster.id + " X",
                    "%.1f in",
                    cluster.x
            );

            telemetry.addData(
                    "#" + cluster.id + " Y",
                    "%.1f in",
                    cluster.y
            );
        }
    }

    @Override
    public void stop() {

        // Stop robot mechanisms
        if (robot != null) {
            robot.stopAll();
        }

        // Stop camera
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }
}