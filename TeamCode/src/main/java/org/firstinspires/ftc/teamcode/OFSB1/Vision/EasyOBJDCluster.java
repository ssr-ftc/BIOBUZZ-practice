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

@TeleOp(name = "EasyOBJD Cluster Drive", group = "OFSB1")
public class EasyOBJDCluster extends OpMode {

    // Use -6.0 instead if the displayed distance is six inches too large.
    private static final double DISTANCE_OFFSET_INCHES = 6.0;

    private Follower follower;
    private OFSB1Subsystem robot;
    private OpenCvWebcam webcam;
    private EasyOBJDPipeline pipeline;
    private volatile boolean cameraInitialized = false;
    private volatile int cameraError = -1;

    @SuppressLint("DiscouragedApi")
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        robot = new OFSB1Subsystem(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(
                        WebcamName.class,
                        EasyOBJDUserConfig.WEBCAM_NAME
                ),
                cameraMonitorViewId
        );

        pipeline = EasyOBJD.createPipeline(EasyOBJDUserConfig.create());
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
                        cameraError = errorCode;
                    }
                }
        );

        telemetry.addData("Status", "OFSB1 + EasyOBJD Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Camera Initialized", cameraInitialized);

        if (cameraError != -1) {
            telemetry.addData("Camera Error", cameraError);
        }

        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_trigger - gamepad1.right_trigger;
        double turn = -gamepad1.right_stick_x;

        follower.setTeleOpDrive(forward, strafe, turn, true);

        telemetry.addData("Robot X", "%.1f", follower.getPose().getX());
        telemetry.addData("Robot Y", "%.1f", follower.getPose().getY());
        telemetry.addData(
                "Robot Heading",
                "%.1f",
                follower.getPose().getHeading()
        );

        // All vision telemetry is directly inside loop().
        telemetry.addData("Camera Initialized", cameraInitialized);

        if (!cameraInitialized) {
            telemetry.addLine("Camera starting...");
        }

        if (cameraError != -1) {
            telemetry.addData("Camera Error", cameraError);
        }

        telemetry.addData("Overlay", pipeline.getConfig().overlayMode);
        telemetry.addData(
                "Grid",
                "%d x %d",
                pipeline.getConfig().gridRows,
                pipeline.getConfig().gridCols
        );
        telemetry.addData("Occupied cells", pipeline.getOccupiedCount());
        telemetry.addData("Clusters", pipeline.getClusterCount());
        telemetry.addData("Balls", pipeline.getBallCount());
        telemetry.addData(
                "Distance correction",
                "+%.1f in",
                DISTANCE_OFFSET_INCHES
        );

        List<ClusterInfo> clusters = pipeline.getClusters();

        for (ClusterInfo cluster : clusters) {
            telemetry.addLine("Cluster #" + cluster.id);
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
            telemetry.addData(
                    "#" + cluster.id + " method",
                    cluster.localization
            );

            double groundDistance = cluster.rangeInches;
            double lensDistance = Double.NaN;

            if (cluster.localization
                    == EasyOBJD.LocalizationMethod.FLOOR_PLANE) {
                double verticalDrop =
                        pipeline.getConfig().cameraHeightInches
                                - pipeline.getConfig().ballDiameterInches / 2.0;

                lensDistance = Math.hypot(
                        groundDistance,
                        verticalDrop
                );
            } else if (cluster.localization
                    == EasyOBJD.LocalizationMethod.SIZE_BASED
                    && cluster.balls.size() == 1) {
                ClusterInfo.Ball ball = cluster.balls.get(0);

                lensDistance = Math.sqrt(
                        ball.x * ball.x
                                + ball.y * ball.y
                                + ball.z * ball.z
                );
            }

            if (Double.isFinite(lensDistance)) {
                String label = (
                        cluster.ballCount > 1
                                ? "Distance to cluster #"
                                : "Distance to ball #"
                ) + cluster.id;

                telemetry.addData(
                        label,
                        "%.1f in (lens to center, corrected)",
                        lensDistance + DISTANCE_OFFSET_INCHES
                );
            } else {
                telemetry.addData(
                        "Distance to cluster #" + cluster.id,
                        "%.1f in (floor distance, corrected)",
                        groundDistance + DISTANCE_OFFSET_INCHES
                );
            }
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stopAll();
        }

        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }
}