package org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;

/**
 * Opens Webcam 1 and reads BIOBUZZ tags 30-45. Does not pick a cell.
 */
public final class AprilTagReader {

    public static final String WEBCAM_NAME = "Webcam 1";

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public void init(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(TagClusterMapper.createTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(aprilTag)
                .build();
    }

    public List<AprilTagDetection> getDetections() {
        if (aprilTag == null) {
            return Collections.emptyList();
        }
        List<AprilTagDetection> detections = aprilTag.getDetections();
        return detections == null ? Collections.<AprilTagDetection>emptyList() : detections;
    }

    public boolean isStreaming() {
        return visionPortal != null
                && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }
}
