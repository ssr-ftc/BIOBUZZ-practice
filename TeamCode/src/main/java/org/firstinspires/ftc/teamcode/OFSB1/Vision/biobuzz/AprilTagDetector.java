package org.firstinspires.ftc.teamcode.OFSB1.Vision.biobuzz;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Reads the webcam. Does not decide which cell matters.
 * Uses the existing {@link AprilTagWebcam} exposure, resolution, and decimation.
 */
public final class AprilTagDetector {

    private final AprilTagWebcam webcam = new AprilTagWebcam();
    private List<AprilTagDetection> latest = Collections.emptyList();

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        webcam.init(hardwareMap, telemetry, TagClusterMapper.createTagLibrary());
    }

    public void update() {
        webcam.update();
        List<AprilTagDetection> tags = webcam.getDetectedTags();
        latest = tags == null
                ? Collections.<AprilTagDetection>emptyList()
                : new ArrayList<>(tags);
    }

    public List<AprilTagDetection> getDetections() {
        return latest;
    }

    public boolean isStreaming() {
        return webcam.isStreaming();
    }

    public void stop() {
        webcam.stop();
    }
}
