package org.firstinspires.ftc.teamcode.OFSB1.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "WebcamExample", group = "OFSB1")
public class WebcamExample extends OpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id21 = aprilTagWebcam.getTagBySpecificId(21);
        aprilTagWebcam.displayDetectionTelemetry(id21);
    }
}
