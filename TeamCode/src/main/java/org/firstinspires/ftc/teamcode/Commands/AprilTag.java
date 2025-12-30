package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTag {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public void init(HardwareMap hardwareMap) {

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    public void scan() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (!detections.isEmpty()) {
            // Store FIRST detected tag globally
            AprilTagGlobals.detectedTagID = detections.get(0).id;
        }
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}

