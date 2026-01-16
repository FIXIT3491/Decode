package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Flywheel TeleOp")
public class WebcamAutoFlywheelTest extends LinearOpMode {

    private DcMotorEx flywheel;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private static final double TICKS_PER_REV = 28.0;

    @Override
    public void runOpMode() {

        // Flywheel motor
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // AprilTag processor
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Vision Portal using Logitech Webcam
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTag
        );

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double targetRPM = 0.0;

            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (!detections.isEmpty()) {
                AprilTagDetection tag = detections.get(0);

                double distanceInches = tag.ftcPose.range;

                targetRPM = distanceToRPM(distanceInches);

                setFlywheelRPM(-targetRPM);

                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Distance (in)", distanceInches);
            } else {
                // No tag → stop flywheel
                flywheel.setVelocity(0);
            }

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", getCurrentRPM());
            telemetry.update();
        }
    }


    private double distanceToRPM(double distanceInches) {

        final double slope = 22.0;     //Rpm/in?
        final double intercept = 2100; // Base RPM

        double rpm = slope * distanceInches + intercept;

        // Safety limits for 5202
        return Range.clip(rpm, 2500, 4500);
    }


    /*
     * Sets flywheel velocity in RPM
     */
    private void setFlywheelRPM(double rpm) {
        double ticksPerSecond = (rpm / TICKS_PER_REV) * 60.0;
        flywheel.setVelocity(ticksPerSecond);
    }

    private double getCurrentRPM() {
        return (flywheel.getVelocity() / TICKS_PER_REV) * 60.0;
    }
}
