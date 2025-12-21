package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Lock-On Test", group = "Test")
@Disabled
public class AprilTagLockOnTest extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private DcMotor turretMotor;

    /*
     * ---------------- TUNING VALUES ----------------
     */

    // Proportional gain
    private static final double kP = 0.004;

    // Motor output limits
    private static final double MAX_POWER = 0.4;

    // Deadzone in pixels (prevents jitter)
    private static final double PIXEL_DEADBAND = 10;

    // Camera resolution
    private static final int IMAGE_WIDTH = 640;

    @Override
    public void runOpMode() {

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTag
        );

        waitForStart();

        while (opModeIsActive()) {

            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (!detections.isEmpty()) {
                AprilTagDetection tag = detections.get(0); // Track first visible tag

                // Tag center X coordinate (pixels)
                double tagX = tag.center.x;

                // Image center
                double imageCenterX = IMAGE_WIDTH / 2.0;

                // Error: positive = tag is to the right
                double error = tagX - imageCenterX;

                double motorPower = 0;

                if (Math.abs(error) > PIXEL_DEADBAND) {
                    motorPower = kP * error;
                }

                motorPower = Range.clip(motorPower, -MAX_POWER, MAX_POWER);

                turretMotor.setPower(motorPower);

                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Tag X", tagX);
                telemetry.addData("Error", error);
                telemetry.addData("Motor Power", motorPower);
            } else {
                turretMotor.setPower(0);
                telemetry.addLine("No AprilTag Detected");
            }

            telemetry.update();
        }

        visionPortal.close();
    }
}
