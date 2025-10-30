package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagDriveSubsystem {

    // --- Constants ---
    private static final double DESIRED_DISTANCE = 60.0; // in inches
    private static final double SPEED_GAIN  = 0.02;
    private static final double STRAFE_GAIN = 0.015;
    private static final double TURN_GAIN   = 0.01;

    private static final double MAX_AUTO_SPEED  = 0.5;
    private static final double MAX_AUTO_STRAFE = 0.5;
    private static final double MAX_AUTO_TURN   = 0.3;

    // --- Hardware ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    private final boolean USE_WEBCAM = true;
    private final HardwareMap hwMap;
    private final Telemetry telemetry;

    // Heading supplier for field-centric drive
    private HeadingSupplier headingSupplier;

    /** A functional interface for fetching heading from IMU or odometry **/
    public interface HeadingSupplier {
        double getHeadingRadians(); // must return radians
    }

    public AprilTagDriveSubsystem(HardwareMap hw, Telemetry tel) {
        this.hwMap = hw;
        this.telemetry = tel;
        initHardware();
        initAprilTag();

        if (USE_WEBCAM) setManualExposure(6, 250);
    }

    private void initHardware() {
        frontLeft  = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft   = hwMap.get(DcMotor.class, "backLeft");
        backRight  = hwMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /** Supply an external heading source (IMU or OTOS) for field-centric driving */
    public void setHeadingSupplier(HeadingSupplier supplier) {
        this.headingSupplier = supplier;
    }

    /** Main update — can be used in TeleOp or Autonomous */
    public void driveToTag(boolean autoEnabled, double manualDrive, double manualStrafe, double manualTurn, int tagNumber) {
        boolean targetFound = false;
        double drive = 0, strafe = 0, turn = 0;

        desiredTag = null;
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null &&
                    ((tagNumber < 0) || (detection.id == tagNumber))) {
                targetFound = true;
                desiredTag = detection;
                break;
            }
        }

        if (targetFound) {
            double rangeError   = desiredTag.ftcPose.range - DESIRED_DISTANCE;
            double headingError = desiredTag.ftcPose.bearing;
            double yawError     = desiredTag.ftcPose.yaw;

            if (autoEnabled) {
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            }

            telemetry.addData("Tag", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range", "%.1f in", desiredTag.ftcPose.range);
        } else {
            drive  = manualDrive;
            strafe = manualStrafe;
            turn   = manualTurn;
            telemetry.addData("Status", "No tag detected");
        }

        telemetry.addData("Mode", autoEnabled && targetFound ? "AUTO" : "MANUAL");
        telemetry.update();

        moveRobotFieldCentric(drive, strafe, turn);
    }

    /** Field-centric drive transformation */
    private void moveRobotFieldCentric(double forward, double strafe, double turn) {
        double botHeading = 0;
        if (headingSupplier != null) botHeading = headingSupplier.getHeadingRadians();

        // Rotate input by negative heading
        double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        moveRobot(rotY, rotX, turn); // (y, x, turn)
    }

    /** Standard mecanum mixing */
    private void moveRobot(double y, double x, double yaw) {
        double frontLeftPower  = y + x + yaw;
        double frontRightPower = y - x - yaw;
        double backLeftPower   = y - x + yaw;
        double backRightPower  = y + x - yaw;

        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    /** Manual exposure tweak for consistent lighting */
    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                try { Thread.sleep(20); } catch (InterruptedException ignored) {}
            }
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }
}
