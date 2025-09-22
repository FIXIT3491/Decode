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

    // --- Tuning constants ---
    private final double DESIRED_DISTANCE = 12.0;
    private final double SPEED_GAIN  = 0.02;
    private final double STRAFE_GAIN = 0.015;
    private final double TURN_GAIN   = 0.01;

    private final double MAX_AUTO_SPEED = 0.5;
    private final double MAX_AUTO_STRAFE= 0.5;
    private final double MAX_AUTO_TURN  = 0.3;

    // --- Hardware ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    private final boolean USE_WEBCAM = true;
    private final int DESIRED_TAG_ID = -1; // any tag
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public AprilTagDriveSubsystem(HardwareMap hw, Telemetry tel) {
        this.hardwareMap = hw;
        this.telemetry = tel;

        // Init hardware
        frontLeftDrive  = hw.get(DcMotor.class, "frontLeft");
        frontRightDrive = hw.get(DcMotor.class, "frontRight");
        backLeftDrive   = hw.get(DcMotor.class, "backLeft");
        backRightDrive  = hw.get(DcMotor.class, "backRight");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        initAprilTag();

        if (USE_WEBCAM) setManualExposure(6, 250);
    }

    // Called repeatedly in your OpMode loop
    public void driveToTag(boolean autoEnabled, double manualDrive, double manualStrafe, double manualTurn) {
        boolean targetFound = false;
        double drive = 0, strafe = 0, turn = 0;

        desiredTag = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null &&
                    ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))) {
                targetFound = true;
                desiredTag = detection;
                break;
            }
        }

        if (targetFound) {
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f deg", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f deg", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("Status", "Searching for tag...");
        }

        if (autoEnabled && targetFound) {
            double rangeError   = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = desiredTag.ftcPose.bearing;
            double yawError     = desiredTag.ftcPose.yaw;

            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Mode", "AUTO");
        } else {
            drive  = manualDrive;
            strafe = manualStrafe;
            turn   = manualTurn;
            telemetry.addData("Mode", "MANUAL");
        }

        telemetry.update();
        moveRobot(drive, strafe, turn);
    }

    public void moveRobot(double x, double y, double yaw) {
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

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
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }
}
