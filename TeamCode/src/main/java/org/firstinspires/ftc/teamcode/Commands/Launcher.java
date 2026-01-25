package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Commands.Launcher;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Launcher {

    /* ===================== HARDWARE ===================== */
    private DcMotorEx flywheel;
    private DcMotorEx turret;
    private Servo kick;

    /* ===================== VISION ===================== */
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    /* ===================== CONSTANTS ===================== */

    // Flywheel encoder
    private static final double FLYWHEEL_TICKS_PER_REV = 28.0;

    // Turret encoder constants (CHANGE THESE)
    private static final double TURRET_TICKS_PER_REV = 537.7; // example: goBILDA 5202
    private static final double TURRET_GEAR_RATIO = 1.0;     // motor:turret
    private static final double DEGREES_PER_TICK =
            360.0 / (TURRET_TICKS_PER_REV * TURRET_GEAR_RATIO);

    // Turret limits (degrees)
    private static final double TURRET_LEFT_LIMIT_DEG = -20.0;
    private static final double TURRET_RIGHT_LIMIT_DEG = 20.0;
    private static final double TURRET_DEADBAND_DEG = 1.5;

    // Turret control
    private static final double TURRET_KP = 0.01;
    private static final double TURRET_MAX_POWER = 0.4;

    // Flywheel PIDF
    private final double kF = 0.0002;
    private final double kP = 0.0004;
    private final double kI = 0.0;
    private final double kD = 0.0001;

    /* ===================== STATE ===================== */
    private double targetRPM = 0;
    private double integral = 0;
    private double lastError = 0;

    private int turretZeroTicks = 0;

    // Tag selection (-1 = any)
    private int trackedTagId = -1;

    /* ===================== INIT ===================== */

    public void init(HardwareMap hardwareMap) {

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        turret = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turret.setDirection(DcMotorEx.Direction.REVERSE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        turretZeroTicks = 0;

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTag
        );
    }

    /* ===================== TAG SELECTION ===================== */

    public void setTrackedTagId(int id) {
        trackedTagId = id;
    }

    /* ===================== APRILTAG TURRET TRACKING ===================== */

    public void updateTurretFromAprilTag() {

        AprilTagDetection tag = getTrackedTag();

        double currentAngle = getTurretAngleDeg();
        double targetAngle = 0;

        if (tag != null) {
            targetAngle = Range.clip(
                    currentAngle - tag.ftcPose.yaw,
                    TURRET_LEFT_LIMIT_DEG,
                    TURRET_RIGHT_LIMIT_DEG
            );
        }

        // If tag lost → return to center (0 deg)
        double error = targetAngle - currentAngle;

        if (Math.abs(error) < TURRET_DEADBAND_DEG) {
            turret.setPower(0);
            return;
        }

        double power = Range.clip(error * TURRET_KP, -TURRET_MAX_POWER, TURRET_MAX_POWER);

        // Prevent pushing past limits
        if ((currentAngle <= TURRET_LEFT_LIMIT_DEG && power < 0) ||
                (currentAngle >= TURRET_RIGHT_LIMIT_DEG && power > 0)) {
            power = 0;
        }

        turret.setPower(power);
    }

    private double getTurretAngleDeg() {
        return (turret.getCurrentPosition() - turretZeroTicks) * DEGREES_PER_TICK;
    }

    /* ===================== APRILTAG AUTO RPM ===================== */

    public boolean updateFlywheelFromAprilTag() {

        AprilTagDetection tag = getTrackedTag();
        if (tag == null) {
            stopFlywheel();
            return false;
        }

        setFlywheelRPM(distanceToRPM(tag.ftcPose.range));
        return true;
    }

    /* ===================== TAG FILTER ===================== */

    private AprilTagDetection getTrackedTag() {

        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection tag : detections) {
            if (trackedTagId == -1 || tag.id == trackedTagId) {
                return tag;
            }
        }
        return null;
    }

    /* ===================== FLYWHEEL ===================== */

    public void setFlywheelRPM(double rpm) {
        targetRPM = rpm;
    }

    public void stopFlywheel() {
        targetRPM = 0;
        flywheel.setPower(0);
    }

    public double getCurrentRPM() {
        return (flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV) * 60.0;
    }

    public void updateFlywheel() {

        if (targetRPM == 0) {
            flywheel.setPower(0);
            integral = 0;
            lastError = 0;
            return;
        }

        double error = targetRPM - getCurrentRPM();
        integral = Range.clip(integral + error, -1500, 1500);

        double output =
                (kF * targetRPM) +
                        (kP * error) +
                        (kI * integral) +
                        (kD * (error - lastError));

        lastError = error;
        flywheel.setPower(Range.clip(output, -1, 1));
    }

    /* ===================== HELPERS ===================== */

    private double distanceToRPM(double distanceInches) {
        return Range.clip(150.0 * distanceInches + 2100.0, 2500, 6000);
    }

    public void closeVision() {
        if (visionPortal != null) visionPortal.close();
    }
}
