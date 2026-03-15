package org.firstinspires.ftc.teamcode.Commands;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

public class Launcher {

    /* ===================== HARDWARE ===================== */
    private DcMotorEx flywheel;
    private DcMotorEx turret;
    private Servo hood;

    /* ===================== VISION ===================== */
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    /* ===================== CONSTANTS ===================== */

    // Flywheel encoder
    private static final double FLYWHEEL_TICKS_PER_REV = 28.0;

    // Turret encoder
    private static final double TURRET_TICKS_PER_REV = 537.7;
    private static final double TURRET_GEAR_RATIO = 2.0;
    private static final double DEGREES_PER_TICK =
            360.0 / (TURRET_TICKS_PER_REV * TURRET_GEAR_RATIO);

    private static final double TICKS_PER_DEGREE =
            (TURRET_TICKS_PER_REV * TURRET_GEAR_RATIO) / 360.0;

    // Turret limits
    private static final double TURRET_LEFT_LIMIT_DEG = -20.0;
    private static final double TURRET_RIGHT_LIMIT_DEG = 20.0;
    private static final double TURRET_DEADBAND_DEG = 4;

    // Turret control
    private static final double TURRET_KP = 0.015;
    private static final double TURRET_MAX_POWER = 0.3;
    private static final double TURRET_SLEW_RATE = 0.02;
    private static final double TURRET_KD = 0.003;
    private static final double BEARING_ALPHA = 0.2; // smoothing factor -> 0.1 is smoothest, 0.3 is the min

    // Flywheel PIDF
    private final double kF = 0.0002;
    private final double kP = 0.0006;
    private final double kI = 0.0;
    private final double kD = 0.0001;

    // Turret
    private double filteredBearing = 0;
    private double lastRawBearing = 0;
    private double lastTurretError = 0;

    /* ===================== RPM PRESETS ===================== */

    private static final double CLOSE_RPM = 3200; // <4 ft
    private static final double MID_RPM   = 3800; // 4–7 ft
    private static final double FAR_RPM   = 4200; // >7 ft

    /* ===================== HOOD PRESETS ===================== */

    private static final double CLOSE_HOOD = 0.0;
    private static final double MID_HOOD   = 0.15;
    private static final double FAR_HOOD   = 0.25;

    /* ===================== STATE ===================== */

    private double targetRPM = 0;
    private double integral = 0;
    private double lastError = 0;

    private int turretZeroTicks = 0;
    private double lastTurretPower = 0;

    private int trackedTagId = -1;

    private ElapsedTime timer = new ElapsedTime();

    private double lastTime = 0;

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

        turretZeroTicks = turret.getCurrentPosition();

        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(FAR_HOOD);

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

    /* ===================== TURRET TRACKING ===================== */
    public void setTargetForTurret() {

        AprilTagDetection tag = getTrackedTag();
        if (tag == null) return;

        double bearingDeg = tag.ftcPose.bearing;

        int ticksToMove = (int)(bearingDeg * TICKS_PER_DEGREE);

        int currentPos = turret.getCurrentPosition();
        int targetPos = currentPos + ticksToMove;

        double targetAngle = getTurretAngleDeg() + bearingDeg;

        if (targetAngle < TURRET_LEFT_LIMIT_DEG || targetAngle > TURRET_RIGHT_LIMIT_DEG) {
            return;
        }

        turret.setTargetPosition(targetPos);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setPower(0.3);

        while (turret.isBusy()) {
            sleep(10);
        }

        turret.setPower(0);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void updateTurretFromAprilTag() {

        AprilTagDetection tag = getTrackedTag();
        double currentAngle = getTurretAngleDeg();

        if (tag == null) {
            applyTurretPower(0);
            return;
        }

        // Horizontal aiming
        filteredBearing =
                BEARING_ALPHA * tag.ftcPose.bearing +
                        (1 - BEARING_ALPHA) * filteredBearing;

        double error = -filteredBearing;

        double rawBearing = tag.ftcPose.bearing;

        // Reject vision spikes
        if (Math.abs(rawBearing - lastRawBearing) > 15) {
            rawBearing = lastRawBearing;
        }

        filteredBearing =
                BEARING_ALPHA * rawBearing +
                        (1 - BEARING_ALPHA) * filteredBearing;

        lastRawBearing = rawBearing;

        if (Math.abs(error) < TURRET_DEADBAND_DEG) {
            applyTurretPower(0);
            return;
        }

        double derivative = error - lastTurretError;

        double rawPower = Range.clip(
                (error * TURRET_KP) + (derivative * TURRET_KD),
                -TURRET_MAX_POWER,
                TURRET_MAX_POWER
        );

        lastTurretError = error;

        double smoothPower = slew(rawPower, lastTurretPower, TURRET_SLEW_RATE);
        lastTurretPower = smoothPower;

        // Respect mechanical limits
        if ((currentAngle <= TURRET_LEFT_LIMIT_DEG && smoothPower < 0) ||
                (currentAngle >= TURRET_RIGHT_LIMIT_DEG && smoothPower > 0)) {
            smoothPower = 0;
        }

        turret.setPower(smoothPower);
    }

    private void applyTurretPower(double power) {
        lastTurretPower = power;
        turret.setPower(power);
    }

    private double getTurretAngleDeg() {
        return (turret.getCurrentPosition() - turretZeroTicks) * DEGREES_PER_TICK;
    }

    /* ===================== FLYWHEEL CONTROL ===================== */

    public boolean updateFlywheelFromAprilTag() {

        AprilTagDetection tag = getTrackedTag();
        if (tag == null) {
            stopFlywheel();
            return false;
        }

        double distanceFeet = tag.ftcPose.range / 12.0;

        setFlywheelRPM(selectRPM(distanceFeet));
        updateHoodFromDistance(distanceFeet);

        return true;
    }

    private double selectRPM(double distanceFeet) {

        if (distanceFeet < 4.0) {
            return CLOSE_RPM;
        }
        else if (distanceFeet <= 7.0) {
            return MID_RPM;
        }
        else if (distanceFeet > 9.0){
            return FAR_RPM;
        }
        return targetRPM;
    }

    private void updateHoodFromDistance(double distanceFeet) {

        if (distanceFeet < 4.0) {
            hood.setPosition(CLOSE_HOOD);
        }
        else if (distanceFeet <= 7.0) {
            hood.setPosition(MID_HOOD);
        }
        else {
            hood.setPosition(FAR_HOOD);
        }
    }

    public void setHoodPosition(double pos) {
        hood.setPosition(Range.clip(pos, 0.0, 1.0));
    }

    public void setFlywheelRPM(double rpm) {
        targetRPM = rpm;
    }

    public void stopFlywheel() {
        targetRPM = 0;
        flywheel.setPower(0);
        integral = 0;
        lastError = 0;
    }

    public double getCurrentRPM() {
        return (flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV) * 60.0;
    }

    public void updateFlywheel() {

        double currentTime = timer.milliseconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        if (targetRPM == 0) {
            flywheel.setPower(0);
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

        RobotLog.i("RyanTag4 actual = %f, target = %f, dt = %f", getCurrentRPM(), targetRPM, currentTime);

        flywheel.setPower(Range.clip(output, 0, 1));
    }

    /* ===================== HELPERS ===================== */

    private AprilTagDetection getTrackedTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection tag : detections) {
            if (trackedTagId == -1 || tag.id == trackedTagId) {
                return tag;
            }
        }
        return null;
    }

    private double slew(double target, double current, double maxDelta) {
        double delta = target - current;
        if (Math.abs(delta) > maxDelta) {
            delta = Math.signum(delta) * maxDelta;
        }
        return current + delta;
    }

    public void closeVision() {
        if (visionPortal != null) visionPortal.close();
    }
}
