package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OTOSDriveSubsystem {

    // === Motors ===
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;


    // === OTOS ===
    private SparkFunOTOS otos;

    // === Tuning ===
    private static final double kP_TRANSLATION = 0.04;
    private static final double kP_ROTATION = 1.2;

    private static final double MAX_TRANSLATION_POWER = 0.6;
    private static final double MAX_ROTATION_POWER = 0.5;

    private static final double POSITION_TOLERANCE = 0.25; // inches
    private static final double HEADING_TOLERANCE = Math.toRadians(2);

    private boolean forwardActive = false;
    private double forwardTargetX;
    private double forwardTargetY;
    private double forwardTargetHeading;

    public OTOSDriveSubsystem(HardwareMap hardwareMap) {


        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        for (DcMotorEx motor : new DcMotorEx[]{
                frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.resetTracking();
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
    }

    // ================== CORE PATHING ==========================
    public boolean goToPoint(double targetX,
                             double targetY,
                             double targetHeading) {


        SparkFunOTOS.Pose2D pose = getPose();


        double currentX = pose.y;
        double currentY = pose.x;   //flip if needed for y


        double errorX = targetX - currentX;
        double errorY = targetY - currentY;


        double distance = Math.hypot(errorX, errorY);
        double headingError = angleWrap(targetHeading - pose.h);


        // Stop condition
        if (distance < POSITION_TOLERANCE &&
                Math.abs(headingError) < HEADING_TOLERANCE) {
            stop();
            return true;
        }


        double cos = Math.cos(pose.h);
        double sin = Math.sin(pose.h);


        double robotX = -errorX * cos + errorY * sin;
        double robotY = -errorX * sin + errorY * cos;


        // === Proportional control ===
        double driveX = robotX * kP_TRANSLATION;
        double driveY = robotY * kP_TRANSLATION;
        double turn   = headingError * kP_ROTATION;


        // Clamp
        driveX = clamp(driveX, -MAX_TRANSLATION_POWER, MAX_TRANSLATION_POWER);
        driveY = clamp(driveY, -MAX_TRANSLATION_POWER, MAX_TRANSLATION_POWER);
        turn   = clamp(turn,   -MAX_ROTATION_POWER,   MAX_ROTATION_POWER);

;
        driveRobotCentric(driveY, driveX, turn);


        return false;
    }


    /*
     * Move forward relative to current heading by N inches.
     * Non-blocking. Call repeatedly in loop.
     */
    public boolean moveForward(double inches) {


        // Initialize target ONCE
        if (!forwardActive) {


            SparkFunOTOS.Pose2D pose = getPose();


            forwardTargetX = pose.x;
            forwardTargetY = pose.y + inches * Math.cos(pose.h);
            forwardTargetHeading = pose.h;


            forwardActive = true;
        }


        boolean done = goToPoint(
                forwardTargetX,
                forwardTargetY,
                forwardTargetHeading
        );


        if (done) {
            forwardActive = false;
        }


        return done;
    }




    // ================== DRIVE METHODS ========================


    public void driveRobotCentric(double y, double x, double rx) {


        double frontLeftPower  = y + x + rx;
        double backLeftPower   = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower  = y + x - rx;


        double max = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
        );


        if (max > 1.0) {
            frontLeftPower  /= max;
            backLeftPower   /= max;
            frontRightPower /= max;
            backRightPower  /= max;
        }


        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }


    // ================== UTILITIES =============================
    public void resetPose(double x, double y, double heading) {
        otos.setPosition(new SparkFunOTOS.Pose2D(-y, -x, -heading));
    }


    private SparkFunOTOS.Pose2D getPose() {
        SparkFunOTOS.Pose2D raw = otos.getPosition();


        return new SparkFunOTOS.Pose2D(
                -raw.y,     // swap
                -raw.x,     // swap
                -raw.h     // invert heading
        );
    }




    public double getHeading() {
        return -getPose().h;
    }


    public double getX() {
        return -getPose().x;
    }


    public double getY() {
        return -getPose().y;
    }


    public void stop() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }


    private double angleWrap(double radians) {
        while (radians > Math.PI)  radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }


    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

