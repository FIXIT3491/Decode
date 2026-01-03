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

    // === Pathing Tuning ===
    private static final double kP_TRANSLATION = 0.05;   // inches → power
    private static final double kP_ROTATION = 1.5;       // radians → power

    private static final double POSITION_TOLERANCE = 0.5; // inches
    private static final double HEADING_TOLERANCE =
            Math.toRadians(2);

    public OTOSDriveSubsystem(HardwareMap hardwareMap) {

        // Motor init
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

        // OTOS init
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.resetTracking();
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

    }

    // copied from BasicMecanumDrive
    public void drive(double y, double x, double rx) {

        double botHeading = getHeading();

        y = -y;
        x = -x;
        rx = -rx;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double frontLeftPower  = rotY + rotX + rx;
        double backLeftPower   = rotY - rotX + rx;
        double frontRightPower = rotY - rotX - rx;
        double backRightPower  = rotY + rotX - rx;

        double max = Math.max(
                Math.max(Math.abs(frontLeftPower),
                        Math.abs(backLeftPower)),
                Math.max(Math.abs(frontRightPower),
                        Math.abs(backRightPower))
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

    //Pathing API
    public boolean goToPoint(double targetX,
                             double targetY,
                             double targetHeading) {

        SparkFunOTOS.Pose2D pose = otos.getPosition();

        double errorX = targetX - pose.x;
        double errorY = targetY - pose.y;

        double distanceError = Math.hypot(errorX, errorY);
        double headingError =
                angleWrap(targetHeading - pose.h);

        if (distanceError < POSITION_TOLERANCE &&
                Math.abs(headingError) < HEADING_TOLERANCE) {
            stop();
            return true;
        }

        // Field-centric proportional control
        double fieldY = errorX * kP_TRANSLATION; // forward
        double fieldX = errorY * kP_TRANSLATION; // strafe
        double turn   = headingError * kP_ROTATION;

        double max = Math.max(1.0,
                Math.max(Math.abs(fieldX),
                        Math.max(Math.abs(fieldY), Math.abs(turn))));

        fieldX /= max;
        fieldY /= max;
        turn   /= max;

        drive(fieldY, fieldX, turn);
        return false;
    }

    //Util
    public double getHeading() {
        return otos.getPosition().h;
    }

    public double getX() {
        return otos.getPosition().x;
    }

    public double getY() {
        return otos.getPosition().y;
    }

    public void resetPose(double x, double y, double heading) {
        otos.setPosition(new SparkFunOTOS.Pose2D(x, y, heading));
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
}

