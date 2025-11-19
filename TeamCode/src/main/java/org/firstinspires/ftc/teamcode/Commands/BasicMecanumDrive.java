package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.robotcore.util.ElapsedTime;

public class BasicMecanumDrive {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private BNO055IMU imu;

    private double headingOffset = 0; // For field-centric reset

    public BasicMecanumDrive(HardwareMap hardwareMap) {
        // Initialize motors
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse left side
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    /* Returns the robot's current heading in radians (adjusted for offset) */
    public double getHeading() {
        double heading = imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS
        ).firstAngle;
        return heading - headingOffset;
    }

    /* Converts radians to degrees for convenience */
    private double getHeadingDegrees() {
        return Math.toDegrees(getHeading());
    }

    /* Resets the field-centric heading to the current IMU angle */
    public void resetHeading() {
        headingOffset = imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS
        ).firstAngle;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    /*
     * Turns the robot to a specific heading (degrees) using the IMU.
     * The robot stops once it reaches the target (within tolerance) or when timeout expires.
     *
     * @param targetHeadingDeg Desired heading in degrees (-180 to 180)
     * @param maxPower Maximum turning power (0.0–1.0)
     * @param timeoutSec Maximum time to attempt the turn (seconds)
     */
    public void turnToHeading(double targetHeadingDeg, double maxPower, double timeoutSec) {
        double kP = 0.015;              // proportional gain (tweak if needed)
        double toleranceDeg = 0.5;      // acceptable error in degrees
        double minPower = 0.06;         // minimum power to overcome static friction

        long startTime = System.currentTimeMillis();
        long timeoutMs = (long)(timeoutSec * 1000.0);

        // compute initial error in degrees (use getHeadingDegrees() which returns degrees)
        double error = normalizeAngle(targetHeadingDeg - getHeadingDegrees());

        while (Math.abs(error) > toleranceDeg && (System.currentTimeMillis() - startTime) < timeoutMs) {
            // recompute error
            error = normalizeAngle(targetHeadingDeg - getHeadingDegrees());

            // P control
            double turnPower = error * kP;

            // clamp to maxPower
            if (turnPower > maxPower) turnPower = maxPower;
            if (turnPower < -maxPower) turnPower = -maxPower;

            // ensure a minimum magnitude so motors actually move (prevent stalling)
            if (Math.abs(turnPower) < minPower) {
                turnPower = Math.signum(turnPower) * minPower;
            }

            // apply turning power (positive => turn right)
            frontLeft.setPower(turnPower);
            backLeft.setPower(turnPower);
            frontRight.setPower(-turnPower);
            backRight.setPower(-turnPower);

            // small sleep to let sensors update and to avoid hammering CPU
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        // stop motors when done or timed out
        stopMotors();
    }

    // Field-centric drive control
    public void drive(double y, double x, double rx) {
        double botHeading = getHeading();

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double frontLeftPower  = rotY + rotX + rx;
        double backLeftPower   = rotY - rotX + rx;
        double frontRightPower = rotY - rotX - rx;
        double backRightPower  = rotY + rotX - rx;

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

    // Stops all motors
    public void stopMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    //Makes the motors brake
    public void brake() {
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

    }

    //Complementary to brake motors: reset them so our motor don't brake by nature
    public void floatMotors() {

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

}
