package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

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

    /** Returns the robot's current heading in radians (adjusted for offset) */
    public double getHeading() {
        double heading = imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS
        ).firstAngle;
        return heading - headingOffset;
    }

    /** Converts radians to degrees for convenience */
    private double getHeadingDegrees() {
        return Math.toDegrees(getHeading());
    }

    /** Resets the field-centric heading to the current IMU angle */
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

    /**
     * Turns the robot to a specific heading (degrees) using the IMU.
     * The robot automatically stops once it reaches the target.
     *
     * @param targetHeading The desired heading in degrees (-180 to 180)
     * @param maxPower The maximum turning power (0.0–1.0)
     */
    public void turnToHeading(double targetHeading, double maxPower) { // FOR FUTURE -> MAKE IT DO A CONSTANT CHECK OF HEADING SO IT STOPS WHEN IT REACHES IT
        double kP = 0.015; // proportional gain (adjust if needed)
        double tolerance = 3.0; // acceptable error in degrees
        double error = normalizeAngle((targetHeading - getHeading()));


        // Continue turning until within tolerance
        while (Math.abs(error) > tolerance) {
            error = normalizeAngle((targetHeading - getHeading()));
            double turnPower = error * kP;


            // Limit power to ±maxPower
            if (turnPower > maxPower) turnPower = maxPower;
            if (turnPower < -maxPower) turnPower = -maxPower;


            // Apply turning power
            frontLeft.setPower(turnPower);
            backLeft.setPower(turnPower);
            frontRight.setPower(-turnPower);
            backRight.setPower(-turnPower);
        }


        // Stop all motors when target reached
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }


    /** Field-centric drive control */
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

    /** Stops all motors */
    private void stopMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

}
