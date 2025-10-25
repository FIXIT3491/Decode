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

    /** Resets the field-centric heading to the current IMU angle */
    public void resetHeading() {
        headingOffset = imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS
        ).firstAngle;
    }

    /** Field-centric drive method */
    public void drive(double y, double x, double rx) {
        // Get current heading
        double botHeading = getHeading();

        // Rotate the joystick input for field-centric control
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Compute motor powers
        double frontLeftPower  = rotY + rotX + rx;
        double backLeftPower   = rotY - rotX + rx;
        double frontRightPower = rotY - rotX - rx;
        double backRightPower  = rotY + rotX - rx;

        // Normalize
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

        // Apply power
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }
}
