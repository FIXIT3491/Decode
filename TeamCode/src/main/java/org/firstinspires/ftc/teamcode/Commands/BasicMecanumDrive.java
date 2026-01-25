package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class BasicMecanumDrive {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private BNO055IMU imu;

    // Used to zero field-centric heading
    private double headingOffset = 0.0;

    public BasicMecanumDrive(HardwareMap hardwareMap) {

        // Initialize motors
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    /**
     * Returns robot heading in RADIANS.
     * 0 rad = field forward.
     */
    public double getHeading() {
        Orientation orientation = imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.RADIANS
        );

        double rawHeading = orientation.firstAngle;

        // Apply hub mounting correction + field reset offset
        return (rawHeading) - headingOffset;
    }

    /**
     * Returns robot heading in DEGREES.
     */
    public double getHeadingDegrees() {
        return Math.toDegrees(getHeading());
    }

    /**
     * Resets field-centric forward to current robot direction.
     */
    public void resetHeading() {
        Orientation orientation = imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.RADIANS
        );

        headingOffset = orientation.firstAngle;
    }

    /**
     * Normalize angle to [-180, 180) degrees
     */
    private double normalizeAngle(double angleDeg) {
        while (angleDeg > 180) angleDeg -= 360;
        while (angleDeg <= -180) angleDeg += 360;
        return angleDeg;
    }

    /**
     * Turn robot to a specific heading using P control.
     */
    public void turnToHeading(double targetHeadingDeg, double maxPower, double timeoutSec) {

        double kP = 0.015;
        double toleranceDeg = 1.5;
        double minPower = 0.06;

        long startTime = System.currentTimeMillis();
        long timeoutMs = (long) (timeoutSec * 1000);

        double error = normalizeAngle(targetHeadingDeg - getHeadingDegrees());

        while (Math.abs(error) > toleranceDeg &&
                (System.currentTimeMillis() - startTime) < timeoutMs) {

            error = normalizeAngle(targetHeadingDeg - getHeadingDegrees());

            double turnPower = error * kP;
            turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));

            if (Math.abs(turnPower) < minPower) {
                turnPower = Math.signum(turnPower) * minPower;
            }

            frontLeft.setPower(turnPower);
            backLeft.setPower(turnPower);
            frontRight.setPower(-turnPower);
            backRight.setPower(-turnPower);

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        stopMotors();
    }

    /**
     * Field-centric mecanum drive.
     */
    public void drive(double y, double x, double rx) {

        double heading = getHeading(); // radians, already corrected

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        // Optional strafe correction
        rotX *= 1.1;

        double denominator = Math.max(
                Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0
        );

        double frontLeftPower  = (rotY + rotX + rx) / denominator;
        double backLeftPower   = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower  = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void stopMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    public void brake() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopMotors();
    }

    public void floatMotors() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
