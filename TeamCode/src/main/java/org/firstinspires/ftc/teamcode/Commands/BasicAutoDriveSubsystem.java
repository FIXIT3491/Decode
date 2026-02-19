package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BasicAutoDriveSubsystem {

    private BasicMecanumDrive drive;

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // GoBILDA 435 RPM motor
    private static final double TICKS_PER_REV = 384.5;
    private static final double WHEEL_DIAMETER_IN = 4.09448819;
    private static final double TICKS_PER_INCH =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);

    public BasicAutoDriveSubsystem(HardwareMap hardwareMap) {

        drive = new BasicMecanumDrive(hardwareMap);

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        resetEncoders();
        setBrakeMode();
    }

    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setBrakeMode() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void runToPosition(int fl, int fr, int bl, int br, double power) {

        frontLeft.setTargetPosition(fl);
        frontRight.setTargetPosition(fr);
        backLeft.setTargetPosition(bl);
        backRight.setTargetPosition(br);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (frontLeft.isBusy() ||
                frontRight.isBusy() ||
                backLeft.isBusy() ||
                backRight.isBusy()) {
        }

        drive.stopMotors();
        resetEncoders();
    }

    public void driveForward(double inches, double power) {

        int ticks = (int)(inches * TICKS_PER_INCH);

        runToPosition(
                ticks,
                ticks,
                ticks,
                ticks,
                power
        );
    }

    public void driveBackward(double inches, double power) {
        driveForward(-inches, power);
    }

    public void strafeRight(double inches, double power) {

        int ticks = (int)(inches * TICKS_PER_INCH);

        runToPosition(
                ticks,
                -ticks,
                -ticks,
                ticks,
                power
        );
    }

    public void strafeLeft(double inches, double power) {
        strafeRight(-inches, power);
    }

    public void turnTo(double headingDeg, double maxPower, double timeoutSec) {
        drive.turnToHeading(headingDeg, maxPower, timeoutSec);
    }

    public void stop() {
        drive.stopMotors();
    }
}
