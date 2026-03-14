package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OTOSDriveSubsystem {

    //CONSTANTS
    private static final double SPARKFUN_SPEED_GAIN = 0.04;
    private static final double SPARKFUN_STRAFE_GAIN = 0.20;
    private static final double SPARKFUN_TURN_GAIN = 0.05;
    private static final double SPARKFUN_MAX_AUTO_SPEED = 0.4;
    private static final double SPARKFUN_MAX_AUTO_STRAFE = 0.4;
    private static final double SPARKFUN_MAX_AUTO_TURN = 0.3;


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
    private LinearOpMode opMode_ref = null;

    public OTOSDriveSubsystem(HardwareMap hardwareMap, LinearOpMode op) {
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx motor : new DcMotorEx[]{
                frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.resetTracking();
        configureOtos();

        opMode_ref = op;
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

    public void otosDrive(double targetX, double targetY, double targetHeading) {
        double drive, strafe, turn;
        double currentRange, targetRange, initialBearing, targetBearing, xError, yError, yawError;
        double opp, adj;
        boolean isDone = false;

        //Pose2D currentPos = myPosition();
        /*xError = targetX - currentPos.x;
        yError = targetY - currentPos.y;
        yawError = targetHeading - currentPos.h;*/
        xError = targetX - getX();
        yError = targetY - getY();
        yawError = targetHeading - getHeading();

        while(opMode_ref.opModeIsActive() && !isDone) {
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(yError * SPARKFUN_SPEED_GAIN, -SPARKFUN_MAX_AUTO_SPEED, SPARKFUN_MAX_AUTO_SPEED);
            strafe = Range.clip(xError * SPARKFUN_STRAFE_GAIN, -SPARKFUN_MAX_AUTO_STRAFE, SPARKFUN_MAX_AUTO_STRAFE);
            turn   = -Range.clip(yawError * SPARKFUN_TURN_GAIN, -SPARKFUN_MAX_AUTO_TURN, SPARKFUN_MAX_AUTO_TURN);

            RobotLog.i("RyanTag5 x = %f, y = %f, heading = %f, Ex = %f, Ey = %f, Eyaw = %f", getX(), getY(), getHeading(), xError, yError, yawError);
            //RobotLog.i("x = " + (double) getX());
            opMode_ref.telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            // current x,y swapped due to 90 degree rotation
            opMode_ref.telemetry.addData("current X coordinate", getX());
            opMode_ref.telemetry.addData("current Y coordinate", getX());
            opMode_ref.telemetry.addData("current Heading angle", getHeading());
            opMode_ref.telemetry.addData("target X coordinate", targetX);
            opMode_ref.telemetry.addData("target Y coordinate", targetY);
            opMode_ref.telemetry.addData("target Heading angle", targetHeading);
            opMode_ref.telemetry.addData("xError", xError);
            opMode_ref.telemetry.addData("yError", yError);
            opMode_ref.telemetry.addData("yawError", yawError);
            opMode_ref.telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobotSparkfun(drive, strafe, turn);

            // then recalc error
            //currentPos = myPosition();
            xError = targetX - getX();
            yError = targetY - getY();
            yawError = targetHeading- getHeading();

            if ((Math.abs(xError) < 0.87) && (Math.abs(yError) < 0.75) && (Math.abs(yawError) < 4)) {
                isDone = true;
            }
        }
        moveRobotSparkfun(0,0,0);
        //currentPos = myPosition();
        opMode_ref.telemetry.addData("current X coordinate", getX());
        opMode_ref.telemetry.addData("current Y coordinate", getY());
        opMode_ref.telemetry.addData("current Heading angle", getHeading());
        opMode_ref.telemetry.update();
    }
    Pose2D myPosition() {
        SparkFunOTOS.Pose2D pos = otos.getPosition();
        Pose2D myPos = new Pose2D(pos.y, pos.x, -pos.h);
        return(myPos);
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

    public void moveRobotSparkfun(double x, double y, double yaw) {

        // Calculate wheel powers.
        double leftFrontPower    =  x +y +yaw;
        double rightFrontPower   =  x -y -yaw;
        double leftBackPower     =  x -y +yaw;
        double rightBackPower    =  x +y -yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
        opMode_ref.sleep(10);
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
    }

    // ================== UTILITIES =============================
    public void resetPose(double x, double y, double heading) {
        otos.setPosition(new SparkFunOTOS.Pose2D(y, x, heading));
    }

    private SparkFunOTOS.Pose2D getPose() {
        SparkFunOTOS.Pose2D raw = otos.getPosition();


        return new SparkFunOTOS.Pose2D(
                raw.y,     // swap
                raw.x,     // swap
                raw.h     // invert heading
        );
    }

    public static class Pose2D {
        public double x;
        public double y;
        public double h;

        public Pose2D() {
            x = 0.0;
            y = 0.0;
            h = 0.0;
        }

        public Pose2D(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }

        public void set(Pose2D pose) {
            this.x = pose.x;
            this.y = pose.y;
            this.h = pose.h;
        }
    }

    public double getHeading() { return getPose().h; }
    public double getX() { return -getPose().x; }
    public double getY() { return getPose().y; }

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

    private void configureOtos() {

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 3.71875, 0);
        otos.setOffset(offset);

        otos.setLinearScalar(0.993095997);//0.993095997
        otos.setAngularScalar(0.994240643);

        otos.calibrateImu();

        otos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);
    }
}

