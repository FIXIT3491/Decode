package org.firstinspires.ftc.teamcode.AutoTest;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Commands.OTOSDriveSubsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "OTOS Drive Test Auto", group = "Test")
public class PathingAutoTest extends LinearOpMode {

    final double TURN_GAIN   =  0.05;   // 0.01 Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    private ElapsedTime runtime = new ElapsedTime();

    IMU imu;
    YawPitchRollAngles orientation;

    private OTOSDriveSubsystem drive;

    @Override
    public void runOpMode() {

        imu = hardwareMap.get(IMU.class, "imu");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        drive = new OTOSDriveSubsystem(hardwareMap, this);
        //YOU NEED THIS
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        drive.configureOtos();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();
        RobotLog.i("Ryan: registerOpMode called");

        // Reset pose to 0,0,0
        drive.resetPose(0, 0, 0);

        drive.otosDrive(0,36,0);
        sleep(5000);
        imuTurn(90);
        sleep(1000);
        drive.resetPose(0,36,0);
        sleep(1000);
        drive.otosDrive(36,36, drive.getHeading());


        RobotLog.i("MyFTCTag", "OpMode stopped.");

    }

    public void imuTurn(double heading) {
        YawPitchRollAngles orientation;
        double turn, headingError;
        boolean done = false;
        runtime.reset();  // start timer
        while (opModeIsActive() && !done && runtime.milliseconds() < 2000) {
            orientation = imu.getRobotYawPitchRollAngles();
            headingError    = heading - orientation.getYaw(AngleUnit.DEGREES);
            if (Math.abs(headingError) > 3) {
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                moveRobot(0, 0, turn);}
            else done = true;
            if (runtime.milliseconds() < 2000){
                telemetry.addData("Done: ", "run out");
            }
            }
        moveRobot(0, 0, 0);
    }

    public void moveRobot(double x, double y, double yaw) {
        /* positive values of x move forward
           positive values of y move sideways to the right
           positive values of yaw rotate clockwise
        */
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
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        sleep(10);
    }

}
