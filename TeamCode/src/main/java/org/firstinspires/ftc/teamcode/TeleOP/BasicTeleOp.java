package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;
//import org.firstinspires.ftc.teamcode.Commands.AprilTagDriveSubsystem;
import org.firstinspires.ftc.teamcode.Commands.Launcher;
import org.firstinspires.ftc.teamcode.Commands.WheelRotation;

@TeleOp(name = "BasicTeleOp")
public class BasicTeleOp extends OpMode {

    //private DcMotorEx leftFlywheel;
    //private DcMotorEx rightFlywheel;
    private BasicMecanumDrive drive;
    //private AprilTagDriveSubsystem aprilTagDrive;
    Launcher launcher = new Launcher();
    //private DcMotor intake;
    private WheelRotation wheel = new WheelRotation();

    private double intakeCounter = 0;
    private double outtakeCounter = 0;
    private int offset = 0;
    private BNO055IMU imu;
    private DcMotor intake;
    private DcMotor ferrisMotor;
    private TouchSensor touchSensor;
    boolean prevLeftBumper = false;
    boolean prevRightBumper = false;
    boolean prevA = false;
    boolean prevB = false;

    private Servo kick;
    private Servo kick2;

    private ElapsedTime timer = new ElapsedTime();
    private int state = 0;

    private boolean lastA = false;

    // positions (adjust as needed)
    private final double KICK_UP = 0.6;
    private final double KICK_DOWN = 0.3;
    private final double KICK2_UP = 1;
    private final double KICK2_DOWN = 0.5;

    @Override
    public void init() {

        // Init Basic Mecanum Drive System
        drive = new BasicMecanumDrive(hardwareMap);

        // Init AprilTag Subsystem
        //aprilTagDrive = new AprilTagDriveSubsystem(hardwareMap, telemetry);

        // Init Launcher Subsystem
        launcher.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //leftFlywheel = hardwareMap.get(DcMotorEx.class, "flyLeft");
        //rightFlywheel = hardwareMap.get(DcMotorEx.class, "flyRight");

        kick = hardwareMap.get(Servo.class, "kick");
        kick2 = hardwareMap.get(Servo.class, "kick2");
        kick2.setDirection(Servo.Direction.REVERSE);

        kick.setPosition(KICK_DOWN);
        kick2.setPosition(KICK2_DOWN);

        intake = hardwareMap.get(DcMotor.class, "intake");

        wheel.init(hardwareMap);

        kick.setPosition(KICK_DOWN);
    }

    @Override
    public void loop() {

        // Manual stick values
        double y = -gamepad1.left_stick_y; // forward/back (abs(left_stick_y) * left_stick_y (Potential for smoother movement (ramp)))
        double x = -gamepad1.left_stick_x; // strafe
        double rx = gamepad1.right_stick_x; // rotate

        if (gamepad1.dpad_left) {

            y = -gamepad1.left_stick_y / 2;
            x = -gamepad1.left_stick_x / 2;
            rx = gamepad1.right_stick_x / 3;

        }

        drive.drive(y, x, rx);

        // Reset IMU heading with back button
        if (gamepad1.back) {

            drive.resetHeading();

        }

        // Left bumper cycle
        boolean leftBumperPressed = gamepad1.left_bumper;

        if (leftBumperPressed && !prevLeftBumper) {     // rising edge
            outtakeCounter = (outtakeCounter + 1) % 3;  // cycle 0 -> 1 -> 2 -> 0

            switch((int)outtakeCounter) {
                case 0: wheel.rotateToAngle(0, 0.8); break;
                case 1: wheel.rotateToAngle(110, 0.8); break;
                case 2: wheel.rotateToAngle(220, 0.8); break;
            }

        }

        prevLeftBumper = leftBumperPressed;

        // Right bumper cycle
        boolean rightBumperPressed = gamepad1.right_bumper;

        if (rightBumperPressed && !prevRightBumper) {   // rising edge
            intakeCounter = (intakeCounter + 1) % 3;    // cycle 0 -> 1 -> 2 -> 0

            switch((int)intakeCounter) {
                case 0: wheel.rotateToAngle(60, 0.8); break;
                case 1: wheel.rotateToAngle(170, 0.8); break;
                case 2: wheel.rotateToAngle(280, 0.8); break;
            }

        }

        prevRightBumper = rightBumperPressed;

        /*adjustment for ferris wheel
        if (gamepad1.a) {

            wheel.adjustWheel(1);

        } else if (gamepad1.b) {

            wheel.adjustWheel(-1);

        } */

        if (gamepad2.y) { //far

            launcher.setFlywheelRPM(3100); //0.85 power prev

        } else if (gamepad2.x){ //close

            launcher.setFlywheelRPM(2400); //0.67 power prev

        } else {

           launcher.stopFlywheel();

        }

        boolean aPressed = gamepad2.a;

        // Rising-edge detection (debounce)
        if (aPressed && !lastA && state == 0) {
            kick.setPosition(KICK_UP);
            timer.reset();
            state = 1;
        }

        // After 0.5s, spin kick2 up
        if (state == 1 && timer.seconds() > 0.5) {
            kick2.setPosition(KICK2_UP);
            timer.reset();
            state = 2;
        }

        // After another 0.5s, return both
        if (state == 2 && timer.seconds() > 1.5) {
            kick.setPosition(KICK_DOWN);
            kick2.setPosition(KICK2_DOWN);
            state = 0;
        }

        // Save button state for next loop
        lastA = aPressed;

        //intake
        intake.setPower(gamepad2.right_trigger);
        intake.setPower(-gamepad2.left_trigger);

        launcher.updateFlywheel();
        /*
        if (touchSensor.isPressed()) {
            // Reset the motor encoder values to zero
            ferrisMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); unless you change the way it spins dont involve this
            ferrisMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ferrisMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("Touch Sensor: ", "Pressed");
        } else {
            telemetry.addData("Touch Sensor: ", "Not Pressed");
        }*/

        telemetry.addData("Outtake Counter: ", outtakeCounter);
        telemetry.addData("Intake Counter: ", intakeCounter);
        telemetry.addData("Current Degree: ", wheel.getCurrentDegrees());
        telemetry.update();

    }
}
