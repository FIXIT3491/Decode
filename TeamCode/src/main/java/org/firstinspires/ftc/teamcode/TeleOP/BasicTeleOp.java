package org.firstinspires.ftc.teamcode.TeleOP;

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
    private DcMotor intake;
    private DcMotor ferrisMotor;
    private TouchSensor touchSensor;
    boolean prevLeftBumper = false;
    boolean prevRightBumper = false;
    boolean prevA = false;
    boolean prevB = false;

    @Override
    public void init() {

        // Init Basic Mecanum Drive System
        drive = new BasicMecanumDrive(hardwareMap);

        // Init AprilTag Subsystem
        //aprilTagDrive = new AprilTagDriveSubsystem(hardwareMap, telemetry);

        // Init Launcher Subsystem
        launcher.init(hardwareMap);
        telemetry.addData("Status", "Initialized");

        //leftFlywheel = hardwareMap.get(DcMotorEx.class, "flyLeft");
        //rightFlywheel = hardwareMap.get(DcMotorEx.class, "flyRight");

        intake = hardwareMap.get(DcMotor.class, "intake");
        ferrisMotor = hardwareMap.get(DcMotor.class, "ferrisWheel");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        wheel.init(hardwareMap);

        launcher.kickBack();
    }

    @Override
    public void loop() {

        // Manual stick values
        double y = -gamepad1.left_stick_y; // forward/back (abs(left_stick_y) * left_stick_y (Potential for smoother movement (ramp)))
        double x = gamepad1.left_stick_x; // strafe
        double rx = gamepad1.right_stick_x; // rotate

        if (gamepad1.dpad_left) {

            y = -gamepad1.left_stick_y / 2;
            x = gamepad1.left_stick_x / 2;
            rx = gamepad1.right_stick_x / 3;

        }

        // If right bumper held → AprilTag auto mode
        if (gamepad1.right_bumper) {

            //aprilTagDrive.driveToTag(true, y, x, rx, -1); // -1 is any tag

        } else {

            // Otherwise use manual mecanum subsystem (field centric)
            drive.drive(y, x, rx);

        }

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

        //adjustment for ferris wheel
        if (gamepad1.a) {

            wheel.adjustWheel(1);

        } else if (gamepad1.b) {

            wheel.adjustWheel(-1);

        }

        if (gamepad2.y) { //far

            launcher.setFlywheelRPM(3100); //0.85 power prev

        } else if (gamepad2.x){ //close

            launcher.setFlywheelRPM(2400); //0.67 power prev

        } else {

            launcher.stop();

        }

        if (gamepad2.b) { //open if held

            launcher.kick();

        } else { // close if released

            launcher.kickBack();

        }

        //intake
        intake.setPower(gamepad2.right_trigger);
        intake.setPower(-gamepad2.left_trigger);

        launcher.updateFlywheels();
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
