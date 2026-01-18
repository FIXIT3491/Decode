package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Commands.Launcher;
import org.firstinspires.ftc.teamcode.Commands.WheelRotation;

@TeleOp(group = "Test", name = "TransferMechanismTest")
public class TransferMechanismTest extends OpMode {

    private Servo kick;

    private TouchSensor touchSensor;

    //private DcMotor intake;
    private DcMotor ferrisMotor;

    private double intakeCounter = 0;
    private double outtakeCounter = 0;

    boolean prevLeftBumper = false;
    boolean prevRightBumper = false;

    //Launcher launcher = new Launcher();
    private WheelRotation wheel = new WheelRotation();

    @Override
    public void init() {
        //kick = hardwareMap.get(Servo.class, "kick");
        ferrisMotor = hardwareMap.get(DcMotor.class, "ferrisWheel"); //motor teeth: 9 | spindex teeth: 14
        //touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
        //intake = hardwareMap.get(DcMotor.class, "intake");

        //kick.setPosition(0);

        //launcher.init(hardwareMap);
        wheel.init(hardwareMap);


        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        // Left bumper cycle
        boolean leftBumperPressed = gamepad1.left_bumper;

        if (leftBumperPressed && !prevLeftBumper) {     // rising edge
            outtakeCounter = (outtakeCounter + 1) % 3;  // cycle 0 -> 1 -> 2 -> 0

            switch((int)outtakeCounter) {
                case 0: wheel.rotateToAngle(0, 0.4); break;
                case 1: wheel.rotateToAngle(110, 0.4); break;
                case 2: wheel.rotateToAngle(220, 0.4); break;
            }

        }

        prevLeftBumper = leftBumperPressed;

        // Right bumper cycle
        boolean rightBumperPressed = gamepad1.right_bumper;

        if (rightBumperPressed && !prevRightBumper) {   // rising edge
            intakeCounter = (intakeCounter + 1) % 3;    // cycle 0 -> 1 -> 2 -> 0

            switch((int)intakeCounter) {
                case 0: wheel.rotateToAngle(60, 0.4); break;
                case 1: wheel.rotateToAngle(170, 0.4); break;
                case 2: wheel.rotateToAngle(280, 0.4); break;
            }

        }

        prevRightBumper = rightBumperPressed;

        // Ball kick
        if (gamepad1.b) {

           // kick.setPosition(0.25);

        } else {

            //kick.setPosition(0);

        }

        /*/intake
        intake.setPower(gamepad1.right_trigger);
        intake.setPower(-gamepad1.left_trigger);

        *///flywheel
        if (gamepad1.y) { //far

            //launcher.setFlywheelRPM(3100); //0.85 power prev

        } else if (gamepad1.x){ //close

           // launcher.setFlywheelRPM(2350); //0.67 power prev

        } else { // stop

           // launcher.stop();

        }

       // launcher.updateFlywheels();

/*
        if (touchSensor.isPressed()) {
            // Reset the motor encoder values to zero
            ferrisMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ferrisMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ferrisMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("Touch Sensor: ", "Pressed");
        } else {
            telemetry.addData("Touch Sensor: ", "Not Pressed");
        }

        telemetry.addData("Ticks: ", ferrisMotor.getCurrentPosition());
        telemetry.addData("Outtake Counter: ", outtakeCounter);
        telemetry.addData("Intake Counter: ", intakeCounter);
        telemetry.update(); */
    }

}
