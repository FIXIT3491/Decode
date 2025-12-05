package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Commands.Launcher;
import org.firstinspires.ftc.teamcode.Commands.WheelRotation;

//TODO: Integrate it into the launcher subsystem

@TeleOp(group = "Test", name = "TransferMechanismTest")
public class TransferMechanismTest extends OpMode {

    private Servo kick;
    private DcMotor intake;

    private double intakeCounter = 0;
    private double outtakeCounter = 0;

    boolean prevLeftBumper = false;
    boolean prevRightBumper = false;


    Launcher launcher = new Launcher();
    private WheelRotation wheel = new WheelRotation();


    @Override
    public void init() {
        kick = hardwareMap.get(Servo.class, "kick");
        intake = hardwareMap.get(DcMotor.class, "intake");

        kick.setPosition(0);

        launcher.init(hardwareMap);
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
                case 0: wheel.rotateToAngle(0, 1); break;
                case 1: wheel.rotateToAngle(110, 1); break;
                case 2: wheel.rotateToAngle(220, 1); break;
            }

        }

        prevLeftBumper = leftBumperPressed;

        // Right bumper cycle
        boolean rightBumperPressed = gamepad1.right_bumper;

        if (rightBumperPressed && !prevRightBumper) {   // rising edge
            intakeCounter = (intakeCounter + 1) % 3;    // cycle 0 -> 1 -> 2 -> 0

            switch((int)intakeCounter) {
                case 0: wheel.rotateToAngle(60, 1); break;
                case 1: wheel.rotateToAngle(170, 1); break;
                case 2: wheel.rotateToAngle(280, 1); break;
            }

        }

        prevRightBumper = rightBumperPressed;

        // Ball kick
        if (gamepad1.b) {

            kick.setPosition(0.2);

        } else {

            kick.setPosition(0);

        }

        //intake
        if (gamepad1.right_trigger > 0) {

            intake.setPower(1);

        } else {

            intake.setPower(0);

        }

        if (gamepad1.y) { //far

            launcher.setFlywheelRPM(3100); //0.85 power prev

        } else if (gamepad1.x){ //close

            launcher.setFlywheelRPM(2350); //0.67 power prev

        } else { // stop

            launcher.stop();

        }

        launcher.updateFlywheels();
        telemetry.addData("Outtake Counter: ", outtakeCounter);
        telemetry.addData("Intake Counter: ", intakeCounter);
        telemetry.update();
    }
}
