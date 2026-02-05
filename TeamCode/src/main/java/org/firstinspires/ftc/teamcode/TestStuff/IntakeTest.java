package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.Launcher;
import org.firstinspires.ftc.teamcode.Commands.WheelRotation;
import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;

@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends OpMode {

    private BasicMecanumDrive drive;

    private DcMotor intake;

    private boolean lastA = false;
    private boolean lastTrigger = false;
    boolean intakeInitialized = false;

    private boolean last2Trigger = false;
    private double intakeCounter = 0;
    private double intake2Counter = 0;

    private WheelRotation wheel = new WheelRotation();

    @Override
    public void init() {

        // Init Basic Mecanum Drive System
        drive = new BasicMecanumDrive(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");

        wheel.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        // Manual stick values
        double y = -gamepad1.left_stick_y; // forward/back (abs(left_stick_y) * left_stick_y (Potential for smoother movement (ramp)))
        double x = gamepad1.left_stick_x; // strafe
        double rx = gamepad1.right_stick_x; // rotate

        drive.drive(y, x, rx);

        boolean triggerPressed = gamepad1.right_trigger > 0.1;
        boolean aPressed = gamepad1.a;

        // Intake runs continuously while trigger is held
        if (triggerPressed) {

            intake.setPower(0.8);

        } else {

            intake.setPower(0.0);
            intakeInitialized = false; // reset when trigger released
            intakeCounter = 0;         // optional: reset cycle when trigger released

        }

        // First trigger press -> run first rotateToAngle
        if (triggerPressed && !lastTrigger && !intakeInitialized) {

            wheel.rotateToAngle(30, 0.3);
            intakeCounter = 1; // next A press goes to second angle
            intakeInitialized = true;

        }

        // A button cycles after initialization
        if (aPressed && !lastA && intakeInitialized) {
            switch ((int) intakeCounter) {
                case 0:
                    wheel.rotateToAngle(30, 0.3);
                    break;
                case 1:
                    wheel.rotateToAngle(150, 0.3);
                    break;
                case 2:
                    wheel.rotateToAngle(290, 0.3);
                    break;
            }

            intakeCounter = (intakeCounter + 1) % 3;

        }

        // Save previous states
        lastTrigger = triggerPressed;
        lastA = aPressed;


/*
        while (gamepad1.right_trigger > 0) { // was an if so if it doesnt work swap back
            intake.setPower(0.8);

            boolean aPressed = gamepad1.a;

            if (aPressed && !lastA) {

                switch ((int) intakeCounter) {
                    case 0:
                        wheel.rotateToAngle(30, 0.3);
                        break;
                    case 1:
                        wheel.rotateToAngle(150, 0.3);
                        break;
                    case 2:
                        wheel.rotateToAngle(290, 0.3);
                        break;
                }

                intakeCounter = (intakeCounter + 1) % 3;

            }

            lastA = aPressed;

        }  */

        if (gamepad1.left_trigger > 0) {
            intake.setPower(-0.8);
        } else {
            intake.setPower(0);
        }

    /*
        boolean aPressed = gamepad1.a;

        if (aPressed && !lastA) {

            switch ((int) intakeCounter) {
                case 0:
                    wheel.rotateToAngle(30, 0.3);
                    break;
                case 1:
                    wheel.rotateToAngle(150, 0.3);
                    break;
                case 2:
                    wheel.rotateToAngle(290, 0.3);
                    break;
            }

            intakeCounter = (intakeCounter + 1) % 3;

        }

        lastA = aPressed; */

        wheel.updateTelemetry();

    }
}