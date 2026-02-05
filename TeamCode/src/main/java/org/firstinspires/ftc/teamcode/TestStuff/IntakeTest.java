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
    private boolean lastB = false;
    private boolean lastTrigger = false;
    boolean intakeInitialized = false;
    boolean wheelAtIdle = true;
    private double intakeCounter = 0;
    private boolean lastOuttakeTrigger = false;
    private boolean outtakeInitialized = false;
    private boolean outtakeWheelAtIdle = true;
    private int outtakeCounter = 0;

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
        boolean outtakePressed = gamepad1.left_trigger > 0.1;
        boolean aPressed = gamepad1.a;
        boolean bPressed = gamepad1.b;

        if (triggerPressed) {

            intake.setPower(0.8);

        } else if (outtakePressed){

            intake.setPower(-0.8);

        } else {

            intake.setPower(0.0);

        }

        if (triggerPressed && !lastTrigger) {

            wheel.rotateToAngle(35, 0.3);
            intakeCounter = 1;
            intakeInitialized = true;
            wheelAtIdle = false;

        }

        if (aPressed && !lastA && intakeInitialized) {
            switch ((int) intakeCounter) {
                case 0:
                    wheel.rotateToAngle(35, 0.3);
                    break;
                case 1:
                    wheel.rotateToAngle(148, 0.3);
                    break;
                case 2:
                    wheel.rotateToAngle(268, 0.3);
                    break;
            }

            intakeCounter = (intakeCounter + 1) % 3;
            wheelAtIdle = false;
        }

        if (!triggerPressed && !wheelAtIdle) {

            wheel.rotateToAngle(358, 0.3);
            wheelAtIdle = true;

            // reset for next cycle
            intakeInitialized = false;
            intakeCounter = 0;
        }

        lastTrigger = triggerPressed;
        lastA = aPressed;

        //Outtake

        if (outtakePressed && !lastOuttakeTrigger) {
            wheel.rotateToAngle(90, 0.3);
            outtakeCounter = 1;
            outtakeInitialized = true;
            outtakeWheelAtIdle = false;
        }

        if (bPressed && !lastB && outtakeInitialized) {
            switch (outtakeCounter) {
                case 0:
                    wheel.rotateToAngle(90, 0.3);
                    break;
                case 1:
                    wheel.rotateToAngle(200, 0.3);
                    break;
                case 2:
                    wheel.rotateToAngle(320, 0.3);
                    break;
            }

            outtakeCounter = (outtakeCounter + 1) % 3;
            outtakeWheelAtIdle = false;
        }

        if (!outtakePressed && !outtakeWheelAtIdle) {
            wheel.rotateToAngle(358, 0.3);
            outtakeWheelAtIdle = true;

            outtakeInitialized = false;
            outtakeCounter = 0;
        }

        lastOuttakeTrigger = outtakePressed;
        lastB = bPressed;


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