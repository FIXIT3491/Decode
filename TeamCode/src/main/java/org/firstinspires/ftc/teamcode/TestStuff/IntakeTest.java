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
    private double intakeCounter = 0;

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

        if (gamepad1.right_trigger > 0) {
            intake.setPower(0.8);
        } else if (gamepad1.left_trigger > 0) {
            intake.setPower(-0.8);
        } else {
            intake.setPower(0);
        }

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

        wheel.updateTelemetry();

    }
}