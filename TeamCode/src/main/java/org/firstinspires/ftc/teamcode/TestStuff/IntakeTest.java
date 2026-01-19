package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;

@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends OpMode {

    private BasicMecanumDrive drive;

    private DcMotor intake;

    @Override
    public void init() {

        // Init Basic Mecanum Drive System
        drive = new BasicMecanumDrive(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void loop() {

        // Manual stick values
        double y = -gamepad1.left_stick_y; // forward/back (abs(left_stick_y) * left_stick_y (Potential for smoother movement (ramp)))
        double x = gamepad1.left_stick_x; // strafe
        double rx = gamepad1.right_stick_x; // rotate

        drive.drive(y, x, rx);

        if (gamepad1.right_trigger > 0){intake.setPower(1);}
        if (gamepad1.left_trigger > 0){intake.setPower(-1);}

    }
}