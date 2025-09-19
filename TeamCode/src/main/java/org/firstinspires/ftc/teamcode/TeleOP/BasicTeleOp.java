package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;

@TeleOp(name="BasicTeleOp")
public class BasicTeleOp extends OpMode {

    private BasicMecanumDrive drive;

    @Override
    public void init() {
        drive = new BasicMecanumDrive(hardwareMap);
    }

    @Override
    public void loop() {
        double y  = -gamepad1.left_stick_y; // forward/back
        double x  = gamepad1.left_stick_x;  // strafe
        double rx = gamepad1.right_stick_x; // rotate

        // Call drive method from helper
        drive.drive(y, x, rx);
    }
}
