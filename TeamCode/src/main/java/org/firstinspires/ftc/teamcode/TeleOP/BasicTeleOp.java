package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.Commands.AprilTagDriveSubsystem;

@TeleOp(name = "BasicTeleOp")
public class BasicTeleOp extends OpMode {

    private BasicMecanumDrive drive;
    private AprilTagDriveSubsystem aprilTagDrive;

    @Override
    public void init() {

        // Init Basic Mecanum Drive System
        drive = new BasicMecanumDrive(hardwareMap);

        // Init AprilTag Subsystem
        aprilTagDrive = new AprilTagDriveSubsystem(hardwareMap, telemetry);

    }

    @Override
    public void loop() {

        // Manual stick values
        double y  = -gamepad1.left_stick_y / 2.0; // forward/back (reduced for smoother control) (abs(left_stick_y) * left_stick_y (Potential for smoother movement (ramp)))
        double x  = gamepad1.left_stick_x / 2.0; // strafe
        double rx = gamepad1.right_stick_x / 3.0; // rotate

        // If right bumper held → AprilTag auto mode
        if (gamepad1.right_bumper) {

            aprilTagDrive.driveToTag(true, y, x, rx);

        } else {

            // Otherwise use manual mecanum subsystem
            drive.drive(y, x, rx);

        }
    }
}
