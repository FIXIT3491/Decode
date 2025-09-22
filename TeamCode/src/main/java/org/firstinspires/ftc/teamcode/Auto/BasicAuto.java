package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.AprilTagDriveSubsystem;

@Autonomous(name = "Basic Auto")
public class BasicAuto extends OpMode{

    private AprilTagDriveSubsystem aprilTagDrive;

    @Override
    public void init() {

        // Init AprilTag Subsystem
        aprilTagDrive = new AprilTagDriveSubsystem(hardwareMap, telemetry);

    }

    @Override
    public void loop() {

        // Manual stick values
        double y  = -gamepad1.left_stick_y / 2.0; // forward/back (reduced for smoother control)
        double x  = gamepad1.left_stick_x / 2.0; // strafe
        double rx = gamepad1.right_stick_x / 3.0; // rotate

        aprilTagDrive.driveToTag(true, y, x, rx);

    }
}
