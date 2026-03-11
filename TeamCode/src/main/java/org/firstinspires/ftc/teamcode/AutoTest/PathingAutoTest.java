package org.firstinspires.ftc.teamcode.AutoTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Commands.OTOSDriveSubsystem;

@Autonomous(name = "OTOS Drive Test Auto", group = "Test")
public class PathingAutoTest extends LinearOpMode {

    private OTOSDriveSubsystem drive;

    @Override
    public void runOpMode() {

        drive = new OTOSDriveSubsystem(hardwareMap, this);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();
        RobotLog.i("Ryan: registerOpMode called");

        // Reset pose to 0,0,0
        drive.resetPose(0, 0, 0);

        int state = 0;

        drive.otosDrive(0, 24, 0);
        sleep(1000);
        drive.otosDrive(12,0,0);
        sleep(1000);

        RobotLog.i("MyFTCTag", "OpMode stopped.");

    }
}
