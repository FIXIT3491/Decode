package org.firstinspires.ftc.teamcode.AutoTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.OTOSDriveSubsystem;

@Autonomous(name = "OTOS Drive Test Auto", group = "Test")
public class PathingAutoTest extends LinearOpMode {

    private OTOSDriveSubsystem drive;

    @Override
    public void runOpMode() {

        drive = new OTOSDriveSubsystem(hardwareMap);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        // Reset pose to 0,0,0
        drive.resetPose(0, 0, 0);

        int state = 0;

        while (opModeIsActive()) {

            drive.goToPoint(24,0,0);

            // === Live Telemetry ===
            telemetry.addData("State", state);
            telemetry.addData("X", drive.getX());
            telemetry.addData("Y", drive.getY());
            telemetry.addData("Heading (deg)",
                    Math.toDegrees(drive.getHeading()));
            telemetry.update();
        }
    }
}
