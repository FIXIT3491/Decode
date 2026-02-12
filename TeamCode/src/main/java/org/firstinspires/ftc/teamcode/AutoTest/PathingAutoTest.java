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

            switch (state) {

                case 0:
                    // Move forward 24 inches
                    if (drive.goToPoint(24, 0, 0)) {
                        state++;
                        sleep(500);
                    }
                    break;

                case 1:
                    // Strafe right 24 inches
                    if (drive.goToPoint(24, 24, 0)) {
                        state++;
                        sleep(500);
                    }
                    break;

                case 2:
                    // Rotate 90 degrees
                    if (drive.goToPoint(24, 24, Math.toRadians(90))) {
                        state++;
                        sleep(500);
                    }
                    break;

                case 3:
                    // Drive back to origin while rotated
                    if (drive.goToPoint(0, 0, Math.toRadians(90))) {
                        state++;
                        sleep(500);
                    }
                    break;

                case 4:
                    // Rotate back to 0 degrees
                    if (drive.goToPoint(0, 0, 0)) {
                        state++;
                        sleep(500);
                    }
                    break;

                default:
                    drive.stop();
                    telemetry.addLine("Test Complete");
                    telemetry.update();
                    return;
            }

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
