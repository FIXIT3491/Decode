package org.firstinspires.ftc.teamcode.AutoTest;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.AprilTag;
import org.firstinspires.ftc.teamcode.Commands.AprilTagGlobals;

@Autonomous
public class AutoWithAprilTag extends LinearOpMode {

    AprilTag tagScanner = new AprilTag();

    @Override
    public void runOpMode() {

        tagScanner.init(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            tagScanner.scan();
            telemetry.addData("Detected Tag", AprilTagGlobals.detectedTagID);
            telemetry.update();
        }

        waitForStart();

        if (AprilTagGlobals.detectedTagID == 1) {
            // auto 1
        } else if (AprilTagGlobals.detectedTagID == 2) {
            // auto 2
        } else if (AprilTagGlobals.detectedTagID == 3) {
            // auto 3
        }

        tagScanner.stop();
    }
}
