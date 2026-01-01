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

        waitForStart();

        sleep(1000);
        //Actions to read obelisk

        //scan obelisk
        while (AprilTagGlobals.detectedTagID == -1) {
            tagScanner.scan();
            telemetry.addData("Detected Tag", AprilTagGlobals.detectedTagID);
            telemetry.update();
        }

        sleep(500);

        //Auto based on obelisk
        if (AprilTagGlobals.detectedTagID == 21) { //GPP
            // auto 1
            telemetry.addData("Auto 1", null );
        } else if (AprilTagGlobals.detectedTagID == 22) { //PGP
            // auto 2
            telemetry.addData("Auto 2", null );
        } else if (AprilTagGlobals.detectedTagID == 23) { //PPG
            // auto 3
            telemetry.addData("Auto 3", null );
        }

        tagScanner.stop();

        telemetry.addData("Tag ID: ", AprilTagGlobals.detectedTagID);
    }
}
