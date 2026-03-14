package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Launcher;

@TeleOp(name = "Auto Turret AprilTag Test", group = "Test")
public class TurretMethodTest extends LinearOpMode {

    Launcher launcher = new Launcher();

    @Override
    public void runOpMode() {

        launcher.init(hardwareMap);

        // Track any AprilTag
        launcher.setTrackedTagId(20);

        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            launcher.setTargetForTurret();

        }
        launcher.closeVision();
    }
}

