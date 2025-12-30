package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "OTOS Test", group = "SensorTests")
public class OTOS_Test extends LinearOpMode {

    private SparkFunOTOS otos;

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware Map
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        configureOtos();

        telemetry.addLine("OTOS ready.");
        telemetry.addLine("Press START to begin tracking");
        telemetry.update();

        waitForStart();

        // Reset position at start
        otos.resetTracking();

        while (opModeIsActive()) {

            SparkFunOTOS.Pose2D pos = otos.getPosition();

            // Calculate total distance from start (0,0)
            double totalDistance =
                    Math.sqrt(pos.x * pos.x + pos.y * pos.y);

            // Controls
            if (gamepad1.y) {
                otos.resetTracking();
            }

            telemetry.addLine("OTOS Distance Test");
            telemetry.addLine("Press Y to reset tracking");
            telemetry.addLine();

            telemetry.addData("X (in)", "%.2f", pos.x);
            telemetry.addData("Y (in)", "%.2f", pos.y);
            telemetry.addData("Heading (deg)", "%.1f", pos.h);
            telemetry.addData("Total Distance (in)", "%.2f", totalDistance);

            telemetry.update();
        }
    }

    private void configureOtos() {

        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Use inches and degrees
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        // Sensor offset from robot center (CHANGE THIS IF NEEDED)
        // Example: new Pose2D(-5, 10, -90);
        SparkFunOTOS.Pose2D offset =
                new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);

        // Scaling (leave at 1.0 for testing)
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        // IMU calibration (robot must be still!)
        otos.calibrateImu();

        // Reset tracking & position
        otos.resetTracking();
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

        telemetry.addLine("OTOS configured.");
        telemetry.update();
    }
}
