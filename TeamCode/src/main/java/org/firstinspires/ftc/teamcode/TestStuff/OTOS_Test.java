package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;

@TeleOp(name = "OTOS Test", group = "SensorTests")
public class OTOS_Test extends LinearOpMode {

    private SparkFunOTOS otos;
    private BasicMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware Map
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        configureOtos();

        drive = new BasicMecanumDrive(hardwareMap);
        telemetry.addLine("OTOS ready.");
        telemetry.addLine("Press START to begin tracking");
        telemetry.update();

        waitForStart();

        // Reset position at start
        otos.resetTracking();

        while (opModeIsActive()) {

            if (gamepad1.back) {drive.resetHeading();}
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            drive.drive(y, x, rx);

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

            telemetry.addData("X (in)", "%.2f", -pos.y);
            telemetry.addData("Y (in)", "%.2f", -pos.x);
            telemetry.addData("Heading (deg)", "%.1f", -pos.h);
            telemetry.addData("Total Distance (in)", "%.2f", totalDistance);

            telemetry.update();
        }
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 3.71875, 180);
        otos.setOffset(offset);

        otos.setLinearScalar(0.993095997);//0.993095997
        otos.setAngularScalar(0.994240643);

        otos.calibrateImu();

        otos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
}
