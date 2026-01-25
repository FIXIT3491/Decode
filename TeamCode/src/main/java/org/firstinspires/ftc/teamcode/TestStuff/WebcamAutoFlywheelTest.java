package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.Commands.Launcher;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Flywheel TeleOp", group = "Test")
public class WebcamAutoFlywheelTest extends LinearOpMode {

    Launcher launcher = new Launcher();

    @Override
    public void runOpMode() {
        launcher.init(hardwareMap);

        waitForStart();

        launcher.setTrackedTagId(20);   // Track tag 3 only
        // launcher.setTrackedTagId(-1); // Track any tag

        while (opModeIsActive()) {
            launcher.updateTurretFromAprilTag();
            launcher.updateFlywheelFromAprilTag();
            launcher.updateFlywheel();

            launcher.closeVision();
        }
    }
}
