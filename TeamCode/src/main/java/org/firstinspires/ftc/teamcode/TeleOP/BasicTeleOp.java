package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.Commands.AprilTagDriveSubsystem;
import org.firstinspires.ftc.teamcode.Commands.Launcher;

@TeleOp(name = "BasicTeleOp")
public class BasicTeleOp extends OpMode {

    private BasicMecanumDrive drive;
    private AprilTagDriveSubsystem aprilTagDrive;
    Launcher launcher = new Launcher();
    private DcMotor intake;

    @Override
    public void init() {

        // Init Basic Mecanum Drive System
        drive = new BasicMecanumDrive(hardwareMap);

        // Init AprilTag Subsystem
        aprilTagDrive = new AprilTagDriveSubsystem(hardwareMap, telemetry);

        // Init Launcher Subsystem
        launcher.init(hardwareMap);
        telemetry.addData("Status", "Initialized");

        // Init Intake
        intake  = hardwareMap.get(DcMotor.class, "intake");

    }

    @Override
    public void loop() {

        // Manual stick values
        double y = -gamepad1.left_stick_y; // forward/back (abs(left_stick_y) * left_stick_y (Potential for smoother movement (ramp)))
        double x = gamepad1.left_stick_x; // strafe
        double rx = gamepad1.right_stick_x; // rotate

        if (gamepad1.left_bumper) {

            y = -gamepad1.left_stick_y / 2;
            x = gamepad1.left_stick_x / 2;
            rx = gamepad1.right_stick_x / 3;

        }

        // If right bumper held → AprilTag auto mode
        if (gamepad1.right_bumper) {

            aprilTagDrive.driveToTag(true, y, x, rx, -1); // -1 is any tag

        } else {

            // Otherwise use manual mecanum subsystem (field centric)
            drive.drive(y, x, rx);

        }

        // Reset IMU heading with back button
        if (gamepad1.back) {

            drive.resetHeading();

        }

        if (gamepad2.y) { //far

            launcher.setFlywheelVelocity(2775);

        } else if (gamepad2.x){ //close

            launcher.setFlywheelVelocity(1800);

        }else {

            launcher.stop();

        }

        if (gamepad2.a) { //open if held

            launcher.openGate();

        } else { // close if released

            launcher.closeGate();

        }

        intake.setPower(gamepad1.left_trigger);

        if (gamepad1.left_bumper) {

            intake.setPower(-0.75);

        }

    }
}
