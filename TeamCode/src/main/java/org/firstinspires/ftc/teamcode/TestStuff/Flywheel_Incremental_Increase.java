package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.Commands.Launcher;

@TeleOp(group = "Test", name = "Flywheel Increase")
public class Flywheel_Incremental_Increase extends OpMode {

    double rpm = 2100;
    double hoodPosition = 0.0;

    private Servo hood;
    private DcMotor intake;
    private DcMotor intake2;

    Launcher launcher = new Launcher();
    private BasicMecanumDrive drive;

    // Previous button states (for debouncing)
    boolean prevDpadLeft = false;
    boolean prevDpadRight = false;
    boolean prevRB = false;
    boolean prevLB = false;
    boolean prevB = false;

    boolean prevDpadLeft2 = false;
    boolean prevDpadRight2 = false;
    boolean prevRB2 = false;
    boolean prevLB2 = false;
    boolean prevB2 = false;

    // Toggle states
    boolean flywheelOn = false;
    boolean hoodEnabled = false;

    @Override
    public void init() {
        launcher.init(hardwareMap);
        drive = new BasicMecanumDrive(hardwareMap);

        hood = hardwareMap.get(Servo.class, "hood");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        if (gamepad1.back) {drive.resetHeading();}
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        drive.drive(y, x, rx);

        boolean rt = gamepad1.right_trigger > 0.1;
        boolean lt = gamepad1.left_trigger > 0.1;
        boolean rt2 = gamepad2.right_trigger > 0.1;

        // FLYWHEEL CONTROLS

        // Decrease RPM by 50
        if (gamepad1.dpad_left && !prevDpadLeft) {
            rpm -= 50;
        }
        prevDpadLeft = gamepad1.dpad_left;

        // Increase RPM by 50
        if (gamepad1.dpad_right && !prevDpadRight) {
            rpm += 50;
        }
        prevDpadRight = gamepad1.dpad_right;

        // Increase RPM by 500
        if (gamepad1.right_bumper && !prevRB) {
            rpm += 500;
        }
        prevRB = gamepad1.right_bumper;

        // Decrease RPM by 500
        if (gamepad1.left_bumper && !prevLB) {
            rpm -= 500;
        }
        prevLB = gamepad1.left_bumper;

        // Toggle flywheel
        if (gamepad1.b && !prevB) {
            flywheelOn = !flywheelOn;
        }
        prevB = gamepad1.b;


        // HOOD CONTROLS

        // Decrease by 0.01
        if (gamepad2.dpad_left && !prevDpadLeft2) {
            hoodPosition -= 0.01;
        }
        prevDpadLeft2 = gamepad2.dpad_left;

        // Increase by 0.01
        if (gamepad2.dpad_right && !prevDpadRight2) {
            hoodPosition += 0.01;
        }
        prevDpadRight2 = gamepad2.dpad_right;

        // Increase by 0.05
        if (gamepad2.right_bumper && !prevRB2) {
            hoodPosition += 0.05;
        }
        prevRB2 = gamepad2.right_bumper;

        // Decrease by 0.05
        if (gamepad2.left_bumper && !prevLB2) {
            hoodPosition -= 0.05;
        }
        prevLB2 = gamepad2.left_bumper;

        // Keep position valid
        hoodPosition = Math.max(0.0, Math.min(1.0, hoodPosition));

        // Toggle hood
        if (gamepad2.b && !prevB2) {
            hoodEnabled = !hoodEnabled;
        }
        prevB2 = gamepad2.b;

        // Apply hood position only when enabled
        if (hoodEnabled) {
            hood.setPosition(hoodPosition);
        }

        // FLYWHEEL UPDATE
        if (flywheelOn) {
            launcher.setFlywheelRPM(rpm);
        } else {
            launcher.stopFlywheel();
        }

        launcher.updateFlywheel();

        double intakePower = 0;
        double intakePower2 = 0;

        if (rt) {
            intakePower = -0.9;
        } else if (lt) {
            intakePower = 0.9;
        } else if (rt2) {
            intakePower = -0.8;
            intakePower2 = -1;
        } else {
            intakePower = 0;
            intakePower2 = 0;
        }

        if (gamepad1.a && !rt2) {
            intakePower2 = -0.25;
        }

        intake.setPower(intakePower);
        intake2.setPower(intakePower2);

        // TELEMETRY
        telemetry.addData("Flywheel Target RPM", rpm);
        telemetry.addData("Actual RPM", launcher.getCurrentRPM());
        telemetry.addData("Flywheel On", flywheelOn);

        telemetry.addLine();

        telemetry.addData("Hood Position", "%.3f", hoodPosition);
        telemetry.addData("Hood Enabled", hoodEnabled);

        telemetry.update();
    }
}