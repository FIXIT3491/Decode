package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.Commands.WheelRotation;

@TeleOp(name = "Intake Test (Color Sensor)", group = "Test")
public class IntakeTest extends OpMode {

    private BasicMecanumDrive drive;
    private DcMotor intake;
    private WheelRotation wheel = new WheelRotation();

    private ColorSensor intakeColor;
    private ColorSensor outtakeColor;

    private boolean lastIntakeColor = false;
    private boolean lastOuttakeColor = false;

    private boolean intakeInitialized = false;
    private boolean outtakeInitialized = false;

    private boolean wheelAtIdle = true;
    private boolean outtakeWheelAtIdle = true;

    private int intakeCounter = 0;
    private int outtakeCounter = 0;

    private boolean lastIntakeTrigger = false;
    private boolean lastOuttakeTrigger = false;

    @Override
    public void init() {
        drive = new BasicMecanumDrive(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");

        intakeColor = hardwareMap.get(ColorSensor.class, "color2");
        outtakeColor = hardwareMap.get(ColorSensor.class, "color1");

        wheel.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        // Drive
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        drive.drive(y, x, rx);

        boolean intakeTrigger = gamepad1.right_trigger > 0.1;
        boolean outtakeTrigger = gamepad1.left_trigger > 0.1;

        // Intake Motor
        if (intakeTrigger) {
            intake.setPower(0.8);
        } else if (outtakeTrigger) {
            intake.setPower(-0.8);
        } else {
            intake.setPower(0);
        }

        // Intake Init
        if (intakeTrigger && !lastIntakeTrigger) {
            wheel.rotateToAngle(35, 0.3);
            intakeCounter = 1;
            intakeInitialized = true;
            wheelAtIdle = false;
        }

        // Intake Color Advance
        boolean intakeColorDetected = isGreenOrPurple(intakeColor);

        if (intakeColorDetected && !lastIntakeColor && intakeInitialized) {
            switch (intakeCounter) {
                case 0:
                    wheel.rotateToAngle(35, 0.4);
                    break;
                case 1:
                    wheel.rotateToAngle(148, 0.4);
                    break;
                case 2:
                    wheel.rotateToAngle(268, 0.4);
                    break;
            }

            intakeCounter = (intakeCounter + 1) % 3;
            wheelAtIdle = false;
        }

        // Intake Reset
        if (!intakeTrigger && !wheelAtIdle) {
            wheel.rotateToAngle(358, 0.3);
            wheelAtIdle = true;
            intakeInitialized = false;
            intakeCounter = 0;
        }

        // Outtake Init
        if (outtakeTrigger && !lastOuttakeTrigger) {
            wheel.rotateToAngle(90, 0.3);
            outtakeCounter = 1;
            outtakeInitialized = true;
            outtakeWheelAtIdle = false;
        }

        // Outtake Color Advance
        boolean outtakeColorDetected = isGreenOrPurple(outtakeColor);

        if (outtakeColorDetected && !lastOuttakeColor && outtakeInitialized) {
            switch (outtakeCounter) {
                case 0:
                    wheel.rotateToAngle(100, 0.4);
                    break;
                case 1:
                    wheel.rotateToAngle(220, 0.4);
                    break;
                case 2:
                    wheel.rotateToAngle(340, 0.4);
                    break;
            }

            outtakeCounter = (outtakeCounter + 1) % 3;
            outtakeWheelAtIdle = false;
        }

        // Outtake Reset
        if (!outtakeTrigger && !outtakeWheelAtIdle) {
            wheel.rotateToAngle(358, 0.3);
            outtakeWheelAtIdle = true;
            outtakeInitialized = false;
            outtakeCounter = 0;
        }

        // Edge Updates
        lastIntakeColor = intakeColorDetected;
        lastOuttakeColor = outtakeColorDetected;
        lastIntakeTrigger = intakeTrigger;
        lastOuttakeTrigger = outtakeTrigger;

        // Telemetry
        telemetry.addData("Intake RGB", "%d %d %d",
                intakeColor.red(), intakeColor.green(), intakeColor.blue());
        telemetry.addData("Outtake RGB", "%d %d %d",
                outtakeColor.red(), outtakeColor.green(), outtakeColor.blue());
        telemetry.addData("Intake Detected", isGreenOrPurple(intakeColor));
        telemetry.addData("Outtake Detected", isGreenOrPurple(outtakeColor));

        wheel.updateTelemetry();
    }

    // Color Detection Helper
    private boolean isGreenOrPurple(ColorSensor sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        int total = r + g + b;

        // Ignore noise / empty intake
        if (total < 400) {
            return false;
        }

        // GREEN: green is clearly highest
        boolean isGreen = g > r + 80 && g > b + 80;

        // PURPLE: blue highest, red close behind, green lower
        boolean isPurple = b > g + 60 && r > g + 20;

        return isGreen || isPurple;
    }
}
