package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;
//import org.firstinspires.ftc.teamcode.Commands.AprilTagDriveSubsystem;
import org.firstinspires.ftc.teamcode.Commands.Launcher;
import org.firstinspires.ftc.teamcode.Commands.WheelRotation;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.Commands.WheelRotation;
import org.firstinspires.ftc.teamcode.Commands.Launcher;

@TeleOp(name = "Basic TeleOp Blue", group = "Main TeleOp")
public class BasicTeleOpBlue extends OpMode {

    /* ---------------- Hardware ---------------- */

    private BasicMecanumDrive drive;
    private Launcher launcher;

    private DcMotor intake;
    private DcMotor ferrisMotor;

    private Servo kick;
    private Servo kick2;
    private Servo hood;

    private Servo park1;
    private Servo park2;

    private WheelRotation wheel = new WheelRotation();

    private ColorSensor intakeColor;
    private ColorSensor outtakeColor;

    /* ---------------- Kick State Machine ---------------- */

    private ElapsedTime kickTimer = new ElapsedTime();
    private ElapsedTime parkTimer = new ElapsedTime();
    private int kickState = 0;
    private int kickWheelCounter = 0;

    private final double KICK_UP = 0.6;
    private final double KICK_DOWN = 0.2;
    private final double KICK2_UP = 1.0;
    private final double KICK2_DOWN = 0.5;

    /* ---------------- Intake / Outtake ---------------- */

    private int intakeCounter = 0;
    private int outtakeCounter = 0;

    private boolean intakeActive = false;
    private boolean outtakeActive = false;

    private boolean lastIntakeColor = false;
    private boolean lastOuttakeColor = false;

    /* ---------------- Button Edge Tracking ---------------- */

    private boolean lastA = false;
    private boolean lastRT = false;
    private boolean lastLT = false;
    private boolean lastLauncherRB = false;
    private boolean hoodManual = false;

    /* ---------------- Launcher ---------------- */

    private boolean launcherAuto = false;

    /* ---------------- Init ---------------- */

    @Override
    public void init() {

        drive = new BasicMecanumDrive(hardwareMap);
        launcher = new Launcher();
        launcher.init(hardwareMap);
        launcher.setTrackedTagId(20);

        intake = hardwareMap.get(DcMotor.class, "intake");
        ferrisMotor = hardwareMap.get(DcMotor.class, "ferrisWheel");

        kick = hardwareMap.get(Servo.class, "kick");
        kick2 = hardwareMap.get(Servo.class, "kick2");

        park1 = hardwareMap.get(Servo.class, "blFoot");
        park2 = hardwareMap.get(Servo.class, "frFoot");

        intakeColor = hardwareMap.get(ColorSensor.class, "color2");
        outtakeColor = hardwareMap.get(ColorSensor.class, "color1");

        kick2.setDirection(Servo.Direction.REVERSE);

        kick.setPosition(KICK_DOWN);
        kick2.setPosition(KICK2_DOWN);

        park1.setPosition(0.7);
        park2.setPosition(0.7);

        hood = hardwareMap.get(Servo.class, "hood");

        wheel.init(hardwareMap, telemetry);
    }

    /* ---------------- Loop ---------------- */

    @Override
    public void loop() {

        /* -------- Drive -------- */
        if (gamepad1.back) {drive.resetHeading();}
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        drive.drive(y, x, rx);

        /* -------- Inputs -------- */

        boolean aPressed = gamepad2.a;
        boolean rt = gamepad1.right_trigger > 0.1;
        boolean lt = gamepad1.left_trigger > 0.1;

        boolean launcherRB = gamepad2.right_bumper;
        boolean launcherLB = gamepad2.left_bumper;

        /* ===================== LAUNCHER CONTROL ===================== */

        if (launcherRB && !lastLauncherRB) {
            launcherAuto = true;
        }

        if (launcherLB) {
            launcherAuto = false;
            launcher.stopFlywheel();
        }

        if (launcherAuto) {
            launcher.updateTurretFromAprilTag();
            launcher.updateFlywheelFromAprilTag();
        }

        if (gamepad2.x) {
            launcher.setFlywheelRPM(3200);
            hood.setPosition(0.0);}
        else if (gamepad2.y) {
            launcher.setFlywheelRPM(4000);
            hood.setPosition(0.17);}
        else if (gamepad2.b) {
            launcher.setFlywheelRPM(4800);
            hood.setPosition(0.25);}
        else {
            launcher.setFlywheelRPM(0);
            hood.setPosition(0.1);
        }

        launcher.updateFlywheel();

        /* ===================== INTAKE MOTOR ===================== */

        if (rt) {
            intake.setPower(1);
        } else if (lt) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        /* ===================== INTAKE MODE ===================== */

        if (rt && !lastRT) {
            wheel.rotateToAngle(35, 0.3);
            intakeCounter = 1;
            intakeActive = true;
        }

        boolean intakeDetected = isGreenOrPurple(intakeColor);

        if (rt && intakeDetected && !lastIntakeColor && intakeActive) {
            switch (intakeCounter) {
                case 0: wheel.rotateToAngle(35, 0.4); break;
                case 1: wheel.rotateToAngle(148, 0.4); break;
                case 2: wheel.rotateToAngle(268, 0.4); break;
            }
            intakeCounter = (intakeCounter + 1) % 3;
        }

        if (!rt && intakeActive) {
            wheel.rotateToAngle(358, 0.3);
            intakeActive = false;
            intakeCounter = 0;
        }

        /* ===================== OUTTAKE MODE ===================== */

        if (lt && !lastLT) {
            wheel.rotateToAngle(90, 0.3);
            outtakeCounter = 1;
            outtakeActive = true;
        }

        boolean outtakeDetected = isGreenOrPurple(outtakeColor);

        if (lt && outtakeDetected && !lastOuttakeColor && outtakeActive) {
            switch (outtakeCounter) {
                case 0: wheel.rotateToAngle(108, 0.4); break;
                case 1: wheel.rotateToAngle(220, 0.4); break;
                case 2: wheel.rotateToAngle(340, 0.4); break;
            }
            outtakeCounter = (outtakeCounter + 1) % 3;
        }

        if (!lt && outtakeActive) {
            wheel.rotateToAngle(358, 0.3);
            outtakeActive = false;
            outtakeCounter = 0;
        }

        /* ===================== KICK MODE ===================== */

        if (!rt && !lt) {

            if (aPressed && !lastA && kickState == 0) {
                kick.setPosition(KICK_UP);
                kickTimer.reset();
                kickState = 1;
            }

            if (kickState == 1 && kickTimer.seconds() > 0.1) {
                kick2.setPosition(KICK2_UP);
                kickTimer.reset();
                kickState = 2;
            }

            if (kickState == 2 && kickTimer.seconds() > 0.2) {
                kick.setPosition(KICK_DOWN);
                kick2.setPosition(KICK2_DOWN);
                kickTimer.reset();
                kickState = 3;
            }

            if (kickState == 3 && kickTimer.seconds() > 0.4) {

                switch (kickWheelCounter) {
                    case 0: wheel.rotateToAngle(122, 0.4); break;
                    case 1: wheel.rotateToAngle(245, 0.4); break;
                    case 2: wheel.rotateToAngle(4, 0.4); break;
                }

                kickWheelCounter = (kickWheelCounter + 1) % 3;
                kickState = 0;
            }
        }

        /* -------- Parking -------- */
        if (gamepad1.dpad_down) {
            park1.setPosition(0.3);
            park2.setPosition(0.3);
            parkTimer.reset();
        } else if (gamepad1.dpad_up) {
            park1.setPosition(0.7);
            park2.setPosition(0.7);
        }

        /* -------- Edge Updates -------- */

        lastA = aPressed;
        lastRT = rt;
        lastLT = lt;
        lastLauncherRB = launcherRB;
        lastIntakeColor = intakeDetected;
        lastOuttakeColor = outtakeDetected;

        wheel.updateTelemetry();

        telemetry.addData("Launcher Auto", launcherAuto);
        telemetry.addData("Flywheel RPM", "%.0f", launcher.getCurrentRPM());
        telemetry.addData("Hood Mode", hoodManual ? "MANUAL" : "AUTO");
        telemetry.update();
    }

    @Override
    public void stop() {
        launcher.stopFlywheel();
        launcher.closeVision();
    }

    /* ---------------- Color Detection ---------------- */

    private boolean isGreenOrPurple(ColorSensor sensor) {

        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        int total = r + g + b;
        if (total < 350) return false;

        boolean isGreen = g > r + 40 && g > b + 40;
        boolean isPurple = b > g + 30 && r > g + 10;

        return isGreen || isPurple;
    }
}
