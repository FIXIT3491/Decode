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
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.Commands.WheelRotation;
import org.firstinspires.ftc.teamcode.Commands.Launcher;

@TeleOp(name = "TeleOpTest", group = "Main TeleOp")
public class TeleOpTest extends OpMode {

    /* ---------------- Hardware ---------------- */
    private BasicMecanumDrive drive;
    private Launcher launcher;

    private DcMotor intake;
    private DcMotor intake2;
    private Servo hood;

    private Servo park1;
    private Servo park2;

    /* ---------------- Kick State Machine ---------------- */
    private ElapsedTime parkTimer = new ElapsedTime();

    /* ---------------- Launcher ---------------- */
    private boolean launcherManual = false;

    /* ---------------- Init ---------------- */

    @Override
    public void init() {

        drive = new BasicMecanumDrive(hardwareMap);
        launcher = new Launcher();
        launcher.init(hardwareMap);
        launcher.setTrackedTagId(24);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake = hardwareMap.get(DcMotor.class, "intake2");

        park1 = hardwareMap.get(Servo.class, "blFoot");
        park2 = hardwareMap.get(Servo.class, "frFoot");

        park1.setPosition(0.61);
        park2.setPosition(0.19);

        hood = hardwareMap.get(Servo.class, "hood");
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
        boolean rt = gamepad1.right_trigger > 0.1;
        boolean lt = gamepad1.left_trigger > 0.1;

        /* ===================== LAUNCHER CONTROL ===================== */
        launcher.turretBasicTest();

        if (gamepad2.x) {
            launcher.setFlywheelRPM(3200);
            hood.setPosition(0.0);}
        else if (gamepad2.y) {
            launcher.setFlywheelRPM(3450);
            hood.setPosition(0.17);}
        else if (gamepad2.b) {
            launcher.setFlywheelRPM(4250);
            hood.setPosition(0.25);}
        else {
            launcher.setFlywheelRPM(0);
            hood.setPosition(0.1);
        }

        launcher.updateFlywheel();

        if (rt) {
            intake.setPower(1);
            intake2.setPower(1);
        } else if (lt) {
            intake.setPower(-1);
            intake2.setPower(1);
        } else {
            intake.setPower(0);
            intake2.setPower(0);
        }

        /* -------- Parking -------- */
        if (gamepad1.dpad_down) {
            park1.setPosition(0.61);// bl
            park2.setPosition(0.19);// fr
            parkTimer.reset();
        } else if (gamepad1.dpad_up) {
            park1.setPosition(0.81); //bl
            park2.setPosition(0.30); //fr
        }

    }

    @Override
    public void stop() {
        launcher.stopFlywheel();
        launcher.closeVision();
    }
}
