package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;
//import org.firstinspires.ftc.teamcode.Commands.AprilTagDriveSubsystem;
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

    /* ---------------- Launcher ---------------- */
    private boolean isLauncherManual = false;
    private boolean lastRightBumper = false;

    /* ---------------- Init ---------------- */

    @Override
    public void init() {

        drive = new BasicMecanumDrive(hardwareMap);
        launcher = new Launcher();
        launcher.init(hardwareMap);
        launcher.setTrackedTagId(24);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        boolean rt2 = gamepad2.right_trigger > 0.1;

        /* ===================== LAUNCHER CONTROL ===================== */
        launcher.turretBasicTest();

        if (isLauncherManual) {
            if (gamepad2.x) {
                launcher.setFlywheelRPM(4000);
                hood.setPosition(0.0);
            } else if (gamepad2.y) {
                launcher.setFlywheelRPM(4500);
                hood.setPosition(0.17);
            } else if (gamepad2.b) {
                launcher.setFlywheelRPM(5250);
                hood.setPosition(0.25);
            } else {
                launcher.setFlywheelRPM(0);
                hood.setPosition(0.1);
            }
        } else {
            if (gamepad2.x) {
                launcher.updateFlywheelFromAprilTag();
            } else {
                launcher.stopFlywheel();
            }
        }

        launcher.updateFlywheel();

        //driver 1
        double intakePower = 0;

        if (rt) {
            intakePower = -1;
        } else if (lt) {
            intakePower = 1;
        }

        intake.setPower(intakePower);

        //driver 2
        if (rt2) {
            intake.setPower(-1);
            intake2.setPower(-1);
        } else {
            intake.setPower(0);
            intake2.setPower(0);
        }

        /* -------- Parking -------- */
        if (gamepad1.dpad_down) {
            park1.setPosition(0.61);// bl
            park2.setPosition(0.19);// fr
        } else if (gamepad1.dpad_up) {
            park1.setPosition(0.81); //bl
            park2.setPosition(0.30); //fr
        }

        boolean rightBumper = gamepad2.right_bumper;

        if (rightBumper && !lastRightBumper) {
            isLauncherManual = !isLauncherManual;
        }

        lastRightBumper = rightBumper;

    }

    @Override
    public void stop() {
        launcher.stopFlywheel();
        launcher.closeVision();
    }
}
