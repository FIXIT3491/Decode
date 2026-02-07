package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.Commands.WheelRotation;

@TeleOp(name = "Full System Test", group = "Test")
public class IntakeKickMergeTest extends OpMode {

    /* ---------------- Hardware ---------------- */

    private BasicMecanumDrive drive;

    private DcMotor intake;
    private DcMotorEx flywheel;
    private DcMotor ferrisMotor;

    private Servo kick;
    private Servo kick2;

    private WheelRotation wheel = new WheelRotation();

    /* ---------------- Kick State Machine ---------------- */

    private ElapsedTime kickTimer = new ElapsedTime();
    private int kickState = 0;
    private int kickWheelCounter = 0;

    private final double KICK_UP = 0.6;
    private final double KICK_DOWN = 0.2;
    private final double KICK2_UP = 1.0;
    private final double KICK2_DOWN = 0.6;

    /* ---------------- Intake / Outtake ---------------- */

    private int intakeCounter = 0;
    private int outtakeCounter = 0;

    private boolean intakeActive = false;
    private boolean outtakeActive = false;

    /* ---------------- Button Edge Tracking ---------------- */

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastRT = false;
    private boolean lastLT = false;

    /* ---------------- Init ---------------- */

    @Override
    public void init() {

        drive = new BasicMecanumDrive(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        ferrisMotor = hardwareMap.get(DcMotor.class, "ferrisWheel");

        kick = hardwareMap.get(Servo.class, "kick");
        kick2 = hardwareMap.get(Servo.class, "kick2");

        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        kick2.setDirection(Servo.Direction.REVERSE);

        kick.setPosition(KICK_DOWN);
        kick2.setPosition(KICK2_DOWN);

        wheel.init(hardwareMap, telemetry);
    }

    /* ---------------- Loop ---------------- */

    @Override
    public void loop() {

        /* -------- Drive -------- */

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        drive.drive(y, x, rx);

        /* -------- Inputs -------- */

        boolean aPressed = gamepad1.a;
        boolean bPressed = gamepad1.b;
        boolean rt = gamepad1.right_trigger > 0.1;
        boolean lt = gamepad1.left_trigger > 0.1;

        /* -------- Flywheel -------- */

        flywheel.setPower(gamepad1.right_trigger > 0 ? 1.0 : 0.0);

        /* -------- Intake Motor -------- */

        if (rt) {
            intake.setPower(0.8);
        } else if (lt) {
            intake.setPower(-0.8);
        } else {
            intake.setPower(0.0);
        }

        /* =================================================
           INTAKE MODE (RT)
           ================================================= */

        if (rt && !lastRT) {
            wheel.rotateToAngle(35, 0.3);
            intakeCounter = 1;
            intakeActive = true;
        }

        if (rt && aPressed && !lastA && intakeActive) {
            switch (intakeCounter) {
                case 0: wheel.rotateToAngle(35, 0.3); break;
                case 1: wheel.rotateToAngle(148, 0.3); break;
                case 2: wheel.rotateToAngle(268, 0.3); break;
            }
            intakeCounter = (intakeCounter + 1) % 3;
        }

        if (!rt && intakeActive) {
            wheel.rotateToAngle(358, 0.3);   // idle ONLY for intake
            intakeActive = false;
            intakeCounter = 0;
        }

        /* =================================================
           OUTTAKE MODE (LT)
           ================================================= */

        if (lt && !lastLT) {
            wheel.rotateToAngle(90, 0.3);
            outtakeCounter = 1;
            outtakeActive = true;
        }

        if (lt && bPressed && !lastB && outtakeActive) {
            switch (outtakeCounter) {
                case 0: wheel.rotateToAngle(90, 0.3); break;
                case 1: wheel.rotateToAngle(200, 0.3); break;
                case 2: wheel.rotateToAngle(320, 0.3); break;
            }
            outtakeCounter = (outtakeCounter + 1) % 3;
        }

        if (!lt && outtakeActive) {
            wheel.rotateToAngle(358, 0.3);   // idle ONLY for outtake
            outtakeActive = false;
            outtakeCounter = 0;
        }

        /* =================================================
           KICK MODE (A with NO triggers)
           ================================================= */

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
                    case 0: wheel.rotateToAngle(120, 0.3); break;
                    case 1: wheel.rotateToAngle(240, 0.3); break;
                    case 2: wheel.rotateToAngle(1, 0.3); break;
                }

                kickWheelCounter = (kickWheelCounter + 1) % 3;
                kickState = 0;   // NO idle after kick
            }
        }

        /* -------- Button memory -------- */

        lastA = aPressed;
        lastB = bPressed;
        lastRT = rt;
        lastLT = lt;

        wheel.updateTelemetry();
    }
}
