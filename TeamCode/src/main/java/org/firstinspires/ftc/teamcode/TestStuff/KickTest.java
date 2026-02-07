package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.Launcher;
import org.firstinspires.ftc.teamcode.Commands.WheelRotation;

@Disabled
@TeleOp(group = "Test", name = "Kick Test")
public class KickTest extends OpMode {

    private Servo kick;
    private Servo kick2;

    private ElapsedTime timer = new ElapsedTime();
    private int state = 0;

    private boolean lastA = false;

    // positions (adjust as needed)
    private final double KICK_UP = 0.6;
    private final double KICK_DOWN = 0.2;
    private final double KICK2_UP = 1;
    private final double KICK2_DOWN = 0.6;

    private DcMotorEx flywheel;
    private DcMotor ferrisMotor;


    private double intakeCounter = 0;
    private double outtakeCounter = 0;

    boolean prevLeftBumper = false;
    boolean prevRightBumper = false;

    private WheelRotation wheel = new WheelRotation();

    @Override
    public void init() {
        kick = hardwareMap.get(Servo.class, "kick");
        kick2 = hardwareMap.get(Servo.class, "kick2");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        ferrisMotor = hardwareMap.get(DcMotor.class, "ferrisWheel");
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        kick2.setDirection(Servo.Direction.REVERSE);

        kick.setPosition(KICK_DOWN);
        kick2.setPosition(KICK2_DOWN);

        wheel.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        boolean aPressed = gamepad1.a;

        // Rising-edge detection for A
        if (aPressed && !lastA && state == 0) {
            kick.setPosition(KICK_UP);
            timer.reset();
            state = 1;
        }

        // After 0.05s, spin kick2 up
        if (state == 1 && timer.seconds() > 0.1) {
            kick2.setPosition(KICK2_UP);
            timer.reset();
            state = 2;
        }

        // After 0.2s, return both
        if (state == 2 && timer.seconds() > 0.2) {
            kick.setPosition(KICK_DOWN);
            kick2.setPosition(KICK2_DOWN);
            timer.reset();
            state = 3;
        }

        // AFTER kicks are done -> rotate to next area
        if (state == 3 && timer.seconds() > 0.4) {

            switch ((int) outtakeCounter) {
                case 0:
                    wheel.rotateToAngle(120, 0.3);
                    break;
                case 1:
                    wheel.rotateToAngle(240, 0.3);
                    break;
                case 2:
                    wheel.rotateToAngle(1, 0.3);
                    break;
            }

            outtakeCounter = (outtakeCounter + 1) % 3;
            state = 0;   // return to idle so it only runs once
        }

        lastA = aPressed;

        if (gamepad1.right_trigger > 0) {

            flywheel.setPower(1);

        } else {

            flywheel.setPower(0);

        }

        wheel.updateTelemetry();

    }
}
