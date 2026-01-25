package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.Launcher;

@TeleOp(group = "Test", name = "Kick Test")
public class KickTest extends OpMode {

    private Servo kick;
    private Servo kick2;

    private ElapsedTime timer = new ElapsedTime();
    private int state = 0;

    private boolean lastA = false;

    // positions (adjust as needed)
    private final double KICK_UP = 0.6;
    private final double KICK_DOWN = 0.25;
    private final double KICK2_UP = 1;
    private final double KICK2_DOWN = 0.6;

    private DcMotorEx flywheel;

    @Override
    public void init() {
        kick = hardwareMap.get(Servo.class, "kick");
        kick2 = hardwareMap.get(Servo.class, "kick2");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        kick2.setDirection(Servo.Direction.REVERSE);

        kick.setPosition(KICK_DOWN);
        kick2.setPosition(KICK2_DOWN);
    }

    @Override
    public void loop() {

        boolean aPressed = gamepad1.a;

        // Rising-edge detection (debounce)
        if (aPressed && !lastA && state == 0) {
            kick.setPosition(KICK_UP);
            timer.reset();
            state = 1;
        }

        // After 0.5s, spin kick2 up
        if (state == 1 && timer.seconds() > 0.05) {
            kick2.setPosition(KICK2_UP);
            timer.reset();
            state = 2;
        }

        // After another 0.5s, return both
        if (state == 2 && timer.seconds() > 0.2) {
            kick.setPosition(KICK_DOWN);
            kick2.setPosition(KICK2_DOWN);
            state = 0;
        }

        // Save button state for next loop
        lastA = aPressed;

        if (gamepad1.right_trigger > 0) {

            flywheel.setPower(1);

        } else {

            flywheel.setPower(0);

        }

    }
}
