package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "Test", name = "Kick Test")
public class KickTest extends OpMode {

    private Servo kick;

    @Override
    public void init() {
        kick = hardwareMap.get(Servo.class, "kick");
        kick.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop() {

       if (gamepad1.a) {
           kick.setPosition(0.5);
       } else {
           kick.setPosition(0.3);
       }

    }
}
