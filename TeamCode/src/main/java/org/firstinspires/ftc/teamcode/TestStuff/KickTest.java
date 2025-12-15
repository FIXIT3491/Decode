package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Launcher;

@TeleOp(group = "Test", name = "Kick Test")
public class KickTest extends OpMode {

    Launcher launcher = new Launcher();

    @Override
    public void init() {
        launcher.init(hardwareMap);
        launcher.kickBack();
    }

    @Override
    public void loop() {

       if (gamepad1.a) {
           launcher.kick();
       } else {
           launcher.kickBack();
       }

    }
}
