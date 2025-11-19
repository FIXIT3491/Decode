package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Launcher;

@TeleOp(group = "Test", name = "Flywheel Increase")
public class Flywheel_Incremental_Increase extends OpMode{

    Launcher launcher = new Launcher();

    @Override
    public void init(){

        //Init Launcher Subsystem
        launcher.init(hardwareMap);

    }

    @Override
    public void loop() {

        double power = 0.5;

        //adjusments of power for flywheel testing
        if (gamepad1.dpad_left) {

            power -= 0.01;

        } else if (gamepad1.dpad_right) {

            power += 0.01;

        } else if (gamepad1.right_bumper) {

            power += 0.1;

        } else if (gamepad1.left_bumper) {

            power += 0.1;

        }

        //Turning on flywheel
        if (gamepad1.b) {

            launcher.setFlywheelRPM(power);

        } else {

            launcher.stop();

        }

        telemetry.addData("Percentage:", power);
        telemetry.update();

    }

}
