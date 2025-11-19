package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Launcher;

@TeleOp(group = "Test", name = "Flywheel Increase")
public class Flywheel_Incremental_Increase extends OpMode {

    double power = 0.5;

    Launcher launcher = new Launcher();

    // Previous button states (for debouncing)
    boolean prevDpadLeft = false;
    boolean prevDpadRight = false;
    boolean prevRB = false;
    boolean prevLB = false;
    boolean prevB = false;

    // Toggle state
    boolean flywheelOn = false;

    @Override
    public void init() {
        launcher.init(hardwareMap);
    }

    @Override
    public void loop() {

        // decrease 0.01
        if (gamepad1.dpad_left && !prevDpadLeft) {
            power -= 0.01;
        }
        prevDpadLeft = gamepad1.dpad_left;

        // increase 0.01
        if (gamepad1.dpad_right && !prevDpadRight) {
            power += 0.01;
        }
        prevDpadRight = gamepad1.dpad_right;

        // increase 0.1
        if (gamepad1.right_bumper && !prevRB) {
            power += 0.1;
        }
        prevRB = gamepad1.right_bumper;

        // decrease 0.1
        if (gamepad1.left_bumper && !prevLB) {
            power -= 0.1;
        }
        prevLB = gamepad1.left_bumper;

        //Toggle Flywheel
        if (gamepad1.b && !prevB) {
            flywheelOn = !flywheelOn;
        }
        prevB = gamepad1.b;

        // Flywheel control based on toggle
        if (flywheelOn) {
            launcher.setFlywheelRPM(power);
        } else {
            launcher.stop();
        }

        telemetry.addData("Flywheel Power:", power);
        telemetry.addData("Flywheel On:", flywheelOn);
        telemetry.update();
    }
}
