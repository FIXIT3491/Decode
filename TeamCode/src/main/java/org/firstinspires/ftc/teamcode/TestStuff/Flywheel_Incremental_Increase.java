package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Commands.Launcher;

@TeleOp(group = "Test", name = "Flywheel Increase")
public class Flywheel_Incremental_Increase extends OpMode {

    double rpm = 2100;

    private DcMotorEx flywheel;

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
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
    }

    @Override
    public void loop() {

        // decrease 50
        if (gamepad1.dpad_left && !prevDpadLeft) {
            rpm -= 50;
        }
        prevDpadLeft = gamepad1.dpad_left;

        // increase 50
        if (gamepad1.dpad_right && !prevDpadRight) {
            rpm += 50;
        }
        prevDpadRight = gamepad1.dpad_right;

        // increase 500
        if (gamepad1.right_bumper && !prevRB) {
            rpm += 500;
        }
        prevRB = gamepad1.right_bumper;

        // decrease 500
        if (gamepad1.left_bumper && !prevLB) {
            rpm -= 500;
        }
        prevLB = gamepad1.left_bumper;

        //Toggle Flywheel
        if (gamepad1.b && !prevB) {
            flywheelOn = !flywheelOn;
        }
        prevB = gamepad1.b;

        // Flywheel control based on toggle
        if (flywheelOn) {
            launcher.setFlywheelRPM(rpm);
        } else {
            launcher.stopFlywheel();
        }

        launcher.updateFlywheel();
        telemetry.addData("Flywheel Target RPM:", rpm);
        telemetry.addData("Actual RPM: ", flywheel.getVelocity());
        telemetry.addData("Flywheel On:", flywheelOn);
        telemetry.update();
    }
}
