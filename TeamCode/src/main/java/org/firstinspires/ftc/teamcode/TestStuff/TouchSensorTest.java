package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class TouchSensorTest extends OpMode {

    // Declare motor and sensor objects
    private DcMotor motor;
    private TouchSensor touchSensor;

    @Override
    public void init() {
        // Initialize the hardware devices
        motor = hardwareMap.get(DcMotor.class, "motor");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        // Set motor to use encoders
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // Check if the touch sensor is pressed
        if (touchSensor.isPressed()) {
            // Reset the motor encoder values to zero
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Touch Sensor: ", "Pressed");
        } else {
            telemetry.addData("Touch Sensor: ", "Not Pressed");
        }

        // Display current encoder position
        telemetry.addData("Encoder Position: ", motor.getCurrentPosition());
        telemetry.update();
    }
}