package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorSensorTest extends OpMode {

    // Declare the ColorSensor object
    private ColorSensor colorSensor;

    // Variables to store the detected colors
    private int greenValue = -1;
    private int purpleValue = -1;

    @Override
    public void init() {
        // Initialize the color sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Read the RGB values from the color sensor
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        // Display the RGB values on the telemetry for feedback
        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue", blue);

        // Check if the sensor detects green (using a simple threshold for the green channel)
        if (green > red && green > blue) {
            // Store the green value
            if (greenValue == -1) {
                greenValue = green;
                //greenPosition == intakePosition
                telemetry.addData("Detected", "Green! Stored value: " + greenValue);
            }
        }

        ///could make it so it assumes everything else is purple if not green
        /* Check if the sensor detects purple (simple check for more blue than red and green)
        else if (blue > red && blue > green) {
            // Store the purple value
            if (purpleValue == -1) {
                purpleValue = blue;
                //greenPosition == intakePosition
                telemetry.addData("Detected", "Purple! Stored value: " + purpleValue);
            }
        } */

        // Display the stored values
        if (greenValue != -1) {
            telemetry.addData("Green Value", greenValue);
        }
        if (purpleValue != -1) {
            telemetry.addData("Purple Value", purpleValue);
        }

        telemetry.update();
    }
}

