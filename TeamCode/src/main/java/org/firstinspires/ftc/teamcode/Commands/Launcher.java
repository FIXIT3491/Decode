package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {

    private DcMotor leftFlywheel;
    private DcMotor rightFlywheel;
    private Servo gate;

    // Adjustable servo positions
    private double gateClosedPos = 0.25;
    private double gateOpenPos = -0.5;

    // Initialize motors
    public void init(HardwareMap hardwareMap) {
        leftFlywheel = hardwareMap.get(DcMotor.class, "flyLeft");
        rightFlywheel = hardwareMap.get(DcMotor.class, "flyRight");
        gate = hardwareMap.get(Servo.class, "gate");

        // Make sure both spin in the same direction
        leftFlywheel.setDirection(DcMotor.Direction.FORWARD);
        rightFlywheel.setDirection(DcMotor.Direction.REVERSE);

        // Set gate to closed on init
        gate.setPosition(gateClosedPos);
    }

    // Spin both flywheels
    public void setFlywheelPower(double power) {
        leftFlywheel.setPower(power);
        rightFlywheel.setPower(power);
    }

    // Stop flywheels
    public void stop() {
        leftFlywheel.setPower(0);
        rightFlywheel.setPower(0);
    }

    // --- GATE METHODS ---

    // Open gate
    public void openGate() {
        gate.setPosition(gateOpenPos);
    }

    // Close gate
    public void closeGate() {
        gate.setPosition(gateClosedPos);
    }

}
