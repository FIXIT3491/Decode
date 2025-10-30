package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {

    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private Servo gate;

    // Adjustable servo positions
    private double gateClosedPos = 0.33;
    private double gateOpenPos = 0.44;

    // Initialize motors
    public void init(HardwareMap hardwareMap) {
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "flyLeft");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "flyRight");
        gate = hardwareMap.get(Servo.class, "gate");

        // Make sure both spin in the same direction
        leftFlywheel.setDirection(DcMotorEx.Direction.FORWARD);
        rightFlywheel.setDirection(DcMotorEx.Direction.REVERSE);

        // Set gate to closed on init
        gate.setPosition(gateClosedPos);
    }

    // Spin both flywheels
    public void setFlywheelVelocity(double angularRate) {
        leftFlywheel.setVelocity(angularRate);
        rightFlywheel.setVelocity(angularRate);
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
