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
    private double gateClosedPos = 0.57;
    private double gateOpenPos = 0.28;

    // REV Through-Bore Encoder → 28 ticks per revolution
    private static final double TICKS_PER_REV = 28.0;

    public void init(HardwareMap hardwareMap) {
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "flyLeft");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "flyRight");
        gate = hardwareMap.get(Servo.class, "gate");

        leftFlywheel.setDirection(DcMotor.Direction.FORWARD);
        rightFlywheel.setDirection(DcMotor.Direction.REVERSE);

        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gate.setPosition(gateClosedPos);
    }

    // Convert RPM → ticks/sec
    public void setFlywheelRPM(double rpm) {
        double ticksPerSecond = (rpm * TICKS_PER_REV) / 60.0;

        leftFlywheel.setVelocity(ticksPerSecond);
        rightFlywheel.setVelocity(ticksPerSecond);
    }

    public void stop() {
        leftFlywheel.setVelocity(0);
        rightFlywheel.setVelocity(0);
    }

    // --- GATE METHODS ---
    public void openGate() {
        gate.setPosition(gateOpenPos);
    }

    public void closeGate() {
        gate.setPosition(gateClosedPos);
    }
}
