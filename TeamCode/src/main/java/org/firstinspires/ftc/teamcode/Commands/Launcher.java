package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        leftFlywheel.setDirection(DcMotorEx.Direction.FORWARD);
        rightFlywheel.setDirection(DcMotorEx.Direction.REVERSE);

        //leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        gate.setPosition(gateClosedPos);
    }

    // Convert RPM → ticks/sec
    public void setFlywheelRPM(double rpm) {
        //double ticksPerSecond = (rpm * TICKS_PER_REV) / 60.0; PIDF doesnt work, need to manage it to make it work for REV Ultraplanetary

        //leftFlywheel.setVelocity(ticksPerSecond);
        //rightFlywheel.setVelocity(ticksPerSecond);

        //temp code until that is figured out
        leftFlywheel.setPower(rpm);
        rightFlywheel.setPower(rpm);
    }

    public void stop() {
        //leftFlywheel.setVelocity(0);
        //rightFlywheel.setVelocity(0);

        leftFlywheel.setPower(0);
        rightFlywheel.setPower(0);

    }

    // --- GATE METHODS ---
    public void openGate() {
        gate.setPosition(gateOpenPos);
    }

    public void closeGate() {
        gate.setPosition(gateClosedPos);
    }
}
