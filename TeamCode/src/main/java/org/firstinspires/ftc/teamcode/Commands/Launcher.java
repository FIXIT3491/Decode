package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {

    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    //private Servo gate;

    // Adjustable servo positions
    //private double gateClosedPos = 0.57;
    //private double gateOpenPos = 0.28;

    // REV Through-Bore Encoder -> 28 ticks per revolution
    private static final double TICKS_PER_REV = 28.0;

    // Attempts to do set RPM for REV motors
    // PIDF constants (tune kP if needed)
    private final double kF = 0.0002;
    private final double kP = 0.0004;
    private final double kD = 0.0001;

    private final double kI = 0.0;

    // Feedforward constant for 6000 RPM UltraPlanetary
    // Controller state
    private double targetRPM = 0;
    private double integral = 0;
    private double lastError = 0;

    public void init(HardwareMap hardwareMap) {
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "flyLeft");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "flyRight");
        //gate = hardwareMap.get(Servo.class, "gate");

        leftFlywheel.setDirection(DcMotorEx.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotorEx.Direction.FORWARD);

        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //gate.setPosition(gateClosedPos);
    }

    public void setFlywheelRPM(double rpm) {
        targetRPM = rpm;
        //double ticksPerSecond = (rpm * TICKS_PER_REV) / 60.0; PIDF doesnt work, need to manage it to make it work for REV Ultraplanetary

        //leftFlywheel.setVelocity(ticksPerSecond);
        //rightFlywheel.setVelocity(ticksPerSecond);

        /*temp code until PIDF stuff is figured out
        leftFlywheel.setPower(rpm);
        rightFlywheel.setPower(rpm); */
    }

    public void updateFlywheels() {

        double rpmLeft  = (leftFlywheel.getVelocity()  / TICKS_PER_REV) * 60.0;
        double rpmRight = (rightFlywheel.getVelocity() / TICKS_PER_REV) * 60.0;
        double currentRPM = (rpmLeft + rpmRight) / 2.0;

        // Reset PID when flywheel is turned off
        if (targetRPM == 0) {
            integral = 0;
            lastError = 0;
        }

        double error = targetRPM - currentRPM;

        integral += error;
        integral = Math.max(-1500, Math.min(integral, 1500));  // anti-windup

        double derivative = error - lastError;
        lastError = error;

        double feedforward = kF * targetRPM;

        double output = feedforward
                + (kP * error)
                + (kI * integral)
                + (kD * derivative);

        // allow braking
        output = Math.max(-1, Math.min(output, 1));

        leftFlywheel.setPower(output);
        rightFlywheel.setPower(output);
    }


    public void stop() {
        //leftFlywheel.setVelocity(0);
        //rightFlywheel.setVelocity(0);
        setFlywheelRPM(0);
        leftFlywheel.setPower(0);
        rightFlywheel.setPower(0);
    }

    // --- GATE METHODS ---
    public void openGate() {
        //gate.setPosition(gateOpenPos);
    }

    public void closeGate() {
        //gate.setPosition(gateClosedPos);
    }

}
