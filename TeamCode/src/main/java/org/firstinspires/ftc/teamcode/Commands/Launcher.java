package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {

    private DcMotorEx flywheel;
    private Servo kick;

    // Adjustable servo positions
    private double kickBackPos = 0;
    private double kickFrontPos = 0.2;

    // REV Through-Bore Encoder -> 28 ticks per revolution
    private static final double TICKS_PER_REV = 28.0;

    // PIDF constants (tune as needed)
    private final double kF = 0.0002;
    private final double kP = 0.0004;
    private final double kI = 0.0;
    private final double kD = 0.0001;

    // Controller state
    private double targetRPM = 0;
    private double integral = 0;
    private double lastError = 0;

    public void init(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        kick = hardwareMap.get(Servo.class, "kick");

        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        kick.setPosition(kickBackPos);
    }

    public void setFlywheelPower(double power) {
        flywheel.setPower(power);
    }

    public void setFlywheelRPM(double rpm) {
        targetRPM = rpm;
    }

    public void updateFlywheel() {
        double currentRPM = (flywheel.getVelocity() / TICKS_PER_REV) * 60.0;

        // Reset PID when stopped
        if (targetRPM == 0) {
            integral = 0;
            lastError = 0;
        }

        double error = targetRPM - currentRPM;

        integral += error;
        integral = Math.max(-1500, Math.min(integral, 1500)); // anti-windup

        double derivative = error - lastError;
        lastError = error;

        double feedforward = kF * targetRPM;

        double output = feedforward
                + (kP * error)
                + (kI * integral)
                + (kD * derivative);

        output = Math.max(-1, Math.min(output, 1));

        flywheel.setPower(output);
    }

    public void stop() {
        targetRPM = 0;
        flywheel.setPower(0);
    }

    // --- KICK METHODS ---
    public void kick() {
        kick.setPosition(kickFrontPos);
    }

    public void kickBack() {
        kick.setPosition(kickBackPos);
    }
}
