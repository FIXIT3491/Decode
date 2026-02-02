package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class WheelRotation {

    // Motor reference
    private DcMotor ferrisMotor;

    // Magnetic limit switch
    private DigitalChannel magSwitch;

    // --- CONFIG ---
    private static final boolean MAGNET_ACTIVE_LOW = true;

    // Encoder constants
    private static final double COUNTS_PER_REV = 537.7;  // goBILDA 312RPM 1:1
    private static final double TICKS_PER_DEGREE = COUNTS_PER_REV / 360.0;
    private static double TUNER = 1.0;
    private Telemetry telemetry;
    private double targetAngleDeg = 0.0;

    public WheelRotation() {}

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        ferrisMotor = hardwareMap.get(DcMotor.class, "ferrisWheel");
        magSwitch  = hardwareMap.get(DigitalChannel.class, "magSwitch");

        magSwitch.setMode(DigitalChannel.Mode.INPUT);

        ferrisMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ferrisMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ferrisMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void rotateToAngle(double targetDegrees, double maxPower) {

        final double TOLERANCE_DEG = 0.5;
        final double MIN_POWER = 0.25;

        final double MAGNET_ARM_DELAY_MS = 250;
        final double MAGNET_ACTIVE_WINDOW_DEG = 5;

        targetDegrees = normalizeAngle(targetDegrees);
        targetDegrees = normalizeAngle(targetDegrees);
        targetAngleDeg = targetDegrees;


        double currentDegrees = normalizeAngle(getCurrentDegrees());

        // Only go in one direction
        double deltaDegrees = (targetDegrees - currentDegrees + 360.0) % 360.0;

        if (deltaDegrees <= TOLERANCE_DEG) return;

        int deltaTicks = (int) (deltaDegrees * TICKS_PER_DEGREE * TUNER);
        int targetTicks = ferrisMotor.getCurrentPosition() + deltaTicks;

        ferrisMotor.setTargetPosition(targetTicks);
        ferrisMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power = Math.max(
                MIN_POWER,
                Math.min(maxPower, deltaDegrees / 45.0)
        );

        ferrisMotor.setPower(power);

        ElapsedTime timer = new ElapsedTime();

        while (ferrisMotor.isBusy()) {

            boolean timeArmed = timer.milliseconds() > MAGNET_ARM_DELAY_MS;
            boolean nearTarget = deltaDegrees <= MAGNET_ACTIVE_WINDOW_DEG;

            if (timeArmed && nearTarget && isMagnetDetected()) {
                ferrisMotor.setPower(0);
                ferrisMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ferrisMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return;
            }
        }

        ferrisMotor.setPower(0);
        ferrisMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    // ---------------- Helpers ----------------

    private boolean isMagnetDetected() {
        boolean state = magSwitch.getState();
        return MAGNET_ACTIVE_LOW ? !state : state;
    }

    private double normalizeAngle(double angle) {
        angle %= 360.0;
        if (angle < 0) angle += 360.0;
        return angle;
    }

    private double shortestAngleDelta(double current, double target) {
        double delta = target - current;
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;
        return delta;
    }

    public double getCurrentDegrees() {
        return (ferrisMotor.getCurrentPosition() / TICKS_PER_DEGREE) % 360;
    }

    public void adjustWheel(int modifier) {
        rotateToAngle(getCurrentDegrees() + (5 * modifier), 0.8);
    }

    public void updateTelemetry() {
        if (telemetry == null) return;

        telemetry.addData("Ferris Target (deg)", "%.2f", targetAngleDeg);
        telemetry.addData("Ferris Actual (deg)", "%.2f", getCurrentDegrees());
        telemetry.addData("Ferris Encoder", ferrisMotor.getCurrentPosition());
        telemetry.addData("Mag Switch", isMagnetDetected() ? "DETECTED" : "clear");
        telemetry.update();
    }

}
