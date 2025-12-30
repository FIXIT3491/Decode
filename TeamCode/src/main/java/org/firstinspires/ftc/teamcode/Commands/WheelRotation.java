package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class WheelRotation {

    // Motor reference
    private DcMotor ferrisMotor;

    // Encoder constants
    private static final double COUNTS_PER_REV = 537.6 * 3.75;  // goBILDA 60RPM 1:1
    private static final double TICKS_PER_DEGREE = COUNTS_PER_REV / 360.0;
    private static final double GEAR_RATIO = 9.0 / 14.0; // driver gear / driven gear
    private static double TUNER = 1.0; // tune encase its slightly off

    // Constructor
    public WheelRotation() {}

    public void init(HardwareMap hardwareMap) {
        ferrisMotor = hardwareMap.get(DcMotor.class, "ferrisWheel");

        ferrisMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ferrisMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ferrisMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void rotateToAngle(double targetDegrees, double power) {

        int targetTicks = (int)(targetDegrees * TICKS_PER_DEGREE * GEAR_RATIO * TUNER);

        ferrisMotor.setTargetPosition(targetTicks);
        ferrisMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ferrisMotor.setPower(power);

        // Wait until movement finishes
        while (ferrisMotor.isBusy()) {
        }

        ferrisMotor.setPower(0);
        ferrisMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Get degrees for telemetry
    public double getCurrentDegrees() {
        return ferrisMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public void adjustWheel (int modifier) {
        rotateToAngle((getCurrentDegrees() + (5 * modifier)), 0.8);
    }

}
