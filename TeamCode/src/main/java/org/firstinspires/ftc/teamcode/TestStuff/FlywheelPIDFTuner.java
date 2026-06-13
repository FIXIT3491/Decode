package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Flywheel PIDF Tuner", group = "Tuning")
public class FlywheelPIDFTuner extends LinearOpMode {

    private DcMotorEx flywheel;

    // ================= PIDF VALUES =================
    // Change these LIVE between runs
    private static double kP = 1.4;
    private static double kI = 0.0;
    private static double kD = 0;
    private static double kF = 14.2;

    // ================= TARGET RPM =================
    private static double targetRPM = 4000;

    // GoBILDA 5202/5203 style motor
    private static final double TICKS_PER_REV = 28.0;

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        flywheel.setDirection(DcMotorEx.Direction.REVERSE);

        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Read original PIDF
        PIDFCoefficients original =
                flywheel.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Original REV PIDF");
        telemetry.addData("P", original.p);
        telemetry.addData("I", original.i);
        telemetry.addData("D", original.d);
        telemetry.addData("F", original.f);
        telemetry.update();

        // Apply new PIDF
        PIDFCoefficients tuned =
                new PIDFCoefficients(kP, kI, kD, kF);

        flywheel.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                tuned
        );

        waitForStart();

        timer.reset();

        while (opModeIsActive()) {

            // ================= CONTROLS =================

            // Dpad Up/Down = RPM adjust
            if (gamepad1.dpad_up) {
                targetRPM += 50;
                sleep(120);
            }

            if (gamepad1.dpad_down) {
                targetRPM -= 50;
                sleep(120);
            }

            // A = Start flywheel
            if (gamepad1.a) {
                flywheel.setVelocity(rpmToTicks(targetRPM));
            }

            // B = Stop flywheel
            if (gamepad1.b) {
                flywheel.setVelocity(0);
            }

            // ================= TELEMETRY =================

            double currentRPM = getRPM();

            telemetry.addLine("==== FLYWHEEL PIDF TUNER ====");

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", currentRPM);
            telemetry.addData("Error", targetRPM - currentRPM);

            telemetry.addLine();

            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);

            telemetry.addLine();

            telemetry.addLine("A = Start Flywheel");
            telemetry.addLine("B = Stop Flywheel");
            telemetry.addLine("Dpad Up/Down = Adjust RPM");

            telemetry.update();
        }
    }

    private double getRPM() {
        return (flywheel.getVelocity() / TICKS_PER_REV) * 60.0;
    }

    private double rpmToTicks(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }
}