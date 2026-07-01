package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Commands.Launcher;

@Disabled
@TeleOp(name = "Flywheel PIDF Sample", group = "Tuning")
public class SamplePIDFTest extends LinearOpMode {

    private DcMotorEx flywheel;
    private Launcher launcher;

    private DcMotor intake;
    private DcMotor intake2;

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

        launcher = new Launcher();
        launcher.init(hardwareMap);

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        timer.reset();

        double F = 0;

        while (opModeIsActive()) {



            while (true) {
                launcher.setFlywheelRPM(4200);
                launcher.updateFlywheel();

                boolean rt = gamepad1.right_trigger > 0.1;
                boolean lt = gamepad1.left_trigger > 0.1;
                boolean rt2 = gamepad2.right_trigger > 0.1;

                double intakePower = 0;
                double intakePower2 = 0;

                if (rt) {
                    intakePower = -0.9;
                } else if (lt) {
                    intakePower = 0.9;
                } else if (rt2) {
                    intakePower = -0.8;
                    intakePower2 = -1;
                } else {
                    intakePower = 0;
                    intakePower2 = 0;
                }

                if (gamepad1.a && !rt2) {
                    intakePower2 = -0.25;
                }

                intake.setPower(intakePower);
                intake2.setPower(intakePower2);
            } /*else {
                if (gamepad1.yWasPressed()) {
                    flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    flywheel.setPower(1.0);

                    if (gamepad1.leftBumperWasReleased()) {//13.501
                        double maxVel = flywheel.getVelocity();
                        F = 32767.0 / maxVel;     // now load-accurate, will be larger
                    }
                }*/


            //telemetry.addData("F: ", F);
            //telemetry.addData("act Vel: ", launcher.getCurrentRPM());
            //telemetry.update();

        }
    }

    /*private double getRPM() {
        return (flywheel.getVelocity() / TICKS_PER_REV) * 60.0;
    }

    private double rpmToTicks(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }*/
}