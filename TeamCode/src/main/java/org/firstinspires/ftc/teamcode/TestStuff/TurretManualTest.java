package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="TurretManualTest", group="Test")
public class TurretManualTest extends LinearOpMode {

    DcMotorEx turret;

    // Adjust to your robot
    private static final double TURRET_TICKS_PER_REV = 537.7;
    private static final double TURRET_GEAR_RATIO = 2.0;
    static final double TICKS_PER_DEGREE =
            (TURRET_TICKS_PER_REV * TURRET_GEAR_RATIO) / 360.0;


    @Override
    public void runOpMode() {

        turret = hardwareMap.get(DcMotorEx.class, "turretMotor");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        moveTurretDegrees(10);
        sleep(2000);

        moveTurretDegrees(-10);
        sleep(2000);

        moveTurretDegrees(20);
        sleep(2000);

        moveTurretDegrees(-20);
    }

    public void moveTurretDegrees(double degrees) {

        int ticksToMove = (int)(degrees * TICKS_PER_DEGREE);

        int currentPos = turret.getCurrentPosition();
        int targetPos = currentPos + ticksToMove;

        telemetry.addData("Current", currentPos);
        telemetry.addData("Target", targetPos);
        telemetry.update();

        turret.setTargetPosition(targetPos);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.3);

        while (opModeIsActive() && turret.isBusy()) {
            telemetry.addData("Turret Pos", turret.getCurrentPosition());
            telemetry.update();
            sleep(10);
        }

        turret.setPower(0);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
