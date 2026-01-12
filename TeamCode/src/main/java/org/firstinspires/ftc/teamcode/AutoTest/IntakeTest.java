package org.firstinspires.ftc.teamcode.AutoTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(group = "Test", name = "Intake Test")
public class IntakeTest extends LinearOpMode {

    private DcMotor intake;

    @Override
    public void runOpMode() throws InterruptedException {

        intake = hardwareMap.get(DcMotor.class, "intake");

        waitForStart();

        intake.setPower(0.8);
        sleep(10000);

    }

}
