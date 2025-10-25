package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;

import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="FieldCentric_AutoTest")
public class BlueBasicAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BasicMecanumDrive drive = new BasicMecanumDrive(hardwareMap);

        waitForStart();

    }
}
