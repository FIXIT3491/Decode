package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.Commands.Launcher;

import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="FrontRedAuto")
public class RedBasicAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BasicMecanumDrive drive = new BasicMecanumDrive(hardwareMap);
        Launcher launcher = new Launcher();

        waitForStart();

        launcher.init(hardwareMap);
/*
        sleep(7500);
        drive.drive(0.8,0,0);
        sleep(650);
        drive.drive(0,0,0);
        sleep(1);
        launcher.setFlywheelPower(0.9);
        sleep(500);
        launcher.openGate();
        sleep(1000);
        launcher.stop();
        launcher.closeGate();
        drive.drive(0,0,0.25);
        drive.drive(0.5,0,0);
        sleep(500);
        drive.drive(0,0,0);
*/
    }
}
