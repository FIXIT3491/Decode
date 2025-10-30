package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.Commands.Launcher;

import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="FrontBlueAuto")
public class BlueBasicAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BasicMecanumDrive drive = new BasicMecanumDrive(hardwareMap);
        Launcher launcher = new Launcher();

        waitForStart();

        launcher.init(hardwareMap);

        sleep(2500);
        drive.drive(0.5,0,0);
        sleep(900);
        drive.turnToHeading(-45, 0.25);
        sleep(3000);
        launcher.setFlywheelVelocity(1350);
        sleep(2500);
        launcher.openGate();
        sleep(1500);
        launcher.stop();
        launcher.closeGate();
        drive.turnToHeading(0,0.25);
        drive.drive(0.5,0,0);
        sleep(500);
        drive.drive(0,0,0);


    }
}
