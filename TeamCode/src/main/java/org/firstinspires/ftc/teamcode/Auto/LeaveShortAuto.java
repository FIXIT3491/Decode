package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.Commands.Launcher;

import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="FrontBlueAuto")
public class LeaveShortAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BasicMecanumDrive drive = new BasicMecanumDrive(hardwareMap);
        Launcher launcher = new Launcher();

        waitForStart();

        launcher.init(hardwareMap);

        sleep(10000);
        drive.drive(0.75,0,0);
        sleep(450);
        drive.drive(0,0,0);


    }
}
