package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.Commands.Launcher;

import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group = "Back", name="BackRedAuto")
public class RedBasicAuto2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BasicMecanumDrive drive = new BasicMecanumDrive(hardwareMap);
        Launcher launcher = new Launcher();

        waitForStart();

        launcher.init(hardwareMap);

        //drive forward to shooting zone
        launcher.closeGate();
        sleep(10000);
        drive.drive(-0.45,0,0);
        sleep(1575);
        drive.drive(0,0,0);
        sleep(400);

        //turn to shoot
        drive.turnToHeading(45,0.45, 0.25);
        sleep(400);
        drive.resetHeading();
        sleep(400);

        //drive back ans shoot
        drive.drive(0.2,0,0);
        sleep(350);
        drive.drive(0,0,0);
        launcher.setFlywheelRPM(1800);
        sleep(1500);
        launcher.openGate();
        sleep(2500);
        launcher.stop();
        launcher.closeGate();
        sleep(1500);

        //turn and sweep
        drive.drive(-0.45,0,0);
        sleep(1125);
        drive.drive(0,0,0);
        sleep(400);
        drive.turnToHeading(-35,0.45,0.24);
        sleep(1000);
        drive.resetHeading();
        drive.drive(0.5,0,0);
        sleep(1400);
        drive.drive(0,0,0);
        sleep(400);

        //turn and move back slightly
        drive.turnToHeading(45,0.45,0.5);
        sleep(1000);
        drive.resetHeading();


    }
}
