package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.Commands.Launcher;
import org.firstinspires.ftc.teamcode.Commands.WheelRotation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group = "Front", name="FrontRedAuto")
public class RedBasicAuto extends LinearOpMode {

    private DcMotor intake;

    @Override
    public void runOpMode() throws InterruptedException {
        BasicMecanumDrive drive = new BasicMecanumDrive(hardwareMap);
        Launcher launcher = new Launcher();
        WheelRotation wheel = new WheelRotation();
        intake = hardwareMap.get(DcMotor.class, "intake");

        waitForStart();

        launcher.init(hardwareMap);
        wheel.init(hardwareMap);

        //move back and shoot
        launcher.kickBack();
        drive.resetHeading();
        sleep(100);
        drive.drive(0.3,0,0);
        sleep(1875);
        drive.stopMotors();
        sleep(400);

        //shoot and score - 1
        launcher.setFlywheelPower(0.45);
        sleep(500);
        launcher.kick();
        sleep(300);
        launcher.kickBack();
        sleep(500);

        //cycle
        wheel.rotateToAngle(110, 0.8);
        sleep(500);
        launcher.kick();
        sleep(300);
        launcher.kickBack();
        sleep(500);

        //cycle
        wheel.rotateToAngle(220, 0.8);
        sleep(500);
        launcher.kick();
        sleep(300);
        launcher.kickBack();
        launcher.stop();
        sleep(50);
        wheel.rotateToAngle(60, 0.8);
        sleep(250);

        //turn
        drive.turnToHeading(-45,0.45,0.5);
        sleep(250);
        drive.turnToHeading(-45,0.45, 0.5);
        sleep(250);
        drive.resetHeading();

        //intake - 1
        drive.drive(0.2,0,0);
        intake.setPower(1);
        sleep(1225);
        drive.drive(0,0,0);
        sleep(800);
        wheel.rotateToAngle(170, 0.8);
        sleep(500);

        //intake - 2
        drive.drive(0.2,0,0);
        sleep(350);
        drive.drive(0,0,0);
        sleep(800);
        wheel.rotateToAngle(280, 0.8);
        sleep(500);

        //intake - 3
        drive.drive(0.2,0,0);
        sleep(325);
        drive.drive(0,0,0);
        sleep(800);
        wheel.rotateToAngle(0, 0.8);
        sleep(500);
        intake.setPower(-1);
        sleep(500);
        intake.setPower(0);

        //aim back
        drive.drive(-0.3,0,0);
        sleep(1375);
        drive.turnToHeading(45,0.45,0.5);
        sleep(250);
        drive.turnToHeading(45,0.45,0.5);
        sleep(250);

        //shoot pt 2
        //shoot and score - 1
        launcher.setFlywheelPower(0.45);
        sleep(500);
        launcher.kick();
        sleep(300);
        launcher.kickBack();
        sleep(500);

        //cycle
        wheel.rotateToAngle(110, 0.8);
        sleep(500);
        launcher.kick();
        sleep(300);
        launcher.kickBack();
        sleep(500);

        //cycle
        wheel.rotateToAngle(220, 0.8);
        sleep(500);
        launcher.kick();
        sleep(500);
        launcher.kickBack();
        launcher.stop();
        sleep(300);
        wheel.rotateToAngle(60, 0.8);

        //turn and move back
        drive.turnToHeading(45,0.45,0.5);
        sleep(100);
        drive.resetHeading();
        drive.drive(0.4,0,0);
        sleep(750);

    }
}
