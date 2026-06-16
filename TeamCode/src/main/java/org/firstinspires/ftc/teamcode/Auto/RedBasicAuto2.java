package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.Commands.Launcher;
import org.firstinspires.ftc.teamcode.Commands.OTOSDriveSubsystem;
import org.firstinspires.ftc.teamcode.Commands.WheelRotation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(group = "Back", name="BackRedAuto")
public class RedBasicAuto2 extends LinearOpMode {

    private Launcher launcher;
    private OTOSDriveSubsystem drive;
    private DcMotor intake;
    private DcMotor intake2;
    private Servo hood;

    @Override
    public void runOpMode() throws InterruptedException {
        Launcher launcher = new Launcher();
        launcher.init(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive = new OTOSDriveSubsystem(hardwareMap, this);
        drive.configureOtos();
        hood = hardwareMap.get(Servo.class, "hood");

        waitForStart();
        drive.resetPose(0, 0, 0);

        //preloads
        launcher.flywheelRPMAuto(4900);
        hood.setPosition(0.18);
        sleep(4700);
        spin(-1);
        sleep(2500);
        spin(0);
        launcher.flywheelRPMAuto(1000);

        //drive forward and pick up balls at back
        intake.setPower(-0.8);
        drive.otosDrive(3,45,0);
        sleep(500);
        drive.otosDrive(3,38,0);
        sleep(100);
        drive.otosDrive(0,46,0);
        launcher.flywheelRPMAuto(5000);
        sleep(550);

        //return to shooting location
        drive.otosDrive(0,3,0);
        sleep(100);

        //shoot
        intake2.setPower(-0.8);
        sleep(2500);

        //reset
        spin(0);
        launcher.flywheelRPMAuto(1000);
        drive.resetPose(0,0,0);

        //drive forward and pick up balls at back
        intake.setPower(-0.8);
        drive.otosDrive(3,48,0);
        sleep(500);
        drive.otosDrive(3,42,0);
        sleep(100);
        drive.otosDrive(5,51,0);
        launcher.flywheelRPMAuto(5000);
        sleep(550);

        //return to shooting location
        drive.otosDrive(0,3,0);
        sleep(100);

        //shoot
        sleep(1000);
        intake2.setPower(-0.8);
        sleep(2500);

        //reset
        spin(0);
        launcher.stopFlywheel();
        drive.resetPose(0,0,0);
    }

    private void spin (double pwr) {
        intake.setPower(pwr);
        intake2.setPower(pwr);
    }
}
