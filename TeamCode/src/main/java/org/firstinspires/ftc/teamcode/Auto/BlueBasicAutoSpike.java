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

@Autonomous(group = "Back", name="BackBlueAutoSpike")
public class BlueBasicAutoSpike extends LinearOpMode {

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
        launcher.flywheelRPMAuto(4800);
        hood.setPosition(0.25);
        sleep(4000);
        spin(-1);
        sleep(2500);
        spin(0);
        launcher.flywheelRPMAuto(2500);

        //drive forward and pick up balls at back
        intake.setPower(-0.8);
        drive.otosDrive(3,47,0);
        sleep(100);
        drive.otosDrive(3,38,0);
        sleep(100);
        drive.otosDrive(5,48,0);
        launcher.flywheelRPMAuto(5000);
        sleep(100);

        //return to shooting location
        drive.otosDrive(0,3,0);
        sleep(100);

        //shoot
        sleep(500);
        intake2.setPower(-0.8);
        sleep(2500);

        //reset
        spin(0);
        launcher.flywheelRPMAuto(2500);
        //drive.resetPose(0,0,0);

        // get last spike mark
        intake.setPower(-0.8);
        drive.otosDrive(26,16,0);
        sleep(100);
        drive.otosDrive(26,42,0);
        sleep(100);
        launcher.flywheelRPMAuto(5000);
        sleep(500);

        //return to shooting location
        drive.otosDrive(0,3,0);
        sleep(100);

        //shoot
        sleep(200);
        intake2.setPower(-0.8);
        sleep(2500);

        //reset
        spin(0);
        launcher.stopFlywheel();
        drive.resetPose(0,0,0);

        //leave
        drive.otosDrive(20,20,0);
    }

    private void spin (double pwr) {
        intake.setPower(pwr);
        intake2.setPower(pwr);
    }
}
