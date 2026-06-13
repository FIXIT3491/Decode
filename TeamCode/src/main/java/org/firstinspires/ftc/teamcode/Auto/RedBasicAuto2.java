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
        launcher.flywheelRPM(6000);
        hood.setPosition(0.1);
        sleep(2000);
        spin(-0.9);
        sleep(1000);
        spin(0);
        sleep(1000);

        //drive forward
        intake.setPower(-0.8);
        drive.otosDrive(0,48,0);
        sleep(3000);

        //return to shooting location
        drive.otosDrive(0,0,0);
        sleep(100);

        //shoot
        launcher.flywheelRPM(6000);
        sleep(1000);
        intake2.setPower(-0.8);
        sleep(1000);

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
