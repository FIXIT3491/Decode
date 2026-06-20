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

@Autonomous(group = "Front", name="FrontRedAuto")
public class RedBasicAuto extends LinearOpMode {

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

        //back up and shoot preloads
        launcher.flywheelRPMAuto(3250);
        sleep(1750);
        hood.setPosition(0);
        drive.otosDrive(18,-18,0);
        launcher.turretAuto();
        sleep(100);
        spin(-1);
        sleep(2000);
        launcher.flywheelRPMAuto(1800);
        spin(0);

        //grab first spike mark
        drive.otosDrive(35,-20,0);
        sleep(100);
        intake.setPower(-1);
        drive.otosDrive(35, 3, 0);
        sleep(100);

        //go back and shoot first spike mark
        launcher.flywheelRPMAuto(3250);
        drive.otosDrive(17,-17,0);
        launcher.turretAuto();
        sleep(100);
        spin(-1);
        sleep(2000);
        launcher.flywheelRPMAuto(1800);
        spin(0);

        // get second spike mark
        intake.setPower(-1);
        drive.otosDrive(59, -20, 0);
        sleep(100);

        drive.otosDrive(59, 0, 0);
        sleep(100);

        //go back and shoot second spike mark
        launcher.flywheelRPMAuto(3250);
        drive.otosDrive(20,-20,0);
        launcher.turretAuto();
        sleep(100);
        spin(-1);
        sleep(2000);
        launcher.stopFlywheel();
        spin(0);

        drive.otosDrive(25,-5,0);

    }

    private void spin (double pwr) {
        intake.setPower(pwr);
        intake2.setPower(pwr);
    }
}
