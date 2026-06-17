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

@Autonomous(group = "Front", name="FrontBlueAuto")
public class BlueBasicAuto extends LinearOpMode {

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
        launcher.flywheelRPMAuto(3500);
        hood.setPosition(0.1);
        drive.otosDrive(-35,-35,0);
        sleep(500);
        spin(1);
        sleep(2500);
        launcher.flywheelRPMAuto(1000);
        spin(0);

        //grab first spike mark
        intake.setPower(1);
        drive.otosDrive(-35, -2, 0);
        sleep(100);

        //go back and shoot first spike mark
        launcher.flywheelRPMAuto(3500);
        drive.otosDrive(-35,-35,0);
        sleep(100);
        spin(1);
        sleep(2500);
        spin(0);



    }

    private void spin (double pwr) {
        intake.setPower(pwr);
        intake2.setPower(pwr);
    }
}
